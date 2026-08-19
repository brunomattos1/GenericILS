# Splits a giant tour (a permutation of customer ids, no route boundaries)
# into routes, minimizing total penalized cost, using the same REFs
# (myInitStateForward/myInitStateBackward/myExtendAlongArc) already used
# everywhere else in the package -- no Segment/concatenate abstraction.
#
# DAG formulation (classical Prins/HGS Split): nodes are positions 0..n along
# the giant tour ("boundary i" = "the first i giant-tour customers are
# already split into complete routes"); an arc (i, j) for i < j represents a
# single route covering giant-tour positions i+1..j. potential[j] = min over
# i<j of potential[i] + arcCost(i, j); the answer is potential[n].
#
# Every candidate route (i, j) is, by construction, a full path starting and
# ending at the depot -- exactly what a forward (resp. backward) sweep from
# the depot already models, so arcCost(i, j) is priced via two REF sweeps:
#   - a forward sweep (fixed start i, growing j) gives feasibleF(i, j), the
#     raw distance, and the route's total warpStd1/warpStd2/labelCost
#     (mirrors the InterShift.jl apply! convention: warp/labelCost of a full
#     route come from its own forward label, no concatenation needed since
#     there is only one segment, not two partial ones being joined).
#   - a backward sweep (fixed end j, growing i downward) gives feasibleB(i, j).
# infeas(i, j) = totalArcs - max(feasibleF(i, j), feasibleB(i, j)), exactly
# the sigma_P estimate already used by src/Infeasibility.jl for local-search
# moves. No route is ever rejected as infeasible -- every candidate gets a
# finite, penalty-weighted cost, so any giant tour permutation always splits.
#
# Both sweeps are O(n) per fixed start/end, O(n^2) total (same complexity
# class as the reference HGS-VRP Split, which is also O(n^2), not the
# linear capacity-only trick -- that trick only applies to a scalar concave
# capacity cost and does not generalize to arbitrary REFs).

mutable struct SplitBuffers
    potential::Vector{Float64}
    pred::Vector{Int}
    feasibleB::Matrix{Int}
    bwdLabelCost::Matrix{Float64}
    boundaries::Vector{Int}
end

SplitBuffers() = SplitBuffers(Float64[], Int[], Matrix{Int}(undef, 0, 0), Matrix{Float64}(undef, 0, 0), Int[])

function resizeSplitBuffers!(sb::SplitBuffers, n::Int)
    resize!(sb.potential, n + 1)
    resize!(sb.pred, n + 1)
    if size(sb.feasibleB) != (n, n)
        sb.feasibleB = Matrix{Int}(undef, n, n)
        sb.bwdLabelCost = Matrix{Float64}(undef, n, n)
    end
    return sb
end

function splitGiantTour!(solver::Solver, ws::Solution{FL, BL}, giantTour::Vector{Int}) where {FL, BL}
    n = length(giantTour)
    res = solver.res
    costMatrix = solver.data.costMatrix
    sb = solver.algorithm.split
    resizeSplitBuffers!(sb, n)

    # ---- backward sweep: feasibleB[i+1, j], bwdLabelCost[i+1, j] for 0<=i<j<=n ----
    for j in 1:n
        lbl = myInitStateBackward(res.customResource)
        firstInfeasibleArc = 0
        arcCount = 0
        frontId = 0
        for boundary in (j - 1):-1:0
            newFrontId = giantTour[boundary + 1]
            if boundary == j - 1
                lbl = myExtendAlongArc(res, lbl, (1, newFrontId + 1))
            else
                lbl = myExtendAlongArc(res, lbl, (frontId + 1, newFrontId + 1))
            end
            arcCount += 1
            if firstInfeasibleArc == 0 && lbl.cost == Inf
                firstInfeasibleArc = arcCount
            end

            openLbl = myExtendAlongArc(res, lbl, (newFrontId + 1, 1))
            openArcCount = arcCount + 1
            openFirstInfeasible = (firstInfeasibleArc == 0 && openLbl.cost == Inf) ? openArcCount : firstInfeasibleArc
            feasibleB = openFirstInfeasible == 0 ? openArcCount : openFirstInfeasible - 1

            sb.feasibleB[boundary + 1, j] = feasibleB
            sb.bwdLabelCost[boundary + 1, j] = openLbl.cost

            frontId = newFrontId
        end
    end

    # ---- forward sweep + DP relaxation ----
    pm = solver.penaltyManager
    costIsRelevant = isCostResource()
    sb.potential[1] = 0.0
    sb.pred[1] = -1
    for k in 2:(n + 1)
        sb.potential[k] = Inf
        sb.pred[k] = -1
    end

    for i in 0:(n - 1)
        sb.potential[i + 1] == Inf && continue
        lbl = myInitStateForward(res.customResource)
        firstInfeasibleArc = 0
        arcCount = 0
        prevId = 0
        dist = 0.0
        for j in (i + 1):n
            curId = giantTour[j]
            lbl = myExtendAlongArc(res, lbl, (prevId + 1, curId + 1))
            arcCount += 1
            if firstInfeasibleArc == 0 && lbl.cost == Inf
                firstInfeasibleArc = arcCount
            end
            dist += costMatrix[prevId + 1, curId + 1]

            closeLbl = myExtendAlongArc(res, lbl, (curId + 1, 1))
            closeArcCount = arcCount + 1
            closeFirstInfeasible = (firstInfeasibleArc == 0 && closeLbl.cost == Inf) ? closeArcCount : firstInfeasibleArc
            feasibleF = closeFirstInfeasible == 0 ? closeArcCount : closeFirstInfeasible - 1
            closeDist = dist + costMatrix[curId + 1, 1]

            totalArcs = j - i + 1
            infeas = totalArcs - max(feasibleF, sb.feasibleB[i + 1, j])
            warp1 = closeLbl.std1State.stdWarp
            warp2 = closeLbl.std2State.stdWarp
            labelCost = min(closeLbl.cost, sb.bwdLabelCost[i + 1, j])

            arcCost = closeDist
            if costIsRelevant
                arcCost += labelCost
            end
            arcCost += pm.penaltyCustom * infeas + pm.penaltyStandard1 * warp1 + pm.penaltyStandard2 * warp2

            cand = sb.potential[i + 1] + arcCost
            if cand < sb.potential[j + 1]
                sb.potential[j + 1] = cand
                sb.pred[j + 1] = i
            end

            prevId = curId
        end
    end

    # ---- reconstruct route boundaries from pred ----
    boundaries = sb.boundaries
    empty!(boundaries)
    k = n
    while k > 0
        push!(boundaries, k)
        k = sb.pred[k + 1]
    end
    push!(boundaries, 0)
    reverse!(boundaries)

    numRoutes = length(boundaries) - 1
    # Pad with empty [0, 0] routes up to maxNbRoutes, mirroring bestParallelInsertion's
    # slack convention -- without at least one genuinely empty route slot, RVND's
    # neighborhoods (which can insert into an existing empty route, but never create
    # one out of thin air) can never grow the number of active routes.
    totalRoutes = max(numRoutes, solver.data.maxNbRoutes)
    sol = ws
    resize!(sol.routes, totalRoutes)
    for r in 1:numRoutes
        if !isassigned(sol.routes, r)
            sol.routes[r] = Route{FL, BL}()
        end
        i = boundaries[r]
        j = boundaries[r + 1]
        rt = sol.routes[r]
        resize!(rt.visits, j - i + 2)
        rt.visits[1] = 0
        for m in 1:(j - i)
            rt.visits[m + 1] = giantTour[i + m]
        end
        rt.visits[end] = 0
    end
    for r in (numRoutes + 1):totalRoutes
        if !isassigned(sol.routes, r)
            sol.routes[r] = Route{FL, BL}()
        end
        rt = sol.routes[r]
        resize!(rt.visits, 2)
        rt.visits[1] = 0
        rt.visits[2] = 0
    end

    computeLabels(solver, sol)

    totalInfeas = 0
    totalWarp1 = 0.0
    totalWarp2 = 0.0
    totalLabelCost = 0.0
    totalDist = 0.0
    for r in 1:totalRoutes
        rt = sol.routes[r]
        rt.infeas = length(rt.visits) - max(rt.feasibleF, rt.feasibleB) - 1
        rt.warpStd1 = rt.forwardLabels[end].std1State.stdWarp
        rt.warpStd2 = rt.forwardLabels[end].std2State.stdWarp
        rt.labelCost = min(rt.forwardLabels[end].cost, rt.backwardLabels[end].cost)
        rt.lastModif = 0

        distR = 0.0
        for m in 1:(length(rt.visits) - 1)
            distR += costMatrix[rt.visits[m] + 1, rt.visits[m + 1] + 1]
        end

        totalInfeas += rt.infeas
        totalWarp1 += rt.warpStd1
        totalWarp2 += rt.warpStd2
        totalLabelCost += rt.labelCost
        totalDist += distR
    end

    sol.totalInfeas = totalInfeas
    sol.totalWarpStd1 = totalWarp1
    sol.totalWarpStd2 = totalWarp2
    sol.totalLabelCost = totalLabelCost
    sol.dist = totalDist
    sol.cost = objectiveValue(solver, sol)

    nVizinhas = length(solver.neighborhoods)
    sol.timeStamp = 0
    # ws is reused across every generation and totalRoutes stabilizes at
    # maxNbRoutes after the first call, so reallocating lastEval every Split
    # would waste an nVizinhas*totalRoutes^2-int allocation (~13KB on a
    # 151-customer instance) every single generation for no reason.
    if size(sol.lastEval) == (nVizinhas, totalRoutes, totalRoutes)
        fill!(sol.lastEval, 0)
    else
        sol.lastEval = zeros(Int, nVizinhas, totalRoutes, totalRoutes)
    end

    return ws
end
