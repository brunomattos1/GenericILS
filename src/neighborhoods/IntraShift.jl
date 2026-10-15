struct IntraShift end

# Move customer `route[i]` inside its route:
#   * i < j: it goes right AFTER `route[j]` (j == i+1 is a swap of two adjacent customers);
#   * i > j: it goes right BEFORE `route[j]`.
function intraShift10Cost(currCost::Float64, costMatrix::Matrix{Float64}, route::Vector{Int}, i::Int, j::Int)
    if i < j
        newCost = currCost - costMatrix[route[i-1]+1, route[i]+1] - costMatrix[route[i]+1, route[i+1]+1] - costMatrix[route[j]+1, route[j+1]+1]
        newCost += costMatrix[route[i-1]+1, route[i+1]+1] + costMatrix[route[j]+1, route[i]+1] + costMatrix[route[i]+1, route[j+1]+1]
    else
        newCost = currCost - costMatrix[route[i-1]+1, route[i]+1] - costMatrix[route[i]+1, route[i+1]+1] - costMatrix[route[j-1]+1, route[j]+1]
        newCost += costMatrix[route[i-1]+1, route[i+1]+1] + costMatrix[route[j-1]+1, route[i]+1] + costMatrix[route[i]+1, route[j]+1]
    end
    return newCost
end

# The labels of the route WITHOUT customer i are built incrementally while j sweeps the route
# (`solver.prevLabelF` going forward, `solver.prevLabelB` going backward). This must be called for
# every j, including those pruned by distance, or the chain would be left behind.
function advanceIntraShift10!(solver::Solver, sol::Solution, r::Int, i::Int, j::Int)
    rt = sol.routes[r]
    if i < j
        if j == i+1
            solver.prevLabelF = myExtendAlongArc(solver.res, rt.forwardLabels[i-1], (rt.visits[i-1]+1, rt.visits[i+1] + 1))
        else
            solver.prevLabelF = myExtendAlongArc(solver.res, solver.prevLabelF, (solver.prevLabelF.last+1, rt.visits[j] + 1))
        end
    else
        if j == i-1
            solver.prevLabelB = myExtendAlongArc(solver.res, rt.backwardLabels[length(rt.visits) - i], (rt.visits[i+1]+1, rt.visits[i-1] + 1))
        else
            solver.prevLabelB = myExtendAlongArc(solver.res, solver.prevLabelB, (solver.prevLabelB.last+1, rt.visits[j] + 1))
        end
    end
    return nothing
end

# Requires `advanceIntraShift10!(solver, sol, r, i, j)` to have been called for this (i, j).
function computeViolIntraShift10(solver::Solver, sol::Solution, r::Int, i::Int, j::Int)
    rt = sol.routes[r]
    if i < j
        # prefix without i up to j, then i, then j+1; joined with the suffix after j+1
        auxLabel = myExtendAlongArc(solver.res, solver.prevLabelF, (solver.prevLabelF.last+1, rt.visits[i] + 1))
        auxLabel = myExtendAlongArc(solver.res, auxLabel, (auxLabel.last + 1, rt.visits[j+1] + 1))
        bwdLabel = rt.backwardLabels[length(rt.visits) - j]
        res = myConcatenationCost(solver.res, bwdLabel.last, auxLabel, bwdLabel)
    else
        # suffix without i from j, then i before j, then j-1; joined with the prefix up to j-2
        auxLabel = myExtendAlongArc(solver.res, solver.prevLabelB, (rt.visits[j]+1, rt.visits[i] + 1))
        auxLabel = myExtendAlongArc(solver.res, auxLabel, (rt.visits[i]+1, rt.visits[j-1] + 1))
        res = myConcatenationCost(solver.res, auxLabel.last, rt.forwardLabels[j - 1], auxLabel)
    end
    warpStd1, warpStd2 = res.std1State.stdWarp, res.std2State.stdWarp
    if res.cost >= Inf
        return 1, res.cost, warpStd1, warpStd2
    else
        return 0, res.cost, warpStd1, warpStd2
    end
end

function evalIntraShift10(solver::Solver, sol::Solution, shift::Shift, dist::Float64)
    r = shift.routeFrom
    i = shift.fromIdx
    j = shift.toIdx
    resViol, labelCost, warpStd1, warpStd2 = computeViolIntraShift10(solver, sol, r, i, j)
    cost = objectiveValue(solver, sol, r, dist, 0, labelCost, warpStd1, warpStd2)
    return cost, resViol
end

function search!(neigh::IntraShift, solver::Solver, sol::Solution)
    improved = false
    copySolution!(solver.bufferSol, sol)
    oldSol = solver.bufferSol
    neighborhoodId = neighIndex(typeof(solver), typeof(neigh))
    for r = 1:length(sol.routes)
        rt = sol.routes[r]
        if sol.lastEval[neighborhoodId, r, r] > rt.lastModif
            continue
        end
        bestMove = BestMove(cost = sol.cost, dist = sol.dist)
        canPrune = canPruneByDist(solver)
        fixedPenalty = canPrune ? pruningFixedPenalty(solver, sol, r) : 0.0
        if rt.feasibleF >= length(rt.visits) - 1
            for i = 2:length(rt.visits)-1
                for j = i+1:length(rt.visits)-1
                    advanceIntraShift10!(solver, sol, r, i, j)
                    dist = intraShift10Cost(sol.dist, solver.data.costMatrix, rt.visits, i, j)
                    if canPrune && dist + fixedPenalty >= bestMove.cost - 1e-6
                        continue
                    end
                    cost, resViol = evalIntraShift10(solver, sol, Shift(r, r, i, j), dist)
                    if resViol == 0 && cost < bestMove.cost - 1e-6
                        bestMove = BestMove(cost, dist, r, 0, i, j)
                    end
                end
            end
        end
        if rt.feasibleB >= length(rt.visits) - 1
            for i = length(rt.visits)-1:-1:2
                for j = i-1:-1:2
                    advanceIntraShift10!(solver, sol, r, i, j)
                    dist = intraShift10Cost(sol.dist, solver.data.costMatrix, rt.visits, i, j)
                    if canPrune && dist + fixedPenalty >= bestMove.cost - 1e-6
                        continue
                    end
                    cost, resViol = evalIntraShift10(solver, sol, Shift(r, r, i, j), dist)
                    if resViol == 0 && cost < bestMove.cost - 1e-6
                        bestMove = BestMove(cost, dist, r, 0, i, j)
                    end
                end
            end
        end
        sol.timeStamp += 1
        sol.lastEval[neighborhoodId, r, r] = sol.timeStamp
        if bestMove.firstIdx > 0
            prevCost = oldSol.cost
            apply!(neigh, solver, sol, bestMove)
            if sol.cost < prevCost - 1e-6
                improved = true
                copySolution!(oldSol, sol)
            else
                copySolution!(sol, oldSol)
            end
        end
    end
    return improved
end

function apply!(::IntraShift, solver::Solver, solution::Solution, move::BestMove)
    r  = move.firstRoute
    i, j = move.firstIdx, move.secondIdx
    rt = solution.routes[r]

    solution.dist            = move.dist
    solution.totalInfeas    -= rt.infeas
    solution.totalWarpStd1  -= rt.warpStd1
    solution.totalWarpStd2  -= rt.warpStd2
    solution.totalLabelCost -= rt.labelCost

    route = rt.visits
    if i < j
        moveBlocksIntra!(route, i, 1, j + 1, solver.bufferRoute)   # lands right after the old route[j]
    else
        moveBlocksIntra!(route, i, 1, j, solver.bufferRoute)       # lands right before the old route[j]
    end

    computeLabels(solver, solution, r)

    rt.infeas = length(rt.visits) - max(rt.feasibleF, rt.feasibleB) - 1
    solution.totalInfeas += rt.infeas

    rt.warpStd1 = rt.forwardLabels[end].std1State.stdWarp
    solution.totalWarpStd1 += rt.warpStd1

    rt.warpStd2 = rt.forwardLabels[end].std2State.stdWarp
    solution.totalWarpStd2 += rt.warpStd2

    rt.labelCost = min(rt.forwardLabels[end].cost, rt.backwardLabels[end].cost)
    solution.totalLabelCost += rt.labelCost

    solution.cost = objectiveValue(solver, solution)
    rt.lastModif = solution.timeStamp
end
