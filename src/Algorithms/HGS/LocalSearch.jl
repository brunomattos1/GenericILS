# Granular local search: for each customer U, only its k nearest customers
# (algo.granularNeighbors[U]) are tried as move partners, instead of an
# exhaustive route-pair scan. Every cost-delta/violation/apply! function below
# is the exact same one src/neighborhoods/*.jl's search! already uses --
# tryMove! only supplies different (route, position) arguments, found via
# algo.customerRoute/customerPos instead of nested position loops.

function tryIntraShift!(solver::Solver, algo::HGSAlgorithm, sol::Solution, r::Int, i::Int, j::Int)
    i == j && return false
    rt = sol.routes[r]
    if i < j
        rt.feasibleF < length(rt.visits) - 1 && return false
    else
        rt.feasibleB < length(rt.visits) - 1 && return false
    end
    dist = intraShift10Cost(sol.dist, solver.data.costMatrix, rt.visits, i, j)
    cost, resViol = evalIntraShift10(solver, sol, Shift(r, r, i, j), dist)
    if resViol == 0 && cost < sol.cost - 1e-6
        sol.timeStamp += 1
        apply!(IntraShift(), solver, sol, BestMove(cost, dist, r, 0, i, j))
        rebuildRouteIndex!(solver, algo, r)
        return true
    end
    return false
end

function tryInterShiftK!(solver::Solver, algo::HGSAlgorithm, sol::Solution, r1::Int, r2::Int, i::Int, posV::Int, ::Val{k}) where {k}
    len1 = length(sol.routes[r1].visits)
    i + k - 1 > len1 - 1 && return false
    block = solver.bufferRoute
    resize!(block, k)
    copyto!(block, 1, sol.routes[r1].visits, i, k)
    for j in (posV, posV + 1)
        dist = interShiftCost(sol.dist, solver.data.costMatrix, sol.routes[r1].visits, sol.routes[r2].visits, i, j, k)
        violInfo, warpR1s1, warpR2s1, warpR1s2, warpR2s2 = computeViolInterShiftK(solver, sol, r1, r2, i, j, k, block)
        cost = objectiveValue(solver, sol, Cost(dist, r1, r2, violInfo, (warpR1s1, warpR2s1), (warpR1s2, warpR2s2)))
        if cost < sol.cost - 1e-6
            sol.timeStamp += 1
            apply!(InterShift{k}(), solver, sol, BestMove(cost, dist, r1, r2, i, j))
            rebuildRouteIndex!(solver, algo, r1)
            rebuildRouteIndex!(solver, algo, r2)
            return true
        end
    end
    return false
end

function tryInterSwap11!(solver::Solver, algo::HGSAlgorithm, sol::Solution, r1::Int, r2::Int, i::Int, j::Int)
    dist = interSwapCost(sol.dist, solver.data.costMatrix, sol.routes[r1].visits, sol.routes[r2].visits, i, j, 1, 1)
    violInfo, warpR1s1, warpR2s1, warpR1s2, warpR2s2 = computeViolInterSwapK(solver, sol, r1, r2, i, j, 1, 1)
    cost = objectiveValue(solver, sol, Cost(dist, r1, r2, violInfo, (warpR1s1, warpR2s1), (warpR1s2, warpR2s2)))
    if cost < sol.cost - 1e-6
        sol.timeStamp += 1
        apply!(InterSwap{1, 1}(), solver, sol, BestMove(cost, dist, r1, r2, i, j))
        rebuildRouteIndex!(solver, algo, r1)
        rebuildRouteIndex!(solver, algo, r2)
        return true
    end
    return false
end

# Cuts the edge right before U and the edge right before V (i = posU-1,
# j = posV-1 are always within TwoOptStar's valid range given posU/posV are
# customer positions) and reconnects the tails -- the standard 2-opt* pairing.
function tryTwoOptStar!(solver::Solver, algo::HGSAlgorithm, sol::Solution, r1::Int, r2::Int, posU::Int, posV::Int)
    i = posU - 1
    j = posV - 1
    dist = twoOptStarCost(sol.dist, solver.data.costMatrix, sol.routes[r1].visits, sol.routes[r2].visits, i, j)
    cost = evalTwoOptStar!(solver, sol, OptStar(r1, r2, i, j), dist)
    if cost < sol.cost - 1e-6
        sol.timeStamp += 1
        apply!(TwoOptStar(), solver, sol, BestMove(cost, dist, r1, r2, i, j))
        rebuildRouteIndex!(solver, algo, r1)
        rebuildRouteIndex!(solver, algo, r2)
        return true
    end
    return false
end

function tryMove!(solver::Solver, algo::HGSAlgorithm, sol::Solution, U::Int, V::Int)
    r1 = algo.customerRoute[U]
    r2 = algo.customerRoute[V]
    posU = algo.customerPos[U]
    posV = algo.customerPos[V]

    if r1 == r2
        return tryIntraShift!(solver, algo, sol, r1, posU, posV)
    end

    tryInterShiftK!(solver, algo, sol, r1, r2, posU, posV, Val(1)) && return true
    tryInterShiftK!(solver, algo, sol, r1, r2, posU, posV, Val(2)) && return true
    tryInterSwap11!(solver, algo, sol, r1, r2, posU, posV) && return true
    tryTwoOptStar!(solver, algo, sol, r1, r2, posU, posV) && return true
    return false
end

# Granular neighbor lists are customer-to-customer, so a (U, V) loop alone can
# never discover "relocate U into a currently-empty route" -- no customer
# lives there to be a V. Mirrors HGS-VRP's dedicated InterShiftToEmptyRoute
# step: try U alone into the first empty route ([0, 0], guaranteed to exist --
# see Split.jl's maxNbRoutes padding), if any.
function tryEmptyRouteRelocate!(solver::Solver, algo::HGSAlgorithm, sol::Solution, U::Int)
    r1 = algo.customerRoute[U]
    i = algo.customerPos[U]
    for r2 in 1:length(sol.routes)
        r2 == r1 && continue
        length(sol.routes[r2].visits) == 2 || continue
        # Only valid insertion point in a [0, 0] route is position 2 -- unlike
        # tryInterShiftK!'s (posV, posV+1) pair, posV=1 would be out of bounds
        # here (position 1 is the depot itself), so this is inlined rather
        # than reusing tryInterShiftK! with a bogus posV.
        j = 2
        block = solver.bufferRoute
        resize!(block, 1)
        block[1] = U
        dist = interShiftCost(sol.dist, solver.data.costMatrix, sol.routes[r1].visits, sol.routes[r2].visits, i, j, 1)
        violInfo, warpR1s1, warpR2s1, warpR1s2, warpR2s2 = computeViolInterShiftK(solver, sol, r1, r2, i, j, 1, block)
        cost = objectiveValue(solver, sol, Cost(dist, r1, r2, violInfo, (warpR1s1, warpR2s1), (warpR1s2, warpR2s2)))
        if cost < sol.cost - 1e-6
            sol.timeStamp += 1
            apply!(InterShift{1}(), solver, sol, BestMove(cost, dist, r1, r2, i, j))
            rebuildRouteIndex!(solver, algo, r1)
            rebuildRouteIndex!(solver, algo, r2)
            return true
        end
        return false
    end
    return false
end

function RVND!(solver::Solver, sol::Solution)
    algo = solver.algorithm
    indices = algo.indices
    loop = 0
    improvedAny = true
    while improvedAny
        improvedAny = false
        shuffle!(solver.seed, indices)
        for U in indices
            lastTestedU = algo.whenLastTested[U]
            improvedU = false
            for V in algo.granularNeighbors[U]
                r1 = algo.customerRoute[U]
                r2 = algo.customerRoute[V]
                if loop > 0
                    lastModif = max(sol.routes[r1].lastModif, sol.routes[r2].lastModif)
                    lastModif <= lastTestedU && continue
                end
                tryMove!(solver, algo, sol, U, V) && (improvedU = true)
            end
            tryEmptyRouteRelocate!(solver, algo, sol, U) && (improvedU = true)
            improvedU && (improvedAny = true)
            algo.whenLastTested[U] = sol.timeStamp
        end
        loop += 1
    end

    # Final polish: two exhaustive, non-granular sweeps as a safety net,
    # matching HGS-VRP's own two-pass finish -- reuses the existing shared
    # search! entirely unmodified, catches anything granularity missed.
    search!(InterShift{1}(), solver, sol)
    search!(InterSwap{1, 1}(), solver, sol)

    return nothing
end
