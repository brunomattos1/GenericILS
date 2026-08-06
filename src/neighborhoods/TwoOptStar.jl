struct TwoOptStar end

function twoOptStarCost(currCost::Float64, costMatrix::Matrix{Float64}, route1::Vector{Int}, route2::Vector{Int}, i::Int, j::Int)
    newCost = currCost - costMatrix[route1[i]+1, route1[i+1]+1] - costMatrix[route2[j]+1, route2[j+1]+1]
    newCost += costMatrix[route1[i]+1, route2[j+1]+1] + costMatrix[route2[j]+1, route1[i+1]+1]
    return newCost
end

function computeViolTwoOptStar(solver::Solver, sol::Solution, r1::Int, r2::Int, i::Int, j::Int)
    infeasR1, labelCostR1, warpR1Std1, warpR1Std2 = infeasArcs2optStar(solver, sol, r1, r2, i, j)
    infeasR2, labelCostR2, warpR2Std1, warpR2Std2 = infeasArcs2optStar(solver, sol, r2, r1, j, i)
    violInfo = ViolationInfo(infeasR1, infeasR2, labelCostR1, labelCostR2)
    return violInfo, warpR1Std1, warpR2Std1, warpR1Std2, warpR2Std2
end

function evalTwoOptStar!(solver::Solver, sol::Solution, move::OptStar, dist::Float64)
    r1, r2 = move.firstRoute, move.secondRoute
    i,  j  = move.firstIdx,   move.secondIdx
    violInfo, warpR1Std1, warpR2Std1, warpR1Std2, warpR2Std2 = computeViolTwoOptStar(solver, sol, r1, r2, i, j)
    cost = objectiveValue(solver, sol, Cost(dist, r1, r2, violInfo, (warpR1Std1, warpR2Std1), (warpR1Std2, warpR2Std2)))
    return cost
end

function search!(neigh::TwoOptStar, solver::Solver, sol::Solution)
    improved = false
    copy_solution!(solver.bufferSol, sol)
    oldSol = solver.bufferSol
    resize!(solver.buffer, length(sol.routes))
    copyto!(solver.buffer, 1:length(sol.routes))
    shuffle!(solver.seed, solver.buffer)
    routesIdx = solver.buffer
    neighborhoodId = neigh_index(typeof(solver), typeof(neigh))

    for r1 in routesIdx
        bestMove = BestMove(cost = sol.cost, dist = sol.dist)
        for r2 in routesIdx
            r1 == r2 && continue
            lastEval = sol.lastEval[neighborhoodId, r1, r2]
            lastEval > max(sol.routes[r1].lastModif, sol.routes[r2].lastModif) && continue
            canPrune = canPruneByDist(solver)
            fixedPenalty = canPrune ? pruningFixedPenalty(solver, sol, r1, r2) : 0.0
            for i = 1:length(sol.routes[r1].visits) - 2
                for j = 1:length(sol.routes[r2].visits) - 2
                    dist = twoOptStarCost(sol.dist, solver.data.costMatrix, sol.routes[r1].visits, sol.routes[r2].visits, i, j)
                    if canPrune && dist + fixedPenalty >= bestMove.cost - 1e-6
                        continue
                    end
                    cost = evalTwoOptStar!(solver, sol, OptStar(r1, r2, i, j), dist)
                    if cost < bestMove.cost - 1e-6
                        bestMove = BestMove(cost, dist, r1, r2, i, j)
                    end
                end
            end
            sol.timeStamp += 1
            sol.lastEval[neighborhoodId, r1, r2] = sol.timeStamp
        end
        if bestMove.firstIdx > 0
            prevCost = oldSol.cost
            apply!(neigh, solver, sol, bestMove)
            if sol.cost < prevCost - 1e-6
                improved = true
                copy_solution!(oldSol, sol)
            else
                copy_solution!(sol, oldSol)
            end
        end
    end
    return improved
end

function apply!(neigh::TwoOptStar, solver::Solver, solution::Solution, move::BestMove)
    r1, r2 = move.firstRoute, move.secondRoute
    i,  j  = move.firstIdx,   move.secondIdx
    rt1 = solution.routes[r1]
    rt2 = solution.routes[r2]

    solution.dist            = move.dist
    solution.totalInfeas    -= rt1.infeas    + rt2.infeas
    solution.totalWarpStd1  -= rt1.warpStd1 + rt2.warpStd1
    solution.totalWarpStd2  -= rt1.warpStd2 + rt2.warpStd2
    solution.totalLabelCost -= rt1.labelCost + rt2.labelCost

    v1 = rt1.visits
    v2 = rt2.visits
    seg1_len = length(v1) - i
    seg2_len = length(v2) - j

    buffer = solver.buffer2opt
    resize!(buffer, seg1_len)
    copyto!(buffer, 1, v1, i+1, seg1_len)

    resize!(v1, i + seg2_len)
    copyto!(v1, i+1, v2, j+1, seg2_len)

    resize!(v2, j + seg1_len)
    copyto!(v2, j+1, buffer, 1, seg1_len)

    computeLabels(solver, solution, r1, r2)

    rt1.infeas = length(v1) - max(rt1.feasibleF, rt1.feasibleB) - 1
    rt2.infeas = length(v2) - max(rt2.feasibleF, rt2.feasibleB) - 1
    solution.totalInfeas += rt1.infeas + rt2.infeas

    rt1.warpStd1 = rt1.forwardLabels[end].std1State.stdWarp
    rt2.warpStd1 = rt2.forwardLabels[end].std1State.stdWarp
    solution.totalWarpStd1 += rt1.warpStd1 + rt2.warpStd1

    rt1.warpStd2 = rt1.forwardLabels[end].std2State.stdWarp
    rt2.warpStd2 = rt2.forwardLabels[end].std2State.stdWarp
    solution.totalWarpStd2 += rt1.warpStd2 + rt2.warpStd2

    rt1.labelCost = min(rt1.forwardLabels[end].cost, rt1.backwardLabels[end].cost)
    rt2.labelCost = min(rt2.forwardLabels[end].cost, rt2.backwardLabels[end].cost)
    solution.totalLabelCost += rt1.labelCost + rt2.labelCost

    solution.cost = objectiveValue(solver, solution)
    rt1.lastModif = solution.timeStamp
    rt2.lastModif = solution.timeStamp
end
