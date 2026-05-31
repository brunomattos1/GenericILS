struct TwoOptStar end

function twoOptStarCost(currCost::Float64, costMatrix::Matrix{Float64}, route1::Vector{Int}, route2::Vector{Int}, i::Int, j::Int)
    newCost = currCost - costMatrix[route1[i]+1, route1[i+1]+1] - costMatrix[route2[j]+1, route2[j+1]+1]
    newCost += costMatrix[route1[i]+1, route2[j+1]+1] + costMatrix[route2[j]+1, route1[i+1]+1]
    return newCost
end

function computeViolTwoOptStar(solver::Solver, sol::Solution, r1::Int, r2::Int, i::Int, j::Int)
    infeasR1, labelCostR1 = infeasArcs2optStar(solver, sol, r1, r2, i, j)
    infeasR2, labelCostR2 = infeasArcs2optStar(solver, sol, r2, r1, j, i)
    return ViolationInfo(infeasR1, infeasR2, labelCostR1, labelCostR2)
end

function evalTwoOptStar!(solver::Solver, sol::Solution, move::OptStar)
    r1, r2 = move.firstRoute, move.secondRoute
    i,  j  = move.firstIdx,   move.secondIdx
    dist     = twoOptStarCost(sol.dist, solver.data.costMatrix, sol.routes[r1], sol.routes[r2], i, j)
    warpR1Std1, warpR1Std2, warpR2Std1, warpR2Std2 = computeStdViolTwoOptStar(solver, sol, r1, r2, i, j)
    violInfo = computeViolTwoOptStar(solver, sol, r1, r2, i, j)
    cost = objectiveValue(solver, sol, Cost(dist, r1, r2, violInfo, (warpR1Std1, warpR2Std1), (warpR1Std2, warpR2Std2)))
    return dist, cost, violInfo.firstRouteInfeas, violInfo.secondRouteInfeas, warpR1Std1, warpR1Std2, warpR2Std1, warpR2Std2
end

function search!(neigh::TwoOptStar, solver::Solver, sol::Solution)
    improved = false
    copy_solution!(solver.bufferSol, sol)
    oldSol = solver.bufferSol
    resize!(solver.buffer, length(sol.routes))
    copyto!(solver.buffer, 1:length(sol.routes))
    shuffle!(solver.buffer)
    routesIdx = solver.buffer
    neighborhoodId = neigh_index(typeof(neigh))

    for r1 in routesIdx
        bestMove = BestMove(cost = sol.cost, dist = sol.dist)
        for r2 in routesIdx
            r1 == r2 && continue
            lastEval = sol.lastEval[neighborhoodId, r1, r2]
            lastEval > max(sol.lastModif[r1], sol.lastModif[r2]) && continue
            for i = 1:length(sol.routes[r1]) - 2
                for j = 1:length(sol.routes[r2]) - 2
                    dist, cost, infeasR1, infeasR2, warpR1Std1, warpR1Std2, warpR2Std1, warpR2Std2 = evalTwoOptStar!(solver, sol, OptStar(r1, r2, i, j))
                    if cost < bestMove.cost - 1e-6
                        bestMove = BestMove(cost, dist, r1, r2, i, j, (infeasR1, infeasR2), (warpR1Std1, warpR1Std2), (warpR2Std1, warpR2Std2))
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

    solution.dist = move.dist
    solution.cost = move.cost

    solution.totalInfeas -= solution.infeas[r1] + solution.infeas[r2]
    solution.totalInfeas += move.infeas[1] + move.infeas[2]
    solution.infeas[r1] = move.infeas[1]
    solution.infeas[r2] = move.infeas[2]

    seg1_len = length(solution.routes[r1]) - i
    seg2_len = length(solution.routes[r2]) - j

    buffer = solver.buffer2opt
    resize!(buffer, seg1_len)
    copyto!(buffer, 1, solution.routes[r1], i+1, seg1_len)

    resize!(solution.routes[r1], i + seg2_len)
    copyto!(solution.routes[r1], i+1, solution.routes[r2], j+1, seg2_len)

    resize!(solution.routes[r2], j + seg1_len)
    copyto!(solution.routes[r2], j+1, buffer, 1, seg1_len)

    computeLabels(solver, solution, r1, r2)

    solution.totalInfeas -= solution.infeas[r1] + solution.infeas[r2]
    solution.infeas[r1] = length(solution.routes[r1]) - max(solution.feasiblesF[r1], solution.feasiblesB[r1]) - 1
    solution.infeas[r2] = length(solution.routes[r2]) - max(solution.feasiblesF[r2], solution.feasiblesB[r2]) - 1
    solution.totalInfeas += solution.infeas[r1] + solution.infeas[r2]

    solution.totalWarpStd1 -= solution.warpsStd1[r1] + solution.warpsStd1[r2]
    solution.warpsStd1[r1] = solution.forwardLabels[r1][end].std1State.stdWarp
    solution.warpsStd1[r2] = solution.forwardLabels[r2][end].std1State.stdWarp
    solution.totalWarpStd1 += solution.warpsStd1[r1] + solution.warpsStd1[r2]

    solution.totalWarpStd2 -= solution.warpsStd2[r1] + solution.warpsStd2[r2]
    solution.warpsStd2[r1] = solution.forwardLabels[r1][end].std2State.stdWarp
    solution.warpsStd2[r2] = solution.forwardLabels[r2][end].std2State.stdWarp
    solution.totalWarpStd2 += solution.warpsStd2[r1] + solution.warpsStd2[r2]

    solution.totalLabelCost -= solution.labelCosts[r1] + solution.labelCosts[r2]
    solution.labelCosts[r1] = min(solution.forwardLabels[r1][end].cost, solution.backwardLabels[r1][end].cost)
    solution.labelCosts[r2] = min(solution.forwardLabels[r2][end].cost, solution.backwardLabels[r2][end].cost)
    solution.totalLabelCost += solution.labelCosts[r1] + solution.labelCosts[r2]

    solution.cost = objectiveValue(solver, solution)
    solution.lastModif[r1] = solution.timeStamp
    solution.lastModif[r2] = solution.timeStamp
end
