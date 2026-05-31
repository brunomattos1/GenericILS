struct InterSwap{k1,k2} end

function interSwapCost(currCost::Float64, costMatrix::Matrix{Float64}, route1::Vector{Int}, route2::Vector{Int}, i::Int, j::Int, k1::Int, k2::Int)
    newCost = currCost
    newCost -= costMatrix[route1[i-1]+1,    route1[i]+1]
    newCost -= costMatrix[route1[i+k1-1]+1, route1[i+k1]+1]
    newCost -= costMatrix[route2[j-1]+1,    route2[j]+1]
    newCost -= costMatrix[route2[j+k2-1]+1, route2[j+k2]+1]
    newCost += costMatrix[route1[i-1]+1,    route2[j]+1]
    newCost += costMatrix[route2[j+k2-1]+1, route1[i+k1]+1]
    newCost += costMatrix[route2[j-1]+1,    route1[i]+1]
    newCost += costMatrix[route1[i+k1-1]+1, route2[j+k2]+1]
    return newCost
end

function computeViolInterSwapK(solver::Solver, sol::Solution, r1::Int, r2::Int, i::Int, j::Int, k1::Int, k2::Int)
    block1 = sol.routes[r1][i:i+k1-1]
    block2 = sol.routes[r2][j:j+k2-1]
    infeasR1, lc1 = infeasArcsReplaceBlockK(solver, sol, r1, i, k1, block2)
    infeasR2, lc2 = infeasArcsReplaceBlockK(solver, sol, r2, j, k2, block1)
    return ViolationInfo(infeasR1, infeasR2, lc1, lc2)
end

function search!(neigh::InterSwap{k1,k2}, solver::Solver, sol::Solution) where {k1,k2}
    flag = false
    copy_solution!(solver.bufferSol, sol)
    oldSol = solver.bufferSol
    neighborhoodId = neigh_index(InterSwap{k1,k2})
    resize!(solver.buffer, length(sol.routes))
    copyto!(solver.buffer, 1:length(sol.routes))
    shuffle!(solver.seed, solver.buffer)
    routesIdx = solver.buffer

    for r1 in routesIdx
        bestMove = BestMove(dist = sol.dist, cost = sol.cost)
        route1 = sol.routes[r1]
        len1   = length(route1)
        len1 <= k1 + 1 && continue

        for r2 in routesIdx
            r2 == r1 && continue
            k1 == k2 && r1 >= r2 && continue
            route2 = sol.routes[r2]
            len2   = length(route2)
            len2 <= k2 + 1 && continue
            sol.lastEval[neighborhoodId, r1, r2] > max(sol.lastModif[r1], sol.lastModif[r2]) && continue

            for i = 2:(len1 - k1)
                for j = 2:(len2 - k2)
                    dist     = interSwapCost(sol.dist, solver.data.costMatrix, route1, route2, i, j, k1, k2)
                    warpR1s1, warpR1s2 = computeStdViolRemoveK(solver, sol, r1, i, k1)
                    warpR2s1, warpR2s2 = computeStdViolRemoveK(solver, sol, r2, j, k2)
                    violInfo = computeViolInterSwapK(solver, sol, r1, r2, i, j, k1, k2)
                    cost = objectiveValue(solver, sol,
                        Cost(dist, r1, r2, violInfo, (warpR1s1, warpR2s1), (warpR1s2, warpR2s2)))
                    if cost < bestMove.cost - 1e-6
                        bestMove = BestMove(cost, dist, r1, r2, i, j,
                            (violInfo.firstRouteInfeas, violInfo.secondRouteInfeas),
                            (warpR1s1, warpR1s2), (warpR2s1, warpR2s2))
                    end
                end
            end
            # sol.timeStamp += 1
            # sol.lastEval[neighborhoodId, r1, r2] = sol.timeStamp
        end

        if bestMove.firstIdx > 0
            prevCost = oldSol.cost
            apply!(neigh, solver, sol, bestMove)
            if sol.cost < prevCost - 1e-6
                flag = true
                copy_solution!(oldSol, sol)
            else
                copy_solution!(sol, oldSol)
            end
        end
    end
    return flag
end

function apply!(::InterSwap{k1,k2}, solver::Solver, sol::Solution, bestMove::BestMove) where {k1,k2}
    r1, r2 = bestMove.firstRoute, bestMove.secondRoute
    i,  j  = bestMove.firstIdx,   bestMove.secondIdx

    sol.dist = bestMove.dist
    sol.cost = bestMove.cost

    sol.totalInfeas -= sol.infeas[r1] + sol.infeas[r2]
    sol.totalInfeas += bestMove.infeas[1] + bestMove.infeas[2]
    sol.infeas[r1] = bestMove.infeas[1]
    sol.infeas[r2] = bestMove.infeas[2]

    block1 = sol.routes[r1][i:i+k1-1]
    block2 = sol.routes[r2][j:j+k2-1]
    splice!(sol.routes[r1], i:i+k1-1, block2)
    splice!(sol.routes[r2], j:j+k2-1, block1)

    computeLabels(solver, sol, r1, r2)

    sol.totalInfeas -= sol.infeas[r1] + sol.infeas[r2]
    sol.infeas[r1] = length(sol.routes[r1]) - max(sol.feasiblesF[r1], sol.feasiblesB[r1]) - 1
    sol.infeas[r2] = length(sol.routes[r2]) - max(sol.feasiblesF[r2], sol.feasiblesB[r2]) - 1
    sol.totalInfeas += sol.infeas[r1] + sol.infeas[r2]

    sol.totalWarpStd1 -= sol.warpsStd1[r1] + sol.warpsStd1[r2]
    sol.warpsStd1[r1] = sol.forwardLabels[r1][end].std1State.stdWarp
    sol.warpsStd1[r2] = sol.forwardLabels[r2][end].std1State.stdWarp
    sol.totalWarpStd1 += sol.warpsStd1[r1] + sol.warpsStd1[r2]

    sol.totalWarpStd2 -= sol.warpsStd2[r1] + sol.warpsStd2[r2]
    sol.warpsStd2[r1] = sol.forwardLabels[r1][end].std2State.stdWarp
    sol.warpsStd2[r2] = sol.forwardLabels[r2][end].std2State.stdWarp
    sol.totalWarpStd2 += sol.warpsStd2[r1] + sol.warpsStd2[r2]

    sol.totalLabelCost -= sol.labelCosts[r1] + sol.labelCosts[r2]
    sol.labelCosts[r1] = min(sol.forwardLabels[r1][end].cost, sol.backwardLabels[r1][end].cost)
    sol.labelCosts[r2] = min(sol.forwardLabels[r2][end].cost, sol.backwardLabels[r2][end].cost)
    sol.totalLabelCost += sol.labelCosts[r1] + sol.labelCosts[r2]

    sol.cost = objectiveValue(solver, sol)
    solver.timeStamp += 1
    sol.lastModif[r1] = solver.timeStamp
    sol.lastModif[r2] = solver.timeStamp
    sol.lastEval[neigh_index(InterSwap{k1,k2}), r1, r2] = solver.timeStamp
end
