struct InterShift{k} end

function interShiftCost(currCost::Float64, costMatrix::Matrix{Float64}, route1::Vector{Int}, route2::Vector{Int}, i::Int, j::Int, k::Int)
    newCost = currCost
    newCost -= costMatrix[route1[i-1]+1,   route1[i]+1]
    newCost -= costMatrix[route1[i+k-1]+1, route1[i+k]+1]
    newCost -= costMatrix[route2[j-1]+1,   route2[j]+1]
    newCost += costMatrix[route1[i-1]+1,   route1[i+k]+1]
    newCost += costMatrix[route2[j-1]+1,   route1[i]+1]
    newCost += costMatrix[route1[i+k-1]+1, route2[j]+1]
    return newCost
end

function computeViolInterShiftK(solver::Solver, sol::Solution, r1::Int, r2::Int, i::Int, j::Int, k::Int)
    infeasR1, lc1 = infeasArcsRemovalK(solver, sol, r1, i, k)
    customers = @view sol.routes[r1][i:i+k-1]
    infeasR2, lc2 = infeasArcsInsertionK(solver, sol, r2, customers, j)
    return ViolationInfo(infeasR1, infeasR2, lc1, lc2)
end

function search!(neigh::InterShift{k}, solver::Solver, sol::Solution) where {k}
    flag = false
    copy_solution!(solver.bufferSol, sol)
    oldSol = solver.bufferSol
    neighborhoodId = neigh_index(InterShift{k})
    resize!(solver.buffer, length(sol.routes))
    copyto!(solver.buffer, 1:length(sol.routes))
    shuffle!(solver.seed, solver.buffer)
    routesIdx = solver.buffer

    for r1 in routesIdx
        bestMove = BestMove(dist = sol.dist, cost = sol.cost)
        route1 = sol.routes[r1]
        len1   = length(route1)
        len1 <= k + 1 && continue

        for r2 in routesIdx
            r2 == r1 && continue
            sol.lastEval[neighborhoodId, r1, r2] > max(sol.lastModif[r1], sol.lastModif[r2]) && continue
            route2 = sol.routes[r2]
            len2   = length(route2)

            for i = 2:(len1 - k)
                customers = @view route1[i:i+k-1]
                for j = 2:len2
                    dist     = interShiftCost(sol.dist, solver.data.costMatrix, route1, route2, i, j, k)
                    warpR1s1, warpR1s2 = computeStdViolRemoveK(solver, sol, r1, i, k)
                    warpR2s1, warpR2s2 = computeStdViolInsertionK(solver, sol, r2, customers, j)
                    violInfo = computeViolInterShiftK(solver, sol, r1, r2, i, j, k)
                    cost = objectiveValue(solver, sol,
                        Cost(dist, r1, r2, violInfo, (warpR1s1, warpR2s1), (warpR1s2, warpR2s2)))
                    if cost < bestMove.cost - 1e-6
                        bestMove = BestMove(cost, dist, r1, r2, i, j,
                            (violInfo.firstRouteInfeas, violInfo.secondRouteInfeas),
                            (warpR1s1, warpR1s2), (warpR2s1, warpR2s2))
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
                flag = true
                copy_solution!(oldSol, sol)
            else
                copy_solution!(sol, oldSol)
            end
        end
    end
    return flag
end

function apply!(::InterShift{k}, solver::Solver, sol::Solution, bestMove::BestMove) where {k}
    r1, r2 = bestMove.firstRoute, bestMove.secondRoute
    i,  j  = bestMove.firstIdx,   bestMove.secondIdx

    sol.dist = bestMove.dist
    sol.cost = bestMove.cost

    sol.totalInfeas -= sol.infeas[r1] + sol.infeas[r2]
    sol.totalInfeas += bestMove.infeas[1] + bestMove.infeas[2]
    sol.infeas[r1] = bestMove.infeas[1]
    sol.infeas[r2] = bestMove.infeas[2]

    block = sol.routes[r1][i:i+k-1]
    deleteat!(sol.routes[r1], i:i+k-1)
    for (offset, c) in enumerate(block)
        insert!(sol.routes[r2], j + offset - 1, c)
    end

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
    sol.lastModif[r1] = sol.timeStamp
    sol.lastModif[r2] = sol.timeStamp
end
