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
    buf1 = solver.bufferRoute
    buf2 = solver.buffer2opt
    resize!(buf1, k1)
    resize!(buf2, k2)
    copyto!(buf1, 1, sol.routes[r1].visits, i, k1)
    copyto!(buf2, 1, sol.routes[r2].visits, j, k2)
    infeasR1, lc1, warpR1s1, warpR1s2 = infeasArcsReplaceBlockK(solver, sol, r1, i, k1, buf2)
    infeasR2, lc2, warpR2s1, warpR2s2 = infeasArcsReplaceBlockK(solver, sol, r2, j, k2, buf1)
    violInfo = ViolationInfo(infeasR1, infeasR2, lc1, lc2)
    return violInfo, warpR1s1, warpR2s1, warpR1s2, warpR2s2
end

function search!(neigh::InterSwap{k1,k2}, solver::Solver, sol::Solution) where {k1,k2}
    flag = false
    copy_solution!(solver.bufferSol, sol)
    oldSol = solver.bufferSol
    neighborhoodId = neigh_index(typeof(solver), InterSwap{k1,k2})
    resize!(solver.buffer, length(sol.routes))
    copyto!(solver.buffer, 1:length(sol.routes))
    shuffle!(solver.seed, solver.buffer)
    routesIdx = solver.buffer

    for r1 in routesIdx
        bestMove = BestMove(dist = sol.dist, cost = sol.cost)
        route1 = sol.routes[r1].visits
        len1   = length(route1)
        len1 <= k1 + 1 && continue
        for r2 in routesIdx
            r2 == r1 && continue
            k1 == k2 && r1 >= r2 && continue
            route2 = sol.routes[r2].visits
            len2   = length(route2)
            len2 <= k2 + 1 && continue
            sol.lastEval[neighborhoodId, r1, r2] > max(sol.routes[r1].lastModif, sol.routes[r2].lastModif) && continue
            canPrune = canPruneByDist(solver)
            fixedPenalty = canPrune ? pruningFixedPenalty(solver, sol, r1, r2) : 0.0

            for i = 2:(len1 - k1)
                for j = 2:(len2 - k2)
                    dist = interSwapCost(sol.dist, solver.data.costMatrix, route1, route2, i, j, k1, k2)
                    if canPrune && dist + fixedPenalty >= bestMove.cost - 1e-6
                        continue
                    end
                    violInfo, warpR1s1, warpR2s1, warpR1s2, warpR2s2 =
                        computeViolInterSwapK(solver, sol, r1, r2, i, j, k1, k2)
                    cost = objectiveValue(solver, sol,
                        Cost(dist, r1, r2, violInfo, (warpR1s1, warpR2s1), (warpR1s2, warpR2s2)))
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
    rt1 = sol.routes[r1]
    rt2 = sol.routes[r2]

    sol.dist             = bestMove.dist
    sol.totalInfeas     -= rt1.infeas    + rt2.infeas
    sol.totalWarpStd1   -= rt1.warpStd1 + rt2.warpStd1
    sol.totalWarpStd2   -= rt1.warpStd2 + rt2.warpStd2
    sol.totalLabelCost  -= rt1.labelCost + rt2.labelCost

    move_blocks!(rt1.visits, i, k1, rt2.visits, j, k2, solver.bufferRoute)

    computeLabels(solver, sol, r1, r2)

    rt1.infeas = length(rt1.visits) - max(rt1.feasibleF, rt1.feasibleB) - 1
    rt2.infeas = length(rt2.visits) - max(rt2.feasibleF, rt2.feasibleB) - 1
    sol.totalInfeas += rt1.infeas + rt2.infeas

    rt1.warpStd1 = rt1.forwardLabels[end].std1State.stdWarp
    rt2.warpStd1 = rt2.forwardLabels[end].std1State.stdWarp
    sol.totalWarpStd1 += rt1.warpStd1 + rt2.warpStd1

    rt1.warpStd2 = rt1.forwardLabels[end].std2State.stdWarp
    rt2.warpStd2 = rt2.forwardLabels[end].std2State.stdWarp
    sol.totalWarpStd2 += rt1.warpStd2 + rt2.warpStd2

    rt1.labelCost = min(rt1.forwardLabels[end].cost, rt1.backwardLabels[end].cost)
    rt2.labelCost = min(rt2.forwardLabels[end].cost, rt2.backwardLabels[end].cost)
    sol.totalLabelCost += rt1.labelCost + rt2.labelCost

    sol.cost = objectiveValue(solver, sol)
    rt1.lastModif = sol.timeStamp
    rt2.lastModif = sol.timeStamp
end
