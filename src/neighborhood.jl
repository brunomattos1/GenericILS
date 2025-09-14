
function intraShift10!(solver::Solver, sol::Solution)
    flag = false
    for r = 1:length(sol.routes)
        bestMove = BestMove(cost = sol.cost, dist = sol.dist)
        if (sol.feasiblesF[r] >= length(sol.routes[r]) - 1) && (sol.warps[r] < 1e-6)
            for i = 2:length(sol.routes[r])-1
                for j = i+1:length(sol.routes[r])-1
                    dist, resViol = evalIntraShift10(solver, sol, Shift(r, r, i, j))
                    cost = sol.cost - sol.dist + dist

                    if resViol == 0 && (dist < bestMove.dist - 1e-5)
                        improvement = true
                    else
                        improvement = false
                    end
                    if improvement
                        bestMove = BestMove(cost, dist, r, 0, i, j, (0, 0), (0.0, 0.0))
                    end
                end
            end
        end
        if (sol.feasiblesB[r] >= length(sol.routes[r]) - 1) && (sol.warps[r] < 1e-6)
            for i = length(sol.routes[r])-1:-1:2
                for j = i-1:-1:2
                    if j == i - 1
                        solver.prevLabelB = myExtendAlongArc(solver.res, sol.backwardLabels[r][length(sol.routes[r]) - i], (sol.routes[r][i+1]+1, sol.routes[r][j] + 1))
                    end
                    dist, resViol = evalIntraShift10(sol.dist, sol, sol.routes, solver, r, i, j)
                    cost = sol.cost - sol.dist + dist
                    if resViol == 0 && (dist < bestMove.dist - 1e-5)
                        improvement = true
                    else
                        improvement = false
                    end
                    if improvement
                        bestMove = BestMove(cost, dist, r, 0, i, j, (0, 0), (0.0, 0.0))
                    end
                end
            end
        end
        if bestMove.firstIdx > 0
            # applyMoveIntraShift10!(solver, sol, bestDist, bestR, bestI, bestJ)
            applyMoveIntraShift10!(solver, sol, bestMove)

            computeLabels(solver, sol, [bestMove.firstRoute])
            flag = true
        end
    end
    return flag
end

function interShift10!(solver::Solver, sol::Solution)
    flag = false
    routesIdx = randperm(solver.seed, length(sol.routes))
    for r1 in routesIdx
        bestMove = BestMove(dist = sol.dist, cost = sol.cost)
        for r2 in routesIdx
            if r1 == r2
                continue
            end
            for i = 2:length(sol.routes[r1]) - 1
                for j = 2:length(sol.routes[r2])
                    # dist, cost, feasR1, feasR2, warpR1, warpR2 = evalInterShift10(sol.dist, sol, routes, solver, r1, r2, i, j)
                    dist, cost, infeasR1, infeasR2, warpR1, warpR2 = evalInterShift10(solver, sol, Shift(r1, r2, i, j))

                    # infeas = length(sol.routes[r1]) + length(sol.routes[r2]) - 4 - feas

                    # infeasR1 = length(sol.routes[r1]) - 1 - feasR1 - 1
                    # infeasR2 = length(sol.routes[r2]) - 1 - feasR2 + 1

                    # cost = objectiveValue(solver, sol, r1, r2, dist, infeasR1, infeasR2, warpR1, warpR2)
                    #=
                        # cost = dist + solver.params.penaltyInfeas * infeas + solver.params.penaltyStandard * warp
                        auxSol = deepcopy(sol)
                        customerI = auxSol.routes[r1][i]
                        deleteat!(auxSol.routes[r1], i)
                        insert!(auxSol.routes[r2], j, customerI)
                        cost_ = manualCost(auxSol, solver.data.costMatrix)
                        # if abs(dist - cost_) > 1e-6
                        #     throw()
                        # end
                        computedInfeasR1, computedInfeasR2, accDemand1F, accDemand2F, accDemand1B, accDemand2B = checkInfeasibles(solver, auxSol.routes[r1], auxSol.routes[r2])
                        currComputedInfeas, currAccDemand1F, currAccDemand2F, currAccDemand1B, currAccDemand2B = checkInfeasibles(solver, solver.outerCurrSol.routes[r1], solver.outerCurrSol.routes[r2])
                        # @show infeasR1, infeasR2, computedInfeasR1, computedInfeasR2
                        if infeasR1 < computedInfeasR1 || infeasR2 < computedInfeasR2
                            # if j != 2 
                                println("cust: $(solver.outerCurrSol.routes[r1][i]) pos: $j")
                                println("curr r1: $(solver.outerCurrSol.routes[r1]) curr r2: $(solver.outerCurrSol.routes[r2])")
                                println("acum D r1 F: $(currAccDemand1F), acum D r2 F: $(currAccDemand2F)")
                                println("acum D r1 B: $(currAccDemand1B), acum D r2 B: $(currAccDemand2B)")
                                println()
                                println("r1: $(auxSol.routes[r1]) r2: $(auxSol.routes[r2])")
                                println("acum D r1 F: $(accDemand1F), acum D r2 F: $(accDemand2F)")
                                println("acum D r1 B: $(accDemand1B), acum D r2 B: $(accDemand2B)")
                                println("Infeas R1: $(infeasR1), Computed Infeas R1: $(computedInfeasR1)")
                                println("Infeas R2: $(infeasR2), Computed Infeas R2: $(computedInfeasR2)")

                                sleep(1000)
                            # end
                        end
                    =#
                    # improvement = improved(cost, bestCost, infeas, bestInfeas)
                    if cost < bestMove.cost - 1e-6
                        improvement = true
                    else
                        improvement = false
                    end
                    if improvement
                        bestMove = BestMove(cost, dist, r1, r2, i, j, (infeasR1, infeasR2), (warpR1, warpR2))
                    end
                end
            end
        end
        if bestMove.firstIdx > 0
            flag = true
            applyMoveInterShift10!(solver, sol, bestMove)
            computeLabels(solver, sol, [bestMove.firstRoute, bestMove.secondRoute])
            sol.infeas[bestMove.firstRoute] = length(sol.routes[bestMove.firstRoute]) - max(sol.feasiblesF[bestMove.firstRoute], sol.feasiblesB[bestMove.firstRoute]) - 1
            sol.infeas[bestMove.secondRoute] = length(sol.routes[bestMove.secondRoute]) - max(sol.feasiblesF[bestMove.secondRoute], sol.feasiblesB[bestMove.secondRoute]) - 1
            sol.totalInfeas = sum(sol.infeas)
        end
    end
    return flag
end

function interSwap11!(solver::Solver, sol::Solution)
    flag = false
    routesIdx = randperm(solver.seed, length(sol.routes))
    for r1 in routesIdx
        bestMove = BestMove(dist = sol.dist, cost = sol.cost)
        for r2 in routesIdx
            if r1 == r2
                continue
            end
            for i = 2:length(sol.routes[r1]) - 1
                for j = 2:length(sol.routes[r2]) - 1
                    dist, cost, infeasR1, infeasR2, warpR1, warpR2 = evalInterSwap11(solver, sol, Swap(r1, r2, i, j))
                    # infeas = length(sol.routes[r1]) + length(sol.routes[r2]) - 4 - feas
                    # infeasR1 = length(sol.routes[r1]) - 1 - feasR1
                    # infeasR2 = length(sol.routes[r2]) - 1 - feasR2

                    # cost = objectiveValue(solver, sol, r1, r2, dist, infeasR1, infeasR2, warpR1, warpR2)
                    # cost = dist + solver.params.penaltyInfeas * infeas + solver.params.penaltyStandard * warp
                    #=
                        # auxSol = deepcopy(sol)
                        # customerI = auxSol.routes[r1][i]
                        # customerJ = auxSol.routes[r2][j]
                        # auxSol.routes[r1][i] = customerJ
                        # auxSol.routes[r2][j] = customerI
                        # computedInfeas, accDemand1F, accDemand2F, accDemand1B, accDemand2B = checkInfeasibles(solver, auxSol.routes[r1], auxSol.routes[r2])
                        # currComputedInfeas, currAccDemand1F, currAccDemand2F, currAccDemand1B, currAccDemand2B = checkInfeasibles(solver, solver.currSol.routes[r1], solver.currSol.routes[r2])
                        # auxCost = 0.
                        # for r = 1:length(auxSol.routes)
                        #     for i = 1:length(auxSol.routes[r]) - 1
                        #         auxCost += solver.data.costMatrix[auxSol.routes[r][i] + 1, auxSol.routes[r][i+1] + 1]
                        #     end
                        # end
                        # if abs(auxCost - cost) > 0.001
                        #     sleep(1000)
                        # end
                        # if infeas < computedInfeas && infeas > -50
                        #     # if j != 2 
                        #         println("cust: $(solver.currSol.routes[r1][i]) pos: $j")
                        #         println("curr r1: $(solver.currSol.routes[r1]) curr r2: $(solver.currSol.routes[r2])")
                        #         println("acum D r1 F: $(currAccDemand1F), acum D r2 F: $(currAccDemand2F)")
                        #         println("acum D r1 B: $(currAccDemand1B), acum D r2 B: $(currAccDemand2B)")
                        #         println()
                        #         println("r1: $(auxSol.routes[r1]) r2: $(auxSol.routes[r2])")
                        #         println("acum D r1 F: $(accDemand1F), acum D r2 F: $(accDemand2F)")
                        #         println("acum D r1 B: $(accDemand1B), acum D r2 B: $(accDemand2B)")
                        #         println("Infeas: $(infeas), Computed Infeas: $(computedInfeas)")
                        #         sleep(1000)
                        #     # end
                        # end
                    =#
                    # improvement = improved(cost, bestCost, infeas, bestInfeas)
                    improvement = false
                    if cost < bestMove.cost - 1e-6
                        improvement = true
                    end
                    if improvement
                        bestMove = BestMove(cost, dist, r1, r2, i, j, (infeasR1, infeasR2), (warpR1, warpR2))
                    end
                end
            end
        end
        if bestMove.firstIdx > 0
            # if bestCost < 34
            #     @show bestI, bestJ, bestR1, bestR2, bestInfeas, bestWarp
            #     @show sol
            #     throw()
            # end
            # applyMoveInterSwap11!(solver, sol, bestDist, bestCost, bestInfeas, bestWarp, bestR1, bestR2, bestI, bestJ)
            applyMoveInterSwap11!(solver, sol, bestMove)

            computeLabels(solver, sol, [bestMove.firstRoute, bestMove.secondRoute])
            sol.infeas[bestMove.firstRoute] = length(sol.routes[bestMove.firstRoute]) - max(sol.feasiblesF[bestMove.firstRoute], sol.feasiblesB[bestMove.firstRoute]) - 1
            sol.infeas[bestMove.secondRoute] = length(sol.routes[bestMove.secondRoute]) - max(sol.feasiblesF[bestMove.secondRoute], sol.feasiblesB[bestMove.secondRoute]) - 1
            sol.totalInfeas = sum(sol.infeas)
            flag = true

        end
    end
    return flag
end

function twoOptStar!(solver::Solver, sol::Solution)
    # routesIdx = shuffle!(solver.seed, Int[i for i = 1:length(solver.currSol.routes)])
    routesIdx = randperm(solver.seed, length(sol.routes))
    flag = false
    for r1 in routesIdx#1:length(sol.routes)
        bestI = 0
        bestJ = 0
        bestR1 = 0
        bestR2 = 0
        # sol = solver.currSol
        routes = sol.routes#getRoutes(sol)
        bestCost = sol.cost#getCost(sol)
        bestDist = sol.dist
        bestInfeas = (0, 0)
        bestWarp = (0.0, 0.0)
        bestMove = BestMove(cost = sol.cost, dist = sol.dist)
        # bestInfeas = length(sol.routes[r1]) - max(sol.feasiblesF[r1], sol.feasiblesB[r1]) - 2
        for r2 in routesIdx
            if r1 == r2
                continue
            end
            # bestInfeas += length(sol.routes[r2]) - max(sol.feasiblesF[r2], sol.feasiblesB[r2]) - 2
            for i = 1:length(sol.routes[r1]) - 2
                for j = 1:length(sol.routes[r2]) - 2

                    # dist, feasR1, feasR2, warpR1, warpR2 = evalTwoOptStar!(sol.dist, sol, sol.routes, solver, r1, r2, i, j)
                    dist, cost, infeasR1, infeasR2, warpR1, warpR2 = evalTwoOptStar!(solver, sol, OptStar(r1, r2, i, j))

                    # infeas = length(sol.routes[r1]) + length(sol.routes[r2]) - 4 - feas
                    # infeasR1 = length(sol.routes[r1]) - 2 - feasR1
                    # infeasR2 = length(sol.routes[r2]) - 2 - feasR2

                    # cost = objectiveValue(solver, sol, r1, r2, dist, infeasR1, infeasR2, warpR1, warpR2)
                    #=
                        # auxSol = deepcopy(sol)
                        # seg1 = copy(auxSol.routes[r1][i+1:end])
                        # seg2 = copy(auxSol.routes[r2][j+1:end])
                        # auxSol.routes[r1] = vcat(auxSol.routes[r1][1:i], seg2)
                        # auxSol.routes[r2] = vcat(auxSol.routes[r2][1:j], seg1)
                        # computedInfeas, accDemand1F, accDemand2F, accDemand1B, accDemand2B = checkInfeasibles(solver, auxSol.routes[r1], auxSol.routes[r2])
                        # currComputedInfeas, currAccDemand1F, currAccDemand2F, currAccDemand1B, currAccDemand2B = checkInfeasibles(solver, solver.currSol.routes[r1], solver.currSol.routes[r2])
                        # computedCost = manualCost(auxSol, solver.data.costMatrix)
                        # if abs(cost - computedCost) > 0.0001
                        #     throw("cost is $cost but should be $computedCost")
                        # end
                        # if infeas < computedInfeas && infeas > -50
                        #     # if j != 2 
                        #         # println("cust: $(solver.currSol.routes[r1][i]) pos: $j")
                        #         # println("curr r1: $(solver.currSol.routes[r1]) curr r2: $(solver.currSol.routes[r2])")
                        #         # println("acum D r1 F: $(currAccDemand1F), acum D r2 F: $(currAccDemand2F)")
                        #         # println("acum D r1 B: $(currAccDemand1B), acum D r2 B: $(currAccDemand2B)")
                        #         # println()
                        #         # println("r1: $(auxSol.routes[r1]) r2: $(auxSol.routes[r2])")
                        #         # println("acum D r1 F: $(accDemand1F), acum D r2 F: $(accDemand2F)")
                        #         # println("acum D r1 B: $(accDemand1B), acum D r2 B: $(accDemand2B)")
                        #         println("Infeas: $(infeas), Computed Infeas: $(computedInfeas)")
                        #         sleep(1000)
                        #     # end
                        # end
                    =#
                    # improvement = improved(cost, bestCost, infeas, bestInfeas)
                    improvement = false
                    if cost < bestMove.cost - 1e-6
                        improvement = true
                    end
                    if improvement
                        # bestDist = dist
                        # bestCost = cost
                        # bestR1 = r1
                        # bestR2 = r2
                        # bestI = i
                        # bestJ = j
                        # bestInfeas = (infeasR1, infeasR2)
                        # bestWarp = (warpR1, warpR2)
                        bestMove = BestMove(cost, dist, r1, r2, i, j, (infeasR1, infeasR2), (warpR1, warpR2))
                    end
                end
            end
        end
        if bestMove.firstIdx > 0
            # applyMoveTwoOptStar!(solver, sol, bestDist, bestCost, bestInfeas, bestWarp, bestR1, bestR2, bestI, bestJ)
            # computeLabels(solver, sol, [bestR1, bestR2])
            # flag = true
            applyMoveTwoOptStar!(solver, sol, bestMove)
            computeLabels(solver, sol, [bestMove.firstRoute, bestMove.secondRoute])
            sol.infeas[bestMove.firstRoute] = length(sol.routes[bestMove.firstRoute]) - max(sol.feasiblesF[bestMove.firstRoute], sol.feasiblesB[bestMove.firstRoute]) - 1
            sol.infeas[bestMove.secondRoute] = length(sol.routes[bestMove.secondRoute]) - max(sol.feasiblesF[bestMove.secondRoute], sol.feasiblesB[bestMove.secondRoute]) - 1
            sol.totalInfeas = sum(sol.infeas)
            flag = true
        end
    end
    return flag
end