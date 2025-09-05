
function intraShift10!(solver::Solver, sol::Solution)
    flag = false
    for r = 1:length(sol.routes)
        # sol = solution
        if (sol.feasiblesF[r] < length(sol.routes[r]) - 2) || (sol.feasiblesB[r] < length(sol.routes[r]) - 2)
            continue
        end
        bestI = 0
        bestJ = 0
        bestR = 0
        routes = sol.routes#getRoutes(sol)
        bestCost = sol.cost#getCost(sol)
        for i = 2:length(sol.routes[r])-1
            for j = i+1:length(sol.routes[r])-1
                # if i == j# || j == i+1 || i == j+1
                #     continue
                # end
                # auxRoute = copy(sol.routes[r])
                # customerI = auxRoute[i]
                # if i < j
                #     deleteat!(auxRoute, i)
                #     insert!(auxRoute, j-1, customerI)
                # else
                #     deleteat!(auxRoute, i)
                #     insert!(auxRoute, j, customerI)
                # end
                # @show r, i, j
                # auxSol = deepcopy(sol)
                # # @show auxSol.routes[r]
                # customerI = auxSol.routes[r][i]
                # if i < j
                #     if j == i + 1
                #         deleteat!(auxSol.routes[r], i)
                #         insert!(auxSol.routes[r], j, customerI)
                #     else
                #         deleteat!(auxSol.routes[r], i)
                #         insert!(auxSol.routes[r], j-1, customerI)
                #     end
                # else
                #     deleteat!(auxSol.routes[r], i)
                #     insert!(auxSol.routes[r], j, customerI)
                # end
                # # @show auxSol.routes[r]
                # cost_ =  manualCost(auxSol, solver.data.costMatrix)


                cost, resViol = evalIntraShift10(sol.cost, sol, routes, solver, r, i, j)
                # if abs(cost - cost_) > 0.001
                #     @show sol.routes[r]
                #     @show auxSol.routes[r]
                #     @show r, i, j
                #     @show cost, cost_
                #     @show resViol
                #     throw("")
                # end
                if resViol == 0 && (cost < bestCost - 1e-5)
                    improvement = true
                else
                    improvement = false
                end
                if improvement
                    bestI = i
                    bestJ = j
                    bestR = r
                    bestCost = cost
                end
            end
        end
        for i = length(sol.routes[r])-1:-1:2
            for j = i-1:-1:2
                if j == i - 1
                    solver.prevLabelB = extendAlongArc(solver.res, sol.backwardLabels[r][length(sol.routes[r]) - i], (sol.routes[r][i+1]+1, sol.routes[r][j] + 1))
                    # continue
                end
                # @show r, i, j
                # auxSol = deepcopy(sol)
                # # @show auxSol.routes[r]
                # customerI = auxSol.routes[r][i]
                # if i < j
                #     deleteat!(auxSol.routes[r], i)
                #     insert!(auxSol.routes[r], j-1, customerI)
                # else
                #     deleteat!(auxSol.routes[r], i)
                #     insert!(auxSol.routes[r], j, customerI)
                # end
                # cost_ = manualCost(auxSol, solver.data.costMatrix)
                # @show auxSol.routes[r]

                cost, resViol = evalIntraShift10(sol.cost, sol, routes, solver, r, i, j)
                # if abs(cost - cost_) > 1e-5
                #     throw()
                # end
                if resViol == 0 && (cost < bestCost - 1e-5)
                    improvement = true
                else
                    improvement = false
                end
                if improvement
                    bestI = i
                    bestJ = j
                    bestR = r
                    bestCost = cost
                end
            end
        end
        if bestI > 0
            applyMoveIntraShift10!(solver, sol, bestCost, bestR, bestI, bestJ)
            computeLabels(solver, sol, [bestR])
            flag = true
        end
    end
    return flag
end

function interShift10!(solver::Solver, sol::Solution)
    flag = false
    # @time routesIdx = shuffle(solver.seed, 1:length(solver.currSol.routes))
    routesIdx = randperm(solver.seed, length(sol.routes))
    for r1 in routesIdx#1:length(sol.routes)
        bestI = 0
        bestJ = 0
        bestR1 = 0
        bestR2 = 0
        # sol = solver.currSol
        routes = sol.routes#getRoutes(sol)
        bestCost = sol.cost#getCost(sol)
        bestInfeas = length(sol.routes[r1]) - max(sol.feasiblesF[r1], sol.feasiblesB[r1]) - 2
        for r2 in routesIdx
            if r1 == r2
                continue
            end
            bestInfeas += length(sol.routes[r2]) - max(sol.feasiblesF[r2], sol.feasiblesB[r2]) - 2
            for i = 2:length(sol.routes[r1]) - 1
                for j = 2:length(sol.routes[r2])
                    cost, feas = evalInterShift10(sol.cost, sol, routes, solver, r1, r2, i, j)
                    infeas = length(sol.routes[r1]) + length(sol.routes[r2]) - 4 - feas

                    # auxSol = deepcopy(sol)
                    # customerI = auxSol.routes[r1][i]
                    # deleteat!(auxSol.routes[r1], i)
                    # insert!(auxSol.routes[r2], j, customerI)
                    # cost_ = manualCost(auxSol, solver.data.costMatrix)
                    # if abs(cost - cost_) > 1e-6
                    #     throw()
                    # end
                    # computedInfeas, accDemand1F, accDemand2F, accDemand1B, accDemand2B = checkInfeasibles(solver, auxSol.routes[r1], auxSol.routes[r2])
                    # currComputedInfeas, currAccDemand1F, currAccDemand2F, currAccDemand1B, currAccDemand2B = checkInfeasibles(solver, solver.currSol.routes[r1], solver.currSol.routes[r2])

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
                    improvement = improved(cost, bestCost, infeas, bestInfeas)
                    if improvement
                        bestI = i
                        bestJ = j
                        bestR1 = r1
                        bestR2 = r2
                        bestCost = cost
                        bestInfeas = infeas
                    end
                end
            end
        end
        if bestI > 0
            flag = true
            applyMoveInterShift10!(solver, sol, bestCost, bestR1, bestR2, bestI, bestJ)
            computeLabels(solver, sol, [bestR1, bestR2])
        end
    end
    return flag
end

function interSwap11!(solver::Solver, sol::Solution)
    flag = false
    # routesIdx = shuffle!(solver.seed, Int[i for i = 1:length(solver.currSol.routes)])
    routesIdx = randperm(solver.seed, length(sol.routes))
    for r1 in routesIdx#1:length(sol.routes)
        bestI = 0
        bestJ = 0
        bestR1 = 0
        bestR2 = 0
        # sol = solver.currSol
        routes = sol.routes#getRoutes(sol)
        bestCost = sol.cost#getCost(sol)
        bestInfeas = length(sol.routes[r1]) - max(sol.feasiblesF[r1], sol.feasiblesB[r1]) - 2
        for r2 in routesIdx
            if r1 == r2
                continue
            end
            bestInfeas += length(sol.routes[r2]) - max(sol.feasiblesF[r2], sol.feasiblesB[r2]) - 2
            for i = 2:length(sol.routes[r1]) - 1
                for j = 2:length(sol.routes[r2]) - 1
                    cost, feas = evalInterSwap11(sol.cost, sol, routes, solver, r1, r2, i, j)
                    infeas = length(sol.routes[r1]) + length(sol.routes[r2]) - 4 - feas
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
                    improvement = improved(cost, bestCost, infeas, bestInfeas)
                    if improvement
                        bestI = i
                        bestJ = j
                        bestR1 = r1
                        bestR2 = r2
                        bestCost = cost
                        bestInfeas = infeas
                    end
                end
            end
        end
        if bestI > 0
            applyMoveInterSwap11!(solver, sol, bestCost, bestR1, bestR2, bestI, bestJ)
            computeLabels(solver, sol, [bestR1, bestR2])
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
        bestInfeas = length(sol.routes[r1]) - max(sol.feasiblesF[r1], sol.feasiblesB[r1]) - 2
        for r2 in routesIdx
            if r1 == r2
                continue
            end
            bestInfeas += length(sol.routes[r2]) - max(sol.feasiblesF[r2], sol.feasiblesB[r2]) - 2
            for i = 1:length(sol.routes[r1]) - 2
                for j = 1:length(sol.routes[r2]) - 2
                    # auxSol = deepcopy(sol)
                    # @show sol.cost
                    # @show i, j
                    # @show auxSol.routes[r1]
                    # @show auxSol.routes[r2]
                    # seg1 = copy(auxSol.routes[r1][i+1:end])
                    # seg2 = copy(auxSol.routes[r2][j+1:end])
                    # auxSol.routes[r1] = vcat(auxSol.routes[r1][1:i], seg2)
                    # auxSol.routes[r2] = vcat(auxSol.routes[r2][1:j], seg1)
                    # @show auxSol.routes[r1]
                    # @show auxSol.routes[r2]
                    cost, feas = evalTwoOptStar!(sol.cost, sol, sol.routes, solver, r1, r2, i, j)
                    infeas = length(sol.routes[r1]) + length(sol.routes[r2]) - 4 - feas
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
                    improvement = improved(cost, bestCost, infeas, bestInfeas)
                    if improvement
                        bestCost = cost
                        bestR1 = r1
                        bestR2 = r2
                        bestI = i
                        bestJ = j
                        bestInfeas = infeas
                    end
                end
            end
        end
        if bestI > 0
            applyMoveTwoOptStar!(solver, sol, bestCost, bestR1, bestR2, bestI, bestJ)
            computeLabels(solver, sol, [bestR1, bestR2])
            flag = true
        end
    end
    return flag
end