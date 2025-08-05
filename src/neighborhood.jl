#=
function intraShift10!(solver::Solver)
    bestI = 0
    bestJ = 0
    bestR = 0
    sol = solver.currSol
    routes = getRoutes(sol)
    bestCost = getCost(sol)
    for r = 1:length(solver.currSol.routes)
        for i = 2:length(sol.routes[r])-1
            for j = i+1:length(sol.routes[r])-1
                if i == j 
                    continue
                end
                if j == i+1
                    solver.prevLabelF = solver.extendAlongArc(solver.res, copy(solver.forwardLabels[r][i-1]), (solver.currSol.routes[r][i-1]+1, solver.currSol.routes[r][i+1] + 1))
                    continue
                end
                # @show r, i, j
                # auxSol = deepcopy(solver.currSol)
                # # @show auxSol.routes[r]
                # customerI = auxSol.routes[r][i]
                # if i < j
                #     deleteat!(auxSol.routes[r], i)
                #     insert!(auxSol.routes[r], j-1, customerI)
                # else
                #     deleteat!(auxSol.routes[r], i)
                #     insert!(auxSol.routes[r], j, customerI)
                # end
                # @show auxSol.routes[r]
                cost, resViol = evalIntraShift10(sol.cost, routes, solver, r, i, j)
                improvement = improved(solver, bestCost, cost, bestViol, r, r, resViol, resViol)
                # @show manualCost(auxSol, solver.data.costMatrix), cost, resViol
                # @show bestCost, cost, bestResViolR, resViol
                # println()
                if improvement
                    bestI = i
                    bestJ = j
                    bestR = r
                    bestCost = cost
                    bestResViolR = resViol
                    bestViol = sol.totalViolation - sol.resViolation[r] + resViol
                end
            end
        end
        for i = length(sol.routes[r])-1:-1:2
            for j = i-1:-1:2
                if i == j
                    continue
                end
                if j == i - 1
                    solver.prevLabelB = solver.extendAlongArc(solver.res, copy(solver.backwardLabels[r][length(solver.currSol.routes[r]) - i]), (solver.currSol.routes[r][i+1]+1, solver.currSol.routes[r][j] + 1))
                    continue
                end
                # @show r, i, j
                # auxSol = deepcopy(solver.currSol)
                # # @show auxSol.routes[r]
                # customerI = auxSol.routes[r][i]
                # if i < j
                #     deleteat!(auxSol.routes[r], i)
                #     insert!(auxSol.routes[r], j-1, customerI)
                # else
                #     deleteat!(auxSol.routes[r], i)
                #     insert!(auxSol.routes[r], j, customerI)
                # end
                # @show auxSol.routes[r]
                cost, resViol = evalIntraShift10(sol.cost, routes, solver, r, i, j)
                improvement = improved(solver, bestCost, cost, bestViol, r, r, resViol, resViol)
                # @show manualCost(auxSol, solver.data.costMatrix), cost, resViol
                # @show bestCost, cost, bestResViolR, resViol
                # println()
                # if abs(manualCost(auxSol, solver.data.costMatrix) - cost) > 0.001
                #     @show solver.currSol.routes[r]
                #     @show auxSol.routes[r]
                #     @show r, i, j
                #     @show manualCost(auxSol, solver.data.costMatrix), cost
                #     sleep(1000)
                # end

                improvement = improved(solver, bestCost, cost, bestViol, r, r, resViol, resViol)
                if improvement
                    bestI = i
                    bestJ = j
                    bestR = r
                    bestCost = cost
                    bestResViolR = resViol
                    bestViol = sol.totalViolation - sol.resViolation[r] + resViol
                end
            end
        end
    end
    if bestI > 0
        applyMoveIntraShift10!(solver, bestCost, bestViol, bestResViolR, bestR, bestI, bestJ)
        return true
    end
    return false
end
=#
function intraShift10!(solver::Solver)
    flag = false
    for r = 1:length(solver.currSol.routes)
        sol = solver.currSol
        if (sol.feasiblesF[r] < length(sol.routes[r]) - 2) || (sol.feasiblesB[r] < length(sol.routes[r]) - 2)
            continue
        end
        bestI = 0
        bestJ = 0
        bestR = 0
        routes = getRoutes(sol)
        bestCost = getCost(sol)
        for i = 2:length(sol.routes[r])-1
            for j = i+1:length(sol.routes[r])-1
                if i == j || j == i+1 || i == j+1
                    continue
                end
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
                # auxSol = deepcopy(solver.currSol)
                # # @show auxSol.routes[r]
                # customerI = auxSol.routes[r][i]
                # if i < j
                #     deleteat!(auxSol.routes[r], i)
                #     insert!(auxSol.routes[r], j-1, customerI)
                # else
                #     deleteat!(auxSol.routes[r], i)
                #     insert!(auxSol.routes[r], j, customerI)
                # end
                # @show auxSol.routes[r]
                cost, resViol = evalIntraShift10(sol.cost, routes, solver, r, i, j)
                if resViol == 0 && (cost < bestCost - 1e-5)
                    improvement = true
                else
                    improvement = false
                end
                # improvement = improved(solver, bestCost, cost, bestViol, r, r, resViol, resViol)
                # @show manualCost(auxSol, solver.data.costMatrix), cost, resViol
                # @show bestCost, cost, bestResViolR, resViol
                # println()
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
                if i == j
                    continue
                end
                if j == i - 1
                    solver.prevLabelB = solver.extendAlongArc(solver.res, copy(solver.backwardLabels[r][length(solver.currSol.routes[r]) - i]), (solver.currSol.routes[r][i+1]+1, solver.currSol.routes[r][j] + 1))
                    continue
                end
                # @show r, i, j
                # auxSol = deepcopy(solver.currSol)
                # # @show auxSol.routes[r]
                # customerI = auxSol.routes[r][i]
                # if i < j
                #     deleteat!(auxSol.routes[r], i)
                #     insert!(auxSol.routes[r], j-1, customerI)
                # else
                #     deleteat!(auxSol.routes[r], i)
                #     insert!(auxSol.routes[r], j, customerI)
                # end
                # @show auxSol.routes[r]
                cost, resViol = evalIntraShift10(sol.cost, routes, solver, r, i, j)
                if resViol == 0 && (cost < bestCost - 1e-5)
                    improvement = true
                else
                    improvement = false
                end
                # @show manualCost(auxSol, solver.data.costMatrix), cost, resViol
                # @show bestCost, cost, bestResViolR, resViol
                # println()
                # if abs(manualCost(auxSol, solver.data.costMatrix) - cost) > 0.001
                #     @show solver.currSol.routes[r]
                #     @show auxSol.routes[r]
                #     @show r, i, j
                #     @show manualCost(auxSol, solver.data.costMatrix), cost
                #     sleep(1000)
                # end

                # improvement = improved(solver, bestCost, cost, bestViol, r, r, resViol, resViol)
                if improvement
                    bestI = i
                    bestJ = j
                    bestR = r
                    bestCost = cost
                end
            end
        end
        if bestI > 0
            applyMoveIntraShift10!(solver, bestCost, bestR, bestI, bestJ)
            computeLabels(solver, [bestR])
            flag = true
        end
    end
    return flag
end

function intraShift20!(solver::Solver)
    bestI = 0
    bestJ = 0
    bestR = 0
    sol = solver.currSol
    routes = getRoutes(sol)
    bestCost = getCost(sol)
    bestResViolR = 0
    bestViol = sol.totalViolation
    for r = 1:length(sol.routes)
        for i = 2:length(sol.routes[r])-2
            for j = 2:length(sol.routes[r])
                if j <= i+2 && j >= i-2
                    continue
                end
                cost, resViol = evalIntraShift20(sol.cost, routes, solver, r, i, j)
                improvement = improved(bestCost, cost, bestResViolR, resViol)
                if improvement
                    bestI = i
                    bestJ = j
                    bestR = r
                    bestCost = cost
                    bestResViolR = resViol
                    bestViol = sol.totalViolation - sol.resViolation[r] + resViol
                end
            end
        end
    end
    if bestI > 0
        applyMoveIntraShift20!(solver, bestCost, bestViol, bestResViolR, bestR, bestI, bestJ)
        return true
    end
    return false
end

function intraSwap11!(solver::Solver)
    bestI = 0
    bestJ = 0
    bestR = 0
    sol = solver.currSol
    routes = getRoutes(sol)
    bestCost = getCost(sol)
    bestResViol = getResViolation(sol)
    for r = 1:length(sol.routes)
        for i = 2:length(sol.routes[r]) - 2
            for j = i+1:length(sol.routes[r]) - 1
                if i == j
                    continue
                end
                cost, resViol = evalIntraSwap11(sol.cost, sol.resViolation, routes, solver, r, i, j)
                improvement = improved(bestCost, cost, bestResViol, resViol)
                if improvement
                    bestI = i
                    bestJ = j
                    bestR = r
                    bestCost = cost
                    bestResViol = resViol
                end
            end
        end
    end
    if bestI > 0
        applyMoveIntraSwap11!(solver, bestCost, bestResViol, bestR, bestI, bestJ)
        return true
    end
    return false
end

function twoOpt!(solver::Solver)
    bestI = 0
    bestJ = 0
    bestR = 0
    sol = solver.currSol
    routes = getRoutes(sol)
    bestCost = getCost(sol)
    bestResViol = getResViolation(sol)
    for r = 1:length(sol.routes)
        for i = 2:length(sol.routes[r]) - 2
            for j = i+1:length(sol.routes[r]) - 1
                if i == j
                    continue
                end
                cost, resViol = eval2opt(sol.cost, sol.resViolation, routes, solver, r, i, j)
                improvement = improved(bestCost, cost, bestResViol, resViol)
                if improvement
                    bestI = i
                    bestJ = j
                    bestR = r
                    bestCost = cost
                    bestResViol = resViol
                end
            end
        end
    end
    if bestI > 0
        applyMove2opt!(solver, bestCost, bestResViol, bestR, bestI, bestJ)
        return true
    end
    return false
end

function interShift10!(solver::Solver)
    bestI = 0
    bestJ = 0
    bestR1 = 0
    bestR2 = 0
    sol = solver.currSol
    routes = getRoutes(sol)
    bestCost = getCost(sol)
    flag = false
    routesIdx = shuffle!(solver.seed, Int[i for i = 1:length(solver.currSol.routes)])
    for r1 in routesIdx#1:length(sol.routes)
        bestI = 0
        bestJ = 0
        bestR1 = 0
        bestR2 = 0
        sol = solver.currSol
        routes = getRoutes(sol)
        bestCost = getCost(sol)
        bestInfeas = length(sol.routes[r1]) - max(sol.feasiblesF[r1], sol.feasiblesB[r1]) - 2
        for r2 in routesIdx
            if r1 == r2
                continue
            end
            bestInfeas += length(sol.routes[r2]) - max(sol.feasiblesF[r2], sol.feasiblesB[r2]) - 2
            for i = 2:length(sol.routes[r1]) - 1
                for j = 2:length(sol.routes[r2])
                    cost, feas = evalInterShift10(sol.cost, routes, solver, r1, r2, i, j)
                    infeas = length(sol.routes[r1]) + length(sol.routes[r2]) - 4 - feas

                    # auxSol = deepcopy(solver.currSol)
                    # customerI = auxSol.routes[r1][i]
                    # deleteat!(auxSol.routes[r1], i)
                    # insert!(auxSol.routes[r2], j, customerI)
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
                    # println("curr r1: $(solver.currSol.routes[r1]) curr r2: $(solver.currSol.routes[r2])")
                    # println("r1: $(auxSol.routes[r1]) r2: $(auxSol.routes[r2])\nFeas: $feas Infeas: $(infeas)")
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
            # r1_orig = deepcopy(solver.currSol.routes[bestR1])
            # r2_orig = deepcopy(solver.currSol.routes[bestR2])

            applyMoveInterShift10!(solver, bestCost, bestR1, bestR2, bestI, bestJ)
            # computeLabels(solver)
            computeLabels(solver, [bestR1, bestR2])

            # infeas = length(solver.currSol.routes[bestR1]) -2 + length(solver.currSol.routes[bestR2]) - 2 - max(solver.currSol.feasiblesF[bestR1], solver.currSol.feasiblesB[bestR1]) - max(solver.currSol.feasiblesF[bestR2], solver.currSol.feasiblesB[bestR2])
            # if bestInfeas < infeas
            #     @show infeas
            #     @show bestInfeas
            #     r1 = solver.currSol.routes[bestR1]
            #     r2 = solver.currSol.routes[bestR2]
            #     @show r1_orig
            #     @show r2_orig
            #     @show bestI
            #     @show bestJ
            #     @show r1
            #     @show r2
            #     println("r1")
            #     for i in r1
            #         if i == 0
            #             continue
            #         end
            #         print("$(solver.res.d[1, i]) ")
            #     end
            #     println()
            #     println("r2")
            #     for i in r2
            #         if i == 0
            #             continue
            #         end
            #         print("$(solver.res.d[1, i]) ")
            #     end
            #     sleep(1000)
            # end

        end
    end
    return flag
end

function interShift20!(solver::Solver)
    bestI = 0
    bestJ = 0
    bestR1 = 0
    bestR2 = 0
    sol = solver.currSol
    routes = getRoutes(sol)
    bestCost = getCost(sol)
    # bestResViol = copy(sol.resViolation)
    bestResViolR1 = 0
    bestResViolR2 = 0
    bestViol = sol.totalViolation
    for r1 = 1:length(sol.routes)
        for r2 = 1:length(sol.routes)
            if r1 == r2
                continue
            end
            for i = 2:length(sol.routes[r1]) - 2
                for j = 2:length(sol.routes[r2])
                    cost, resViolR1, resViolR2 = evalInterShift20(sol.cost, routes, solver, r1, r2, i, j)
                    improvement = improved(cost, bestCost, dFeas, bestDFeas)
                    if improvement
                        bestI = i
                        bestJ = j
                        bestR1 = r1
                        bestR2 = r2
                        bestCost = cost
                        bestResViolR1 = resViolR1
                        bestResViolR2 = resViolR2
                        bestViol = sol.totalViolation - sol.resViolation[r1] - sol.resViolation[r2] + resViolR1 + resViolR2
                    end
                end
            end
        end
    end
    if bestI > 0
        applyMoveInterShift20!(solver, bestCost, bestViol, bestResViolR1, bestResViolR2, bestR1, bestR2, bestI, bestJ)
        return true
    end
    return false
end

function interSwap11!(solver::Solver)
    flag = false
    routesIdx = shuffle!(solver.seed, Int[i for i = 1:length(solver.currSol.routes)])
    for r1 in routesIdx#1:length(sol.routes)
        bestI = 0
        bestJ = 0
        bestR1 = 0
        bestR2 = 0
        sol = solver.currSol
        routes = getRoutes(sol)
        bestCost = getCost(sol)
        bestInfeas = length(sol.routes[r1]) - max(sol.feasiblesF[r1], sol.feasiblesB[r1]) - 2
        for r2 in routesIdx
            if r1 == r2
                continue
            end
            bestInfeas += length(sol.routes[r2]) - max(sol.feasiblesF[r2], sol.feasiblesB[r2]) - 2
            for i = 2:length(sol.routes[r1]) - 1
                for j = 2:length(sol.routes[r2]) - 1
                    cost, feas = evalInterSwap11(sol.cost, routes, solver, r1, r2, i, j)
                    infeas = length(sol.routes[r1]) + length(sol.routes[r2]) - 4 - feas
                    # auxSol = deepcopy(solver.currSol)
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
            applyMoveInterSwap11!(solver, bestCost, bestR1, bestR2, bestI, bestJ)
            # computeLabels(solver)
            computeLabels(solver, [bestR1, bestR2])
            flag = true
        end
    end
    return flag
end

function interSwap22!(solver::Solver)
    bestI = 0
    bestJ = 0
    bestR1 = 0
    bestR2 = 0
    bestCost = getCost(sol)
    bestResViol = getResViolation(sol)
    routes = getRoutes(sol)
    nbRoutes = length(routes)
    nbRoutes = length(routes)
    routeIndex = rand(solver.seed, 1:nbRoutes)
    for r1 in routeIndex
        nbCustomers = length(routes[r1])
        iIndex = rand(2:nbCustomers)
        for r2 in routeIndex
            if r1 == r2
                continue
            end
            nbCustomers = length(routes[r2])
            jIndex = rand(2:nbCustomers)
            for i in iIndex
                if routes[r1][i] == 0 || routes[r1][i+1] == 0
                    continue
                end
                for j in jIndex
                    if routes[r2][j] == 0 || routes[r2][j+1] == 0
                        continue
                    end
                    cost, resViol, improvement = evalInterSwap22Cost(sol.cost, sol.resViolation, routes, solver, r1, r2, i, j)
                    if improvement
                        bestI = i
                        bestJ = j
                        bestR1 = r1
                        bestR2 = r2
                        bestCost = cost
                        bestViol = viol
                    end
                end
            end
        end
    end
    if bestI > 0
        moveInterSwap22(solver, bestCost, bestResViol, routes, solver, bestR, bestI, bestJ)
        return true
    end
    return false
end

function twoOptStar!(solver::Solver)
    bestI = 0
    bestJ = 0
    bestR1 = 0
    bestR2 = 0
    sol = solver.currSol
    routes = getRoutes(sol)
    bestCost = getCost(sol)
    flag = false
    routesIdx = shuffle!(solver.seed, Int[i for i = 1:length(solver.currSol.routes)])
    for r1 in routesIdx#1:length(sol.routes)
        bestI = 0
        bestJ = 0
        bestR1 = 0
        bestR2 = 0
        sol = solver.currSol
        routes = getRoutes(sol)
        bestCost = getCost(sol)
        bestInfeas = length(sol.routes[r1]) - max(sol.feasiblesF[r1], sol.feasiblesB[r1]) - 2
        for r2 in routesIdx
            if r1 == r2
                continue
            end
            bestInfeas += length(sol.routes[r2]) - max(sol.feasiblesF[r2], sol.feasiblesB[r2]) - 2
            for i = 1:length(sol.routes[r1]) - 2
                for j = 1:length(sol.routes[r2]) - 2
                    og1 = deepcopy(sol.routes[r1])
                    og2 = deepcopy(sol.routes[r2])
                    seg1 = sol.routes[r1][i+1:end]
                    seg2 = sol.routes[r2][j+1:end]
                    route1 = vcat(sol.routes[r1][1:i], seg2)
                    route2 = vcat(sol.routes[r2][1:j], seg1)
                    cost = twoOptStarCost(sol.cost, solver.data.costMatrix, sol.routes[r1], sol.routes[r2], i, j)

                    @show i, j
                    @show route1
                    @show route2
                    computeViolTwoOptStar(solver, r1, r2, i, j)
                    println()
                    sol.routes[r1] = route1
                    sol.routes[r2] = route2
                    cost_ = manualCost(sol, solver.data.costMatrix)
                    if abs(cost - cost_) > 0.001
                        @show cost, cost_
                        println("i: $i, j: $j")
                        println("r1: $(og1)")
                        println("r2: $(og2)")
                        println("new r1: $route1")
                        println("new r2: $route2")
                        sleep(1000)
                    end
                    sol.routes[r1] = og1
                    sol.routes[r2] = og2
                end
            end
        end
    end
end