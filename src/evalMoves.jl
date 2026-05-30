function improved(cost::Float64, bestCost::Float64, infeas::Int, bestInfeas::Int)
    # comparar quantidade de inviaveis (length(route) - feas)
    if infeas < -100
        return false
    end
    if infeas < bestInfeas
        return true
    end
    if infeas == bestInfeas && cost < bestCost - 1e-5
        return true
    end
    return false
end

function objectiveValue(solver::Solver, sol::Solution)
    objVal = sol.dist
    if isCostResource()
        objVal += sol.totalLabelCost
    end
    objVal += solver.parameters.penaltyCustom * (sol.totalInfeas)
    objVal += solver.parameters.penaltyStandard1 * (sol.totalWarpStd1)
    objVal += solver.parameters.penaltyStandard2 * (sol.totalWarpStd2)

    return objVal
end

function objectiveValue(solver::Solver, sol::Solution, r::Int, dist::Float64, infeas::Int, labelCost::Float64, warpStd1::Float64, warpStd2::Float64)
    objVal = dist
    if isCostResource()
        objVal += sol.totalLabelCost - sol.labelCosts[r] + labelCost
    end
    objVal += solver.parameters.penaltyCustom * (sol.totalInfeas - sol.infeas[r] + infeas)
    objVal += solver.parameters.penaltyStandard1 * (sol.totalWarpStd1 - sol.warpsStd1[r] + warpStd1)
    objVal += solver.parameters.penaltyStandard2 * (sol.totalWarpStd2 - sol.warpsStd2[r] + warpStd2)

    return objVal
end

function objectiveValue(solver::Solver, sol::Solution, costing::Cost)
    objVal = costing.dist
    if isCostResource()
        objVal += sol.totalLabelCost - sol.labelCosts[costing.route1] + costing.violInfo.firstRouteLabelCost - sol.labelCosts[costing.route2] + costing.violInfo.secondRouteLabelCost
    end
    objVal += solver.parameters.penaltyCustom * (sol.totalInfeas - sol.infeas[costing.route1]  + costing.violInfo.firstRouteInfeas - sol.infeas[costing.route2] + costing.violInfo.secondRouteInfeas)
    objVal += solver.parameters.penaltyStandard1 * (sol.totalWarpStd1 - sol.warpsStd1[costing.route1] + costing.warpStd1[1] - sol.warpsStd1[costing.route2] + costing.warpStd1[2])
    objVal += solver.parameters.penaltyStandard1 * (sol.totalWarpStd2 - sol.warpsStd2[costing.route1] + costing.warpStd2[1] - sol.warpsStd2[costing.route2] + costing.warpStd2[2])
    return objVal
end

function evalBestInsertion(solver::Solver, sol::Solution, insertion::Insertion)
    currCost = sol.dist
    routes = sol.routes
    r = insertion.route
    customer = insertion.customer
    j = insertion.pos
    dist = bestInsertionCost(currCost, solver.data.costMatrix, routes[r], customer, j)
    warpStd1, warpStd2 = computeStdViolInsertion1(solver, sol, r, customer, j)
    # feas, labelCost = computeViolInsertion1(solver, sol, r, customer, j)
    # infeas = length(sol.routes[r]) - 1 - feas + 1
    infeas, labelCost = infeasArcsInsertion(solver, sol, r, customer, j)
    cost = objectiveValue(solver, sol, r, dist, infeas, labelCost, warpStd1, warpStd2)
    return dist, cost, infeas, warpStd1, warpStd2
end

function evalIntraShift10(solver::Solver, sol::Solution, shift::Shift)
    currCost = sol.dist
    routes = sol.routes
    r = shift.routeFrom
    i = shift.fromIdx
    j = shift.toIdx

    dist = intraShift10Cost(currCost, solver.data.costMatrix, routes[r], i, j)
    resViol, labelCost = computeViolIntraShift10(solver, sol, r, i, j)
    warpStd1, warpStd2 = computeStdViolIntraShift10(solver, sol, r, i, j)
    cost = objectiveValue(solver, sol, r, dist, 0, labelCost, warpStd1, warpStd2)
    return dist, cost, resViol, warpStd1, 0.0, warpStd2, 0.0
end

function evalInterShift10(solver::Solver, sol::Solution, shift::Shift)
    currCost = sol.dist
    routes = sol.routes
    r1 = shift.routeFrom
    r2 = shift.routeTo
    i = shift.fromIdx
    j = shift.toIdx
    
    dist = interShift10Cost(currCost, solver.data.costMatrix, routes[r1], routes[r2], i, j)
    warpR1Std1, warpR1Std2 = computeStdViolRemove1(solver, sol, r1, i)
    warpR2Std1, warpR2Std2 = computeStdViolInsertion1(solver, sol, r2, sol.routes[r1][i], j)
    violInfo = computeViolInterShift10(solver, sol, r1, r2, i, j)
    # infeasR1 = length(sol.routes[r1]) - 1 - feasR1 - 1
    # infeasR2 = length(sol.routes[r2]) - 1 - feasR2 + 1
    cost = objectiveValue(solver, sol, Cost(dist, r1, r2, violInfo, (warpR1Std1, warpR2Std1), (warpR1Std2, warpR2Std2)))
    return dist, cost, violInfo.firstRouteInfeas, violInfo.secondRouteInfeas, warpR1Std1, warpR1Std2, warpR2Std1, warpR2Std2
end

function evalInterSwap11(solver::Solver, sol::Solution, swap::Swap)
    currCost = sol.dist
    routes = sol.routes
    r1 = swap.firstRoute
    r2 = swap.secondRoute
    i = swap.firstIdx
    j = swap.secondIdx
    dist = interSwap11Cost(currCost, solver.data.costMatrix, routes[r1], routes[r2], i, j)
    warpR1Std1, warpR1Std2 = computeStdViolSwap11(solver, sol, r1, i, sol.routes[r2][j])
    warpR2Std1, warpR2Std2 = computeStdViolSwap11(solver, sol, r2, j, sol.routes[r1][i])
    violInfo = computeViolInterSwap11(solver, sol, r1, r2, i, j)
    cost = objectiveValue(solver, sol, Cost(dist, r1, r2, violInfo, (warpR1Std1, warpR2Std1), (warpR1Std2, warpR2Std2)))
    return dist, cost, violInfo.firstRouteInfeas, violInfo.secondRouteInfeas, warpR1Std1, warpR1Std2, warpR2Std1, warpR2Std2
end

function evalInterShift20(solver::Solver, sol::Solution, shift::Shift)
    currCost = sol.dist
    r1 = shift.routeFrom
    r2 = shift.routeTo
    i  = shift.fromIdx
    j  = shift.toIdx

    dist = interShift20Cost(currCost, solver.data.costMatrix, sol.routes[r1], sol.routes[r2], i, j)
    warpR1Std1, warpR1Std2 = computeStdViolRemove2(solver, sol, r1, i)
    warpR2Std1, warpR2Std2 = computeStdViolInsertion2(solver, sol, r2, sol.routes[r1][i], sol.routes[r1][i+1], j)
    violInfo = computeViolInterShift20(solver, sol, r1, r2, i, j)
    cost = objectiveValue(solver, sol,
        Cost(dist, r1, r2, violInfo, (warpR1Std1, warpR2Std1), (warpR1Std2, warpR2Std2)))
    return dist, cost, violInfo.firstRouteInfeas, violInfo.secondRouteInfeas,
           warpR1Std1, warpR1Std2, warpR2Std1, warpR2Std2
end

function evalTwoOptStar!(solver::Solver, sol::Solution, move::OptStar)
    currCost = sol.dist
    routes = sol.routes
    r1, r2 = move.firstRoute, move.secondRoute
    i, j = move.firstIdx, move.secondIdx
    dist = twoOptStarCost(currCost, solver.data.costMatrix, routes[r1], routes[r2], i, j)
    warpR1Std1, warpR1Std2, warpR2Std1, warpR2Std2 = computeStdViolTwoOptStar(solver, sol, r1, r2, i, j)
    violInfo = computeViolTwoOptStar(solver, sol, r1, r2, i, j)
    cost = objectiveValue(solver, sol, Cost(dist, r1, r2, violInfo, (warpR1Std1, warpR2Std1), (warpR1Std2, warpR2Std2)))
    return dist, cost, violInfo.firstRouteInfeas, violInfo.secondRouteInfeas, warpR1Std1, warpR1Std2, warpR2Std1, warpR2Std2
end