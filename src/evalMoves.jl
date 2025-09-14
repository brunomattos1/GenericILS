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

function objectiveValue(solver::Solver, sol::Solution, r::Int, dist::Float64, infeas::Int, warp::Float64)
    objVal = dist
    objVal += solver.params.penaltyCustom * (sol.totalInfeas - sol.infeas[r] + infeas)
    objVal += solver.params.penaltyStandard * (sol.totalWarp - sol.warps[r] + warp)
    return objVal
end

function objectiveValue(solver::Solver, sol::Solution, costing::Cost)
    objVal = costing.dist
    objVal += solver.params.penaltyCustom * (sol.totalInfeas - sol.infeas[costing.route1]  + costing.infeas[1] - sol.infeas[costing.route2] + costing.infeas[2])
    objVal += solver.params.penaltyStandard * (sol.totalWarp - sol.warps[costing.route1] + costing.warp[1] - sol.warps[costing.route2] + costing.warp[2])
    return objVal
end

function evalBestInsertion(solver::Solver, sol::Solution, insertion::Insertion)
    currCost = sol.dist
    routes = sol.routes
    r = insertion.route
    customer = insertion.customer
    j = insertion.pos
    dist = bestInsertionCost(currCost, solver.data.costMatrix, routes[r], customer, j)
    warp = computeStdViolInsertion1(solver, sol, r, customer, j)
    feas = computeViolInsertion1(solver, sol, r, customer, j)
    infeas = length(sol.routes[r]) - 1 - feas + 1
    cost = objectiveValue(solver, sol, r, dist, infeas, warp)
    return dist, cost, infeas, warp
end

function evalIntraShift10(currCost::Float64,  sol::Solution, routes::Vector{Vector{Int}}, solver::Solver, r::Int, i::Int, j::Int)
    dist = intraShift10Cost(currCost, solver.data.costMatrix, routes[r], i, j)
    resViol = computeViolIntraShift10(solver, sol, r, i, j)
    return dist, resViol
end

function evalIntraShift10(solver::Solver, sol::Solution, shift::Shift)
    currCost = sol.dist
    routes = sol.routes
    r = shift.routeFrom
    i = shift.fromIdx
    j = shift.toIdx
    dist = intraShift10Cost(currCost, solver.data.costMatrix, routes[r], i, j)
    resViol = computeViolIntraShift10(solver, sol, r, i, j)
    return dist, resViol
end

function evalInterShift10(solver::Solver, sol::Solution, shift::Shift)
    currCost = sol.dist
    routes = sol.routes
    r1 = shift.routeFrom
    r2 = shift.routeTo
    i = shift.fromIdx
    j = shift.toIdx
    
    dist = interShift10Cost(currCost, solver.data.costMatrix, routes[r1], routes[r2], i, j)
    warpR1 = computeStdViolRemove1(solver, sol, r1, i)
    warpR2 = computeStdViolInsertion1(solver, sol, r2, sol.routes[r1][i], j)
    feasR1, feasR2 = computeViolInterShift10(solver, sol, r1, r2, i, j)
    infeasR1 = length(sol.routes[r1]) - 1 - feasR1 - 1
    infeasR2 = length(sol.routes[r2]) - 1 - feasR2 + 1
    cost = objectiveValue(solver, sol, Cost(dist, r1, r2, (infeasR1, infeasR2), (warpR1, warpR2)))
    return dist, cost, infeasR1, infeasR2, warpR1, warpR2
end

function evalInterSwap11(solver::Solver, sol::Solution, swap::Swap)
    currCost = sol.dist
    routes = sol.routes
    r1 = swap.firstRoute
    r2 = swap.secondRoute
    i = swap.firstIdx
    j = swap.secondIdx
    dist = interSwap11Cost(currCost, solver.data.costMatrix, routes[r1], routes[r2], i, j)
    warpR1 = computeStdViolSwap11(solver, sol, r1, i, sol.routes[r2][j])
    warpR2 = computeStdViolSwap11(solver, sol, r2, j, sol.routes[r1][i])
    feasR1, feasR2 = computeViolInterSwap11(solver, sol, r1, r2, i, j)
    infeasR1 = length(sol.routes[r1]) - 1 - feasR1
    infeasR2 = length(sol.routes[r2]) - 1 - feasR2
    cost = objectiveValue(solver, sol, Cost(dist, r1, r2, (infeasR1, infeasR2), (warpR1, warpR2)))
    return dist, cost, feasR1, feasR2, warpR1, warpR2
end


function evalTwoOptStar!(currCost::Float64, sol::Solution, routes::Vector{Vector{Int}}, solver::Solver, r1::Int, r2::Int, i::Int, j::Int)
    dist = twoOptStarCost(currCost, solver.data.costMatrix, routes[r1], routes[r2], i, j)
    warpR1, warpR2 = computeStdViolTwoOptStar(solver, sol, r1, r2, i, j)
    feasR1, feasR2 = computeViolTwoOptStar(solver, sol, r1, r2, i, j)
    return dist, feasR1, feasR2, warpR1, warpR2
end

function evalTwoOptStar!(solver::Solver, sol::Solution, move::Move)
    currCost = sol.dist
    routes = sol.routes
    r1, r2 = move.firstRoute, move.secondRoute
    i, j = move.firstIdx, move.secondIdx
    dist = twoOptStarCost(currCost, solver.data.costMatrix, routes[r1], routes[r2], i, j)
    warpR1, warpR2 = computeStdViolTwoOptStar(solver, sol, r1, r2, i, j)
    feasR1, feasR2 = computeViolTwoOptStar(solver, sol, r1, r2, i, j)
    infeasR1 = i + length(sol.routes[r2]) - j - feasR1 - 1
    infeasR2 = j + length(sol.routes[r1]) - i - feasR2 - 1
    cost = objectiveValue(solver, sol, Cost(dist, r1, r2, (infeasR1, infeasR2), (warpR1, warpR2)))
    return dist, cost, infeasR1, infeasR2, warpR1, warpR2
end

function evalSplit!(currCost::Float64, routes::Vector{Vector{Int}}, solver::Solver, r::Int, i::Int)
    cost = splitCost(currCost, solver.data.costMatrix, routes[r], i)
    return cost
end

function evalIntraShift20(currCost::Float64, routes::Vector{Vector{Int}}, solver::Solver, r::Int, i::Int, j::Int)
    cost = intraShift20Cost(currCost, solver.data.costMatrix, routes[r], i, j)
    resViol = computeViolIntraShift20(solver, r)
    return cost, resViol
end

# function evalIntraSwap11(currCost::Float64, routes::Vector{Vector{Int}}, solver::Solver, r::Int, i::Int, j::Int)
#     cost = intraSwap11Cost(currCost, solver.data.costMatrix, routes[r], i, j)
#     # resViol = computeViolIntraSwap11()
#     return cost
# end

# function eval2opt(currCost::Float64, routes::Vector{Vector{Int}}, solver::Solver, r::Int, i::Int, j::Int)
#     cost = twoOptCost(currCost, solver.data.costMatrix, routes[r], i, j)
#     # resViol = computeViol2opt()
#     return cost
# end

# function evalInterShift20(currCost::Float64, routes::Vector{Vector{Int}}, solver::Solver, r1::Int, r2::Int, i::Int, j::Int)
#     cost = interShift20Cost(currCost, solver.data.costMatrix, routes[r1], routes[r2], i, j)
#     dFeas = computeViolInterShift20(solver, r1, r2, i, j)
#     return cost, dFeas
# end

# function evalInterSwap22!(currCost::Float64, currResViol::Int, routes::Vector{Vector{Int}}, solver::Solver, r1::Int, r2::Int, i::Int, j::Int)
#     cost = interSwap22Cost(currCost, solver.costMatrix, routes[r1], routes[r2], i, j)
#     resViol = computeViolInterSwap22()
#     return cost, resViol, improved(currCost, cost, currResViol, resViol)
# end