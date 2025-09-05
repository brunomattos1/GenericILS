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

function evalBestInsertion(currCost::Float64, sol::Solution, routes::Vector{Vector{Int}}, solver::Solver, r::Int, customer::Int, j::Int)
    cost = bestInsertionCost(currCost, solver.data.costMatrix, routes[r], customer, j)
    dFeas = computeViolInsertion1(solver, sol, r, customer, j)
    return cost, dFeas
end

function evalIntraShift10(currCost::Float64,  sol::Solution, routes::Vector{Vector{Int}}, solver::Solver, r::Int, i::Int, j::Int)
    cost = intraShift10Cost(currCost, solver.data.costMatrix, routes[r], i, j)
    resViol = computeViolIntraShift10(solver, sol, r, i, j)
    return cost, resViol
end

function evalInterShift10(currCost::Float64, sol::Solution, routes::Vector{Vector{Int}}, solver::Solver, r1::Int, r2::Int, i::Int, j::Int)
    cost = interShift10Cost(currCost, solver.data.costMatrix, routes[r1], routes[r2], i, j)
    dFeas = computeViolInterShift10(solver, sol, r1, r2, i, j)
    return cost, dFeas
end

function evalInterSwap11(currCost::Float64, sol::Solution, routes::Vector{Vector{Int}}, solver::Solver, r1::Int, r2::Int, i::Int, j::Int)
    cost = interSwap11Cost(currCost, solver.data.costMatrix, routes[r1], routes[r2], i, j)
    dFeas = computeViolInterSwap11(solver, sol, r1, r2, i, j)
    return cost, dFeas
end



function evalTwoOptStar!(currCost::Float64, sol::Solution, routes::Vector{Vector{Int}}, solver::Solver, r1::Int, r2::Int, i::Int, j::Int)
    cost = twoOptStarCost(currCost, solver.data.costMatrix, routes[r1], routes[r2], i, j)
    dFeas = computeViolTwoOptStar(solver, sol, r1, r2, i, j)
    return cost, dFeas
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