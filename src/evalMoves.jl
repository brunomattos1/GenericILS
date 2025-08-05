# function improved(bestCost::Float64, cost::Float64, bestResViolR1::Int, bestResViolR2::Int, r1::Int, r2::Int, resViolR1::Int, resViolR2::Int)
#     if resViolR1 < bestResViolR1 && resViolR2 == bestResViolR2
#         return true
#     end
#     if resViolR2 < bestResViolR2 && resViolR1 == bestResViolR1
#         return true
#     end
#     if (resViolR1 == bestResViolR1 && resViolR2 == bestResViolR2) && cost < bestCost - 0.001
#         return true
#     end

#     return false
# end

# function improved(bestCost::Float64, cost::Float64, bestResViol::Int, resViol::Int)
#     if resViol < bestResViol
#         return true
#     end
#     if resViol == bestResViol && cost < bestCost - 0.001
#         return true
#     end
#     return false
# end

# function improved(solver::Solver, bestCost::Float64, cost::Float64, bestViol::Int, r1::Int, r2::Int, resViolR1::Int, resViolR2::Int)
#     viol = solver.currSol.totalViolation - solver.currSol.resViolation[r1] - solver.currSol.resViolation[r2] + resViolR1 + resViolR2
#     if viol < bestViol
#         return true
#     end
#     if (viol == bestViol) && cost < bestCost - 0.01
#         return true
#     end
#     return false
# end

# function improved(cost::Float64, bestCost::Float64, feas::Int, bestFeas::Int)
#     # comparar quantidade de inviaveis (length(route) - feas)
#     if feas > bestFeas
#         return true
#     end
#     if feas == bestFeas && cost < bestCost - 1e-5
#         return true
#     end
#     return false
# end

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

# function evalBestInsertion(currCost::Float64, currResViol::Int, bestCost::Float64, bestResViol::Int, routes::Vector{Vector{Int}}, solver::Solver, r::Int, customer::Int, j::Int)
#     cost = bestInsertionCost(currCost, solver.data.costMatrix, routes[r], customer, j)
#     resViol = computeViolInsertion(solver, r, customer, j)
#     return cost, resViol
# end

function evalBestInsertion(currCost::Float64, routes::Vector{Vector{Int}}, solver::Solver, r::Int, customer::Int, j::Int)
    cost = bestInsertionCost(currCost, solver.data.costMatrix, routes[r], customer, j)
    dFeas = computeViolInsertion1(solver, r, customer, j)
    return cost, dFeas
end

function evalIntraShift10(currCost::Float64, routes::Vector{Vector{Int}}, solver::Solver, r::Int, i::Int, j::Int)
    cost = intraShift10Cost(currCost, solver.data.costMatrix, routes[r], i, j)
    resViol = computeViolIntraShift10(solver, r, i, j)
    return cost, resViol
end

function evalIntraShift20(currCost::Float64, routes::Vector{Vector{Int}}, solver::Solver, r::Int, i::Int, j::Int)
    cost = intraShift20Cost(currCost, solver.data.costMatrix, routes[r], i, j)
    resViol = computeViolIntraShift20(solver, r)
    return cost, resViol
end

function evalIntraSwap11(currCost::Float64, currResViol::Int, routes::Vector{Vector{Int}}, solver::Solver, r::Int, i::Int, j::Int)
    cost = intraSwap11Cost(currCost, solver.data.costMatrix, routes[r], i, j)
    resViol = computeViolIntraSwap11()
    return cost, resViol
end

function eval2opt(currCost::Float64, currResViol::Int, routes::Vector{Vector{Int}}, solver::Solver, r::Int, i::Int, j::Int)
    cost = twoOptCost(currCost, solver.data.costMatrix, routes[r], i, j)
    resViol = computeViol2opt()
    return cost, resViol
end

function evalInterShift10(currCost::Float64, routes::Vector{Vector{Int}}, solver::Solver, r1::Int, r2::Int, i::Int, j::Int)
    cost = interShift10Cost(currCost, solver.data.costMatrix, routes[r1], routes[r2], i, j)
    dFeas = computeViolInterShift10(solver, r1, r2, i, j)
    return cost, dFeas
end

function evalInterShift20(currCost::Float64, routes::Vector{Vector{Int}}, solver::Solver, r1::Int, r2::Int, i::Int, j::Int)
    cost = interShift20Cost(currCost, solver.data.costMatrix, routes[r1], routes[r2], i, j)
    dFeas = computeViolInterShift20(solver, r1, r2, i, j)
    return cost, dFeas
end

function evalInterSwap11(currCost::Float64, routes::Vector{Vector{Int}}, solver::Solver, r1::Int, r2::Int, i::Int, j::Int)
    cost = interSwap11Cost(currCost, solver.data.costMatrix, routes[r1], routes[r2], i, j)
    dFeas = computeViolInterSwap11(solver, r1, r2, i, j)
    return cost, dFeas
end

function evalInterSwap22!(currCost::Float64, currResViol::Int, routes::Vector{Vector{Int}}, solver::Solver, r1::Int, r2::Int, i::Int, j::Int)
    cost = interSwap22Cost(currCost, solver.costMatrix, routes[r1], routes[r2], i, j)
    resViol = computeViolInterSwap22()
    return cost, resViol, improved(currCost, cost, currResViol, resViol)
end

function evalSplit!(currCost::Float64, routes::Vector{Vector{Int}}, solver::Solver, r::Int, i::Int)
    cost = splitCost(currCost, solver.data.costMatrix, routes[r], i)
    return cost
end

function evalTwoOptStar!(currCost::Float64, routes::Vector{Vector{Int}}, solver::Solver, r1::Int, r2::Int, i::Int, j::Int)
    cost = twoOptStarCost(currCost, solver.data.costMatrix, routes[r1], routes[r2], i, j)
    dFeas = computeViolTwoOptStar(solver, r1, r2, i, j)
    return cost, dFeas
end

