# function applyMoveInsertion(solver::Solver, solution::Solution, newDist::Float64, newCost::Float64, 
#     newInfeas::Int, newWarp::Float64, r::Int, customer::Int, j::Int)
#     solution.dist = newDist
#     solution.cost = newCost
#     solution.totalInfeas = solution.totalInfeas - solution.infeas[r] + newInfeas
#     solution.infeas[r] = newInfeas
#     solution.totalWarp = solution.totalWarp -  solution.warps[r] + newWarp
#     solution.warps[r] = newWarp
#     insert!(solution.routes[r], j, customer)
# end

function applyMoveInsertion(solver::Solver, solution::Solution, move::BestInsertion)
    r = move.route
    i = move.pos
    c = move.customer

    # atualizar distância e custo
    solution.dist = move.dist
    solution.cost = move.cost

    # atualizar infeasibility
    solution.totalInfeas -= solution.infeas[r]
    solution.totalInfeas += move.infeas
    solution.infeas[r] = move.infeas

    # atualizar warp
    solution.totalWarp -= solution.warps[r]
    solution.totalWarp += move.warp
    solution.warps[r] = move.warp

    # inserir cliente
    insert!(solution.routes[r], i, c)
end

# function applyMoveIntraShift10!(solver::Solver, sol::Solution, newCost::Float64, r::Int, i::Int, j::Int)
#     sol.cost = newCost
#     customerI = sol.routes[r][i]
#     if i < j
#         if j == i + 1
#             deleteat!(sol.routes[r], i)
#             insert!(sol.routes[r], j, customerI)
#         else
#             deleteat!(sol.routes[r], i)
#             insert!(sol.routes[r], j-1, customerI)
#         end
#     else
#         deleteat!(sol.routes[r], i)
#         insert!(sol.routes[r], j, customerI)
#     end
# end

function applyMoveIntraShift10!(solver::Solver, sol::Solution, move::BestMove)
    r = move.firstRoute
    i, j = move.firstIdx, move.secondIdx

    # atualizar custo
    sol.cost = move.cost
    sol.dist = move.dist

    # pegar cliente a mover
    customerI = sol.routes[r][i]

    if i < j
        if j == i + 1
            deleteat!(sol.routes[r], i)
            insert!(sol.routes[r], j, customerI)
        else
            deleteat!(sol.routes[r], i)
            insert!(sol.routes[r], j-1, customerI)
        end
    else
        deleteat!(sol.routes[r], i)
        insert!(sol.routes[r], j, customerI)
    end
end

# function applyMoveInterShift10!(solver::Solver, solution::Solution, newDist::Float64, newCost::Float64, 
#         newInfeas::Tuple{Int, Int}, newWarp::Tuple{Float64, Float64}, r1::Int, r2::Int, i::Int, j::Int)
#     solution.dist = newDist
#     solution.cost = newCost
#     solution.totalInfeas = solution.totalInfeas - solution.infeas[r1] - solution.infeas[r2] + newInfeas[1] + newInfeas[2]
#     solution.infeas[r1] = newInfeas[1]
#     solution.infeas[r2] = newInfeas[2]

#     solution.totalWarp = solution.totalWarp - solution.warps[r1] - solution.warps[r2] + newWarp[1] + newWarp[2]
#     solution.warps[r1] = newWarp[1]
#     solution.warps[r2] = newWarp[2]

#     customerI = solution.routes[r1][i]
#     deleteat!(solution.routes[r1], i)
#     insert!(solution.routes[r2], j, customerI)
# end

function applyMoveInterShift10!(solver::Solver, solution::Solution, move::BestMove)
    r1, r2 = move.firstRoute, move.secondRoute
    i, j = move.firstIdx, move.secondIdx

    solution.dist = move.dist
    solution.cost = move.cost

    solution.totalInfeas -= solution.infeas[r1] + solution.infeas[r2]
    solution.totalInfeas += move.infeas[1] + move.infeas[2]
    solution.infeas[r1] = move.infeas[1]
    solution.infeas[r2] = move.infeas[2]

    solution.totalWarp -= solution.warps[r1] + solution.warps[r2]
    solution.totalWarp += move.warps[1] + move.warps[2]
    solution.warps[r1] = move.warps[1]
    solution.warps[r2] = move.warps[2]

    customerI = solution.routes[r1][i]
    deleteat!(solution.routes[r1], i)
    insert!(solution.routes[r2], j, customerI)
end

# function applyMoveInterSwap11!(solver::Solver, solution::Solution, newDist::Float64, newCost::Float64, 
#         newInfeas::Tuple{Int, Int}, newWarp::Tuple{Float64, Float64}, r1::Int, r2::Int, i::Int, j::Int)
#     solution.dist = newDist
#     solution.cost = newCost
#     solution.totalInfeas = solution.totalInfeas - solution.infeas[r1] - solution.infeas[r2] + newInfeas[1] + newInfeas[2]
#     solution.infeas[r1] = newInfeas[1]
#     solution.infeas[r2] = newInfeas[2]

#     solution.totalWarp = solution.totalWarp - solution.warps[r1] - solution.warps[r2] + newWarp[1] + newWarp[2]
#     solution.warps[r1] = newWarp[1]
#     solution.warps[r2] = newWarp[2]

#     customerI = solution.routes[r1][i]
#     customerJ = solution.routes[r2][j]
#     solution.routes[r1][i] = customerJ
#     solution.routes[r2][j] = customerI
# end

function applyMoveInterSwap11!(solver::Solver, solution::Solution, move::BestMove)
    r1, r2 = move.firstRoute, move.secondRoute
    i, j   = move.firstIdx, move.secondIdx

    # atualizar custo/distância
    solution.dist = move.dist
    solution.cost = move.cost

    # atualizar infeasibility
    solution.totalInfeas -= solution.infeas[r1] + solution.infeas[r2]
    solution.totalInfeas += move.infeas[1] + move.infeas[2]
    solution.infeas[r1] = move.infeas[1]
    solution.infeas[r2] = move.infeas[2]

    # atualizar warp
    solution.totalWarp -= solution.warps[r1] + solution.warps[r2]
    solution.totalWarp += move.warps[1] + move.warps[2]
    solution.warps[r1] = move.warps[1]
    solution.warps[r2] = move.warps[2]

    # troca os clientes
    customerI = solution.routes[r1][i]
    customerJ = solution.routes[r2][j]
    solution.routes[r1][i] = customerJ
    solution.routes[r2][j] = customerI
end

# function applyMoveTwoOptStar!(solver::Solver, solution::Solution, newDist::Float64, newCost::Float64, 
#         newInfeas::Tuple{Int, Int}, newWarp::Tuple{Float64, Float64}, r1::Int, r2::Int, i::Int, j::Int)
#     solution.dist = newDist
#     solution.cost = newCost
#     solution.totalInfeas = solution.totalInfeas - solution.infeas[r1] - solution.infeas[r2] + newInfeas[1] + newInfeas[2]
#     solution.infeas[r1] = newInfeas[1]
#     solution.infeas[r2] = newInfeas[2]

#     solution.totalWarp = solution.totalWarp - solution.warps[r1] - solution.warps[r2] + newWarp[1] + newWarp[2]
#     solution.warps[r1] = newWarp[1]
#     solution.warps[r2] = newWarp[2]

#     seg1 = solution.routes[r1][i+1:end]
#     seg2 = solution.routes[r2][j+1:end]
#     solution.routes[r1] = vcat(solution.routes[r1][1:i], seg2)
#     solution.routes[r2] = vcat(solution.routes[r2][1:j], seg1)
# end

function applyMoveTwoOptStar!(solver::Solver, solution::Solution, move::BestMove)
    r1, r2 = move.firstRoute, move.secondRoute
    i, j   = move.firstIdx, move.secondIdx

    # atualizar custo e distância
    solution.dist = move.dist
    solution.cost = move.cost

    # atualizar infeasibility
    solution.totalInfeas -= solution.infeas[r1] + solution.infeas[r2]
    solution.totalInfeas += move.infeas[1] + move.infeas[2]
    solution.infeas[r1] = move.infeas[1]
    solution.infeas[r2] = move.infeas[2]

    # atualizar warp
    solution.totalWarp -= solution.warps[r1] + solution.warps[r2]
    solution.totalWarp += move.warps[1] + move.warps[2]
    solution.warps[r1] = move.warps[1]
    solution.warps[r2] = move.warps[2]

    # realizar troca dos segmentos
    seg1 = solution.routes[r1][i+1:end]
    seg2 = solution.routes[r2][j+1:end]
    solution.routes[r1] = vcat(solution.routes[r1][1:i], seg2)
    solution.routes[r2] = vcat(solution.routes[r2][1:j], seg1)
end

function applyMoveSplit!(solver::Solver, newCost::Float64, r::Int, i::Int)
    solver.currSol.cost = newCost
    split1 = copy(solver.currSol.routes[r][1:i-1])
    push!(split1, 0)
    split2 = copy(solver.currSol.routes[r][i:end])
    pushfirst!(split2, 0)
    solver.currSol.routes[r] = split1
    push!(solver.currSol.routes, split2)
end

# not implemented (yet?)
# function applyMoveInterSwap22!(solver::Solver, newCost::Float64, newResViol::Int, r1::Int, r2::Int, i::Int, j::Int)
#     solver.currSol.cost = newCost
#     solver.currSol.resViolation = newResViol
#     customerI1 = solver.currSol.routes[r1][i]
#     customerI2 = solver.currSol.routes[r1][i]
#     customerJ1 = solver.currSol.routes[r2][j]
#     customerJ2 = solver.currSol.routes[r2][j]
#     solver.currSol.routes[r1][i] = customerJ1
#     solver.currSol.routes[r2][i+1] = customerJ2
#     solver.currSol.routes[r1][j] = customerI1
#     solver.currSol.routes[r2][j+1] = customerI2
# end

# function applyMoveIntraShift20!(solver::Solver, newCost::Float64, r::Int, i::Int, j::Int)
#     solver.currSol.cost = newCost
#     customerI1 = solver.currSol.routes[r][i]
#     customerI2 = solver.currSol.routes[r][i+1]
#     if i < j
#         deleteat!(solver.currSol.routes[r], [i, i+1])
#         insert!(solver.currSol.routes[r], j-2, customerI2)
#         insert!(solver.currSol.routes[r], j-2, customerI1)
#     else
#         deleteat!(solver.currSol.routes[r], [i, i+1])
#         insert!(solver.currSol.routes[r], j, customerI2)
#         insert!(solver.currSol.routes[r], j, customerI1)
#     end
# end

# function applyMoveIntraSwap11!(solver::Solver, newCost::Float64, r::Int, i::Int, j::Int)
#     solver.currSol.cost = newCost
#     solver.currSol.routes[r][i], solver.currSol.routes[r][j] = solver.currSol.routes[r][j], solver.currSol.routes[r][i]
# end

# function applyMove2opt!(solver::Solver, newCost::Float64, r::Int, i::Int, j::Int)
#     solver.currSol.cost = newCost
#     reverse!(solver.currSol.routes[r], i, j)
# end

# function applyMoveInterShift20!(solver::Solver, newCost::Float64, newTotalViol::Int, newResViolR1::Int, newResViolR2::Int, r1::Int, r2::Int, i::Int, j::Int)
#     solver.currSol.cost = newCost
#     solver.currSol.totalViolation = newTotalViol
#     solver.currSol.resViolation[r1] = newResViolR1
#     solver.currSol.resViolation[r2] = newResViolR2

#     customerI1 = solver.currSol.routes[r1][i]
#     customerI2 = solver.currSol.routes[r1][i+1]
#     deleteat!(solver.currSol.routes[r1], [i, i+1])
#     insert!(solver.currSol.routes[r2], j, customerI2)
#     insert!(solver.currSol.routes[r2], j, customerI1)
# end

