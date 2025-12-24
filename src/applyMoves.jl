
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
    computeLabels(solver, solution, r)

    solution.totalInfeas -= solution.infeas[r]
    solution.infeas[r] = length(solution.routes[r]) - max(solution.feasiblesF[r], solution.feasiblesB[r]) - 1
    solution.totalInfeas += solution.infeas[r]

    solution.totalWarp -= solution.warps[r]
    solution.warps[r] = solution.forwardLabels[r][end].std_res.stdWarp
    solution.totalWarp += solution.warps[r]
    solution.cost = objectiveValue(solver, solution)

end

function applyMoveIntraShift10!(solver::Solver, solution::Solution, move::BestMove)
    r = move.firstRoute
    i, j = move.firstIdx, move.secondIdx

    # atualizar custo
    solution.cost = move.cost
    solution.dist = move.dist
    solution.totalInfeas -= solution.infeas[r]
    solution.totalInfeas += move.infeas[1]
    solution.infeas[r] = move.infeas[1]
    # atualizar warp
    solution.totalWarp -= solution.warps[r]
    solution.totalWarp += move.warps[1]
    solution.warps[r] = move.warps[1]

    # pegar cliente a mover
    customerI = solution.routes[r][i]

    if i < j
        if j == i + 1
            deleteat!(solution.routes[r], i)
            insert!(solution.routes[r], j, customerI)
        else
            deleteat!(solution.routes[r], i)
            insert!(solution.routes[r], j-1, customerI)
        end
    else
        deleteat!(solution.routes[r], i)
        insert!(solution.routes[r], j, customerI)
    end
    computeLabels(solver, solution, r)
    solution.totalInfeas -= solution.infeas[r]
    solution.infeas[r] = length(solution.routes[r]) - max(solution.feasiblesF[r], solution.feasiblesB[r]) - 1
    solution.totalInfeas += solution.infeas[r]

    solution.totalWarp -= solution.warps[r]
    solution.warps[r] = solution.forwardLabels[r][end].std_res.stdWarp
    solution.totalWarp += solution.warps[r]

    solution.cost = objectiveValue(solver, solution)
    solver.timeStamp += 1
    solution.lastModif[r] = solver.timeStamp
    # solution.lastEval[(:intraShift, r, r)] = solver.timeStamp
    solution.lastEval[1, r, r] = solver.timeStamp

end

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
    computeLabels(solver, solution, r1, r2)

    solution.totalInfeas -= solution.infeas[r1] + solution.infeas[r2]
    solution.infeas[r1] = length(solution.routes[r1]) - max(solution.feasiblesF[r1], solution.feasiblesB[r1]) - 1
    solution.infeas[r2] = length(solution.routes[r2]) - max(solution.feasiblesF[r2], solution.feasiblesB[r2]) - 1
    solution.totalInfeas += solution.infeas[r1] + solution.infeas[r2]

    solution.totalWarp -= solution.warps[r1] + solution.warps[r2]
    solution.warps[r1] = solution.forwardLabels[r1][end].std_res.stdWarp
    solution.warps[r2] = solution.forwardLabels[r2][end].std_res.stdWarp
    solution.totalWarp += solution.warps[r1] + solution.warps[r2]
    # println("-"^100)
    solution.cost = objectiveValue(solver, solution)

    solver.timeStamp += 1
    solution.lastModif[r1] = solver.timeStamp
    solution.lastModif[r2] = solver.timeStamp
    # solution.lastEval[(:interShift, r1, r2)] = solver.timeStamp
    solution.lastEval[2, r1, r2] = solver.timeStamp

end

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
    # @show solution
    # @show r1, r2, i, j
    solution.routes[r1][i] = customerJ
    solution.routes[r2][j] = customerI
    computeLabels(solver, solution, r1, r2)
    # @show solution.infeas[r1]
    # @show solution.infeas[r2]
    solution.totalInfeas -= solution.infeas[r1] + solution.infeas[r2]
    solution.infeas[r1] = length(solution.routes[r1]) - max(solution.feasiblesF[r1], solution.feasiblesB[r1]) - 1
    solution.infeas[r2] = length(solution.routes[r2]) - max(solution.feasiblesF[r2], solution.feasiblesB[r2]) - 1
    solution.totalInfeas += solution.infeas[r1] + solution.infeas[r2]
    # @show solution.infeas[r1]
    # @show solution.infeas[r2]

    solution.totalWarp -= solution.warps[r1] + solution.warps[r2]
    solution.warps[r1] = solution.forwardLabels[r1][end].std_res.stdWarp
    solution.warps[r2] = solution.forwardLabels[r2][end].std_res.stdWarp
    solution.totalWarp += solution.warps[r1] + solution.warps[r2]

    # println("-"^100)
    solution.cost = objectiveValue(solver, solution)
    solver.timeStamp += 1
    solution.lastModif[r1] = solver.timeStamp
    solution.lastModif[r2] = solver.timeStamp
    # solution.lastEval[(:interSwap, r1, r2)] = solver.timeStamp
    solution.lastEval[3, r1, r2] = solver.timeStamp

end

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
    seg1_len = length(solution.routes[r1]) - i
    seg2_len = length(solution.routes[r2]) - j

    # salva cauda de r1 no buffer
    buffer = solver.buffer2opt
    resize!(buffer, seg1_len)
    copyto!(buffer, 1, solution.routes[r1], i+1, seg1_len)

    # move cauda de r2 para r1
    resize!(solution.routes[r1], i + seg2_len)
    copyto!(solution.routes[r1], i+1, solution.routes[r2], j+1, seg2_len)

    # move cauda salva (seg1) para r2
    resize!(solution.routes[r2], j + seg1_len)
    copyto!(solution.routes[r2], j+1, buffer, 1, seg1_len)
    computeLabels(solver, solution, r1, r2)
    solution.totalInfeas -= solution.infeas[r1] + solution.infeas[r2]
    solution.infeas[r1] = length(solution.routes[r1]) - max(solution.feasiblesF[r1], solution.feasiblesB[r1]) - 1
    solution.infeas[r2] = length(solution.routes[r2]) - max(solution.feasiblesF[r2], solution.feasiblesB[r2]) - 1
    solution.totalInfeas += solution.infeas[r1] + solution.infeas[r2]

    solution.totalWarp -= solution.warps[r1] + solution.warps[r2]
    solution.warps[r1] = solution.forwardLabels[r1][end].std_res.stdWarp
    solution.warps[r2] = solution.forwardLabels[r2][end].std_res.stdWarp
    solution.totalWarp += solution.warps[r1] + solution.warps[r2]

    solution.cost = objectiveValue(solver, solution)
    solver.timeStamp += 1
    solution.lastModif[r1] = solver.timeStamp
    solution.lastModif[r2] = solver.timeStamp
    # solution.lastEval[(:twoOptStar, r1, r2)] = solver.timeStamp
    solution.lastEval[4, r1, r2] = solver.timeStamp

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