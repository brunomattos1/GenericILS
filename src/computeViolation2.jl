function computeViolInsertion1(solver::Solver, r::Int, customer::Int, pos::Int)
    # FORWARD
    if solver.currSol.feasiblesF[r] == length(solver.currSol.routes[r]) - 2
        prev = solver.extendAlongArc(solver.res, copy(solver.forwardLabels[r][pos-1]), (solver.currSol.routes[r][pos-1]+1, customer + 1))
        concatCost = solver.concatenationCost(solver.res, customer, prev, solver.backwardLabels[r][length(solver.currSol.routes[r]) + 1 - pos])
        if prev.cost == Inf
            dFeasForward = - length(solver.currSol.routes[r]) - 2
        elseif prev.cost < Inf && concatCost.cost < Inf
            dFeasForward = 2
        elseif prev.cost < Inf && concatCost.cost == Inf
            dFeasForward = - length(solver.currSol.routes[r]) - 1
        end
    else
        # tentar um concat solver.forwardLabels[r][pos-1] + customer + backward
        # só faz o resto se der inviavel
        prev = solver.extendAlongArc(solver.res, copy(solver.forwardLabels[r][pos-1]), (solver.currSol.routes[r][pos-1]+1, customer + 1))
        prev2 = solver.extendAlongArc(solver.res, copy(prev), (customer + 1, solver.currSol.routes[r][pos]+1))
        if pos < solver.currSol.lastFeasibleF[r] + 1
            dFeasForward = -1
            prev.cost < Inf ? dFeasForward += 1 : 0
            prev2.cost < Inf ? dFeasForward += 1 : 0   
        end
        if pos == solver.currSol.lastFeasibleF[r] + 1
            dFeasForward = 0
            prev.cost < Inf ? dFeasForward += 1 : 0
            prev2.cost < Inf ? dFeasForward += 1 : 0
        end
        # TO DO
        if pos > solver.currSol.lastFeasibleF[r] + 1
            dFeasForward = -typemax(Int)
        end
    end
    # BACKWARD
    if solver.currSol.feasiblesB[r] == length(solver.currSol.routes[r]) - 2
        # Corrigir as extensoes
        prev = solver.extendAlongArc(solver.res, copy(solver.backwardLabels[r][pos-1]), (solver.currSol.routes[r][pos-1]+1, customer + 1))
        concatCost = solver.concatenationCost(solver.res, customer, prev, solver.backwardLabels[r][length(solver.currSol.routes[r]) + 1 - pos])
        if prev.cost == Inf
            dFeasBackward = - length(solver.currSol.routes[r]) - 2
        elseif prev.cost < Inf && concatCost.cost < Inf
            dFeasBackward = 2
        elseif prev.cost < Inf && concatCost.cost == Inf
            dFeasBackward = - length(solver.currSol.routes[r]) - 1
        end
    else
        prev = solver.extendAlongArc(solver.res, copy(solver.backwardLabels[r][length(solver.currSol.routes[r]) - pos + 1]), (solver.currSol.routes[r][pos]+1, customer+1))
        prev2 = solver.extendAlongArc(solver.res, copy(prev), (customer + 1, solver.currSol.routes[r][pos-1]+1))
        if pos > length(solver.currSol.routes[r]) - solver.currSol.lastFeasibleB[r] + 1
            dFeasBackward = -1
            prev.cost < Inf ? dFeasBackward += 1 : 0
            prev2.cost < Inf ? dFeasBackward += 1 : 0   
        end
        if pos == length(solver.currSol.routes[r]) - solver.currSol.lastFeasibleB[r] + 1
            dFeasBackward = 0
            prev.cost < Inf ? dFeasBackward += 1 : 0
            prev2.cost < Inf ? dFeasBackward += 1 : 0
        end
        if pos < length(solver.currSol.routes[r]) - solver.currSol.lastFeasibleB[r] + 1
            dFeasBackward = -typemax(Int)
        end
    end
    return max(solver.currSol.feasiblesF[r] + dFeasForward, solver.currSol.feasiblesB[r] + dFeasBackward)
end

function computeViolRemove1(solver::Solver, r::Int, pos::Int)
    # FORWARD
    if solver.currSol.feasiblesF[r] == length(solver.currSol.routes[r]) - 2
        removLabel = concatenationCost(solver.res, pos, solver.forwardLabels[r][pos-1], solver.backwardLabels[r][length(solver.currSol.routes[r]) + 1 - pos - 1])
        if removLabel.cost < Inf
            dFeasForward = -2
        else
            dFeasForward = -length(solver.currSol.routes[r])-1
        end
        # println("FEASIBLE ROUTE: $(removLabel)")
    else
        if pos < solver.currSol.lastFeasibleF[r] + 1
            dFeasForward = - 2
            # tentar concat antes igual ao Insertion1
            removLabel = solver.extendAlongArc(solver.res, copy(solver.forwardLabels[r][pos-1]), (solver.currSol.routes[r][pos-1]+1, solver.currSol.routes[r][pos+1]+1))
            if removLabel.cost < Inf
                dFeasForward += 1
            else
                dFeasForward += 0
            end
            # println("i < : $(removLabel)")

        elseif pos == solver.currSol.lastFeasibleF[r] + 1
            removLabel = solver.extendAlongArc(solver.res, copy(solver.forwardLabels[r][pos-1]), (solver.currSol.routes[r][pos-1]+1, solver.currSol.routes[r][pos+1]+1))
            if removLabel.cost < Inf
                dFeasForward = 1
            else
                dFeasForward = 0
            end
            # println("i == : $(removLabel)")

        elseif pos > solver.currSol.lastFeasibleF[r] + 1
            # println("i > ")
            dFeasForward = -typemax(Int)
        end
    end
    
    # BACKWARD
    if solver.currSol.feasiblesB[r] == length(solver.currSol.routes[r]) - 2
        removLabel = concatenationCost(solver.res, pos, solver.forwardLabels[r][pos-1], solver.backwardLabels[r][length(solver.currSol.routes[r]) + 1 - pos - 1])
        if removLabel.cost < Inf
            dFeasBackward = -2
        else
            dFeasBackward = -length(solver.currSol.routes[r])-1
        end
        # println("FEASIBLE ROUTE: $(removLabel)")
    else
        removLabel = solver.extendAlongArc(solver.res, copy(solver.backwardLabels[r][pos-1]), (solver.currSol.routes[r][pos+1]+1, solver.currSol.routes[r][pos-1]+1))
        if pos > length(solver.currSol.routes[r]) - solver.currSol.lastFeasibleB[r] + 1
            dFeasBackward = - 2
            if removLabel.cost < Inf
                dFeasBackward += 1
            else
                dFeasBackward += 0
            end
            # println("i < : ")

        elseif pos == length(solver.currSol.routes[r]) - solver.currSol.lastFeasibleB[r] + 1
            if removLabel.cost < Inf
                dFeasBackward = 1
            else
                dFeasBackward = 0
            end
            # println("i == : ")

        elseif pos < length(solver.currSol.routes[r]) - solver.currSol.lastFeasibleB[r] + 1
            # println("i > ")
            dFeasBackward = -typemax(Int)
        end
    end
    return max(solver.currSol.feasiblesF[r] + dFeasForward, solver.currSol.feasiblesB[r] + dFeasBackward)
end

function computeViolInsertion2(solver::Solver, r::Int, i::Int, j::Int)
    # FORWARD
    if solver.currSol.feasiblesF[r] == length(solver.currSol.routes[r]) - 2
        prev = solver.extendAlongArc(solver.res, copy(solver.forwardLabels[r][j-1]), (solver.currSol.routes[r][j-1]+1, i + 1))
        concatCost = solver.concatenationCost(solver.res, i, prev, solver.backwardLabels[r][length(solver.currSol.routes[r]) + 1 - j])
        if prev.cost == Inf
            dFeasForward = - length(solver.currSol.routes[r]) - 2
        elseif prev.cost < Inf && concatCost.cost < Inf
            dFeasForward = 2
        elseif prev.cost < Inf && concatCost.cost == Inf
            dFeasForward = - length(solver.currSol.routes[r]) - 1
        end
    else
        prev = solver.extendAlongArc(solver.res, copy(solver.forwardLabels[r][j-1]), (solver.currSol.routes[r][j-1]+1, i + 1))
        prev2 = solver.extendAlongArc(solver.res, copy(prev), (i + 1, solver.currSol.routes[r][j]+1))
        if j < solver.currSol.lastFeasibleF[r] + 1
            dFeasForward = -1
            prev.cost < Inf ? dFeasForward += 1 : 0
            prev2.cost < Inf ? dFeasForward += 1 : 0   
        end
        if j == solver.currSol.lastFeasibleF[r] + 1
            dFeasForward = 0
            prev.cost < Inf ? dFeasForward += 1 : 0
            prev2.cost < Inf ? dFeasForward += 1 : 0
        end
        if j > solver.currSol.lastFeasibleF[r] + 1
            dFeasForward = -typemax(Int)
        end
    end

    # BACKWARD
    if solver.currSol.feasiblesB[r] == length(solver.currSol.routes[r]) - 2
        prev = solver.extendAlongArc(solver.res, copy(solver.forwardLabels[r][j-1]), (solver.currSol.routes[r][j-1]+1, i + 1))
        concatCost = solver.concatenationCost(solver.res, i, prev, solver.backwardLabels[r][length(solver.currSol.routes[r]) + 1 - j])
        if prev.cost == Inf
            dFeasBackward = - length(solver.currSol.routes[r]) - 2
        elseif prev.cost < Inf && concatCost.cost < Inf
            dFeasBackward = 2
        elseif prev.cost < Inf && concatCost.cost == Inf
            dFeasBackward = - length(solver.currSol.routes[r]) - 1
        end
    else
        prev = solver.extendAlongArc(solver.res, copy(solver.backwardLabels[r][length(solver.currSol.routes[r]) - j + 1]), (solver.currSol.routes[r][j]+1, i+1))
        prev2 = solver.extendAlongArc(solver.res, copy(prev), (i + 1, solver.currSol.routes[r][j-1]+1))
        if j > length(solver.currSol.routes[r]) - solver.currSol.lastFeasibleB[r] + 1
            dFeasBackward = -1
            prev.cost < Inf ? dFeasBackward += 1 : 0
            prev2.cost < Inf ? dFeasBackward += 1 : 0   
        end
        if j == length(solver.currSol.routes[r]) - solver.currSol.lastFeasibleB[r] + 1
            dFeasBackward = 0
            prev.cost < Inf ? dFeasBackward += 1 : 0
            prev2.cost < Inf ? dFeasBackward += 1 : 0
        end
        if j < length(solver.currSol.routes[r]) - solver.currSol.lastFeasibleB[r] + 1
            dFeasBackward = -typemax(Int)
        end
    end
    return max(solver.currSol.feasiblesF[r] + dFeasForward, solver.currSol.feasiblesB[r] + dFeasBackward)
end

#=
    # TO DO
    # function computeViolRemove2(solver::Solver, r::Int, i::Int)
    #     # FORWARD
    #     if solver.currSol.feasiblesF[r] == length(solver.currSol.routes[r]) - 2
    #         removLabel = concatenationCost(solver.res, i, solver.forwardLabels[r][i-1], solver.backwardLabels[r][length(solver.currSol.routes[r]) + 1 - i - 1])
    #         if removLabel.cost < Inf
    #             dFeasForward = -2
    #         else
    #             dFeasForward = -length(solver.currSol.routes[r])-1
    #         end
    #         # println("FEASIBLE ROUTE: $(removLabel)")
    #     else
    #         if i < solver.currSol.lastFeasibleF[r] + 1
    #             dFeasForward = - 2
    #             removLabel = solver.extendAlongArc(solver.res, copy(solver.forwardLabels[r][i-1]), (solver.currSol.routes[r][i-1]+1, solver.currSol.routes[r][i+1]+1))
    #             if removLabel.cost < Inf
    #                 dFeasForward += 1
    #             else
    #                 dFeasForward += 0
    #             end
    #             # println("i < : $(removLabel)")

    #         elseif i == solver.currSol.lastFeasibleF[r] + 1
    #             removLabel = solver.extendAlongArc(solver.res, copy(solver.forwardLabels[r][i-1]), (solver.currSol.routes[r][i-1]+1, solver.currSol.routes[r][i+1]+1))
    #             if removLabel.cost < Inf
    #                 dFeasForward = 1
    #             else
    #                 dFeasForward = 0
    #             end
    #             # println("i == : $(removLabel)")

    #         elseif i > solver.currSol.lastFeasibleF[r] + 1
    #             # println("i > ")
    #             dFeasForward = -typemax(Int)
    #         end
    #     end
        
    #     # BACKWARD
    #     if solver.currSol.feasiblesB[r] == length(solver.currSol.routes[r]) - 2
    #         removLabel = concatenationCost(solver.res, i, solver.forwardLabels[r][i-1], solver.backwardLabels[r][length(solver.currSol.routes[r]) + 1 - i - 1])
    #         if removLabel.cost < Inf
    #             dFeasBackward = -2
    #         else
    #             dFeasBackward = -length(solver.currSol.routes[r])-1
    #         end
    #         # println("FEASIBLE ROUTE: $(removLabel)")
    #     else
    #         removLabel = solver.extendAlongArc(solver.res, copy(solver.backwardLabels[r][i-1]), (solver.currSol.routes[r][i+1]+1, solver.currSol.routes[r][i-1]+1))
    #         if i > length(solver.currSol.routes[r]) - solver.currSol.lastFeasibleB[r] + 1
    #             dFeasBackward = - 2
    #             if removLabel.cost < Inf
    #                 dFeasBackward += 1
    #             else
    #                 dFeasBackward += 0
    #             end
    #             # println("i < : ")

    #         elseif i == length(solver.currSol.routes[r]) - solver.currSol.lastFeasibleB[r] + 1
    #             if removLabel.cost < Inf
    #                 dFeasBackward = 1
    #             else
    #                 dFeasBackward = 0
    #             end
    #             # println("i == : ")

    #         elseif i < length(solver.currSol.routes[r]) - solver.currSol.lastFeasibleB[r] + 1
    #             # println("i > ")
    #             dFeasBackward = -typemax(Int)
    #         end
    #     end
    #     return max(solver.currSol.feasiblesF[r] + dFeasForward, solver.currSol.feasiblesB[r] + dFeasForward)
    # end
=#

function computeViolIntraShift10(solver::Solver, r::Int, i::Int, j::Int)
    # return solver.currSol.resViolation[r]
    if i < j
        if j == i+1
            solver.prevLabelF = solver.extendAlongArc(solver.res, copy(solver.forwardLabels[r][i-1]), (solver.currSol.routes[r][i-1]+1, solver.currSol.routes[r][i+1] + 1))
            auxLabel = solver.extendAlongArc(solver.res, copy(solver.prevLabelF), (solver.currSol.routes[r][j]+1, solver.currSol.routes[r][i] + 1))
            res = solver.concatenationCost(solver.res, i, auxLabel, solver.backwardLabels[r][length(solver.currSol.routes[r]) + 1 - j - 1])
        else
            solver.prevLabelF = solver.extendAlongArc(solver.res, copy(solver.prevLabelF), (solver.currSol.routes[r][i-1]+1, solver.currSol.routes[r][j] + 1))
            auxLabel = solver.extendAlongArc(solver.res, copy(solver.prevLabelF), (solver.currSol.routes[r][j]+1, solver.currSol.routes[r][i] + 1))
            res = solver.concatenationCost(solver.res, i, auxLabel, solver.backwardLabels[r][length(solver.currSol.routes[r]) + 1 - j - 1])
        end
    else
        if j == i-1
            solver.prevLabelB = solver.extendAlongArc(solver.res, copy(solver.backwardLabels[r][length(solver.currSol.routes[r]) - i]), (solver.currSol.routes[r][i+1]+1, solver.currSol.routes[r][j] + 1))
            auxLabel = solver.extendAlongArc(solver.res, copy(solver.prevLabelB), (solver.currSol.routes[r][j]+1, solver.currSol.routes[r][j+1] + 1))
            res = solver.concatenationCost(solver.res, j+1, auxLabel, solver.forwardLabels[r][j - 1])
        else
            solver.prevLabelB = solver.extendAlongArc(solver.res, copy(solver.prevLabelB), (solver.currSol.routes[r][i+1]+1, solver.currSol.routes[r][j] + 1))
            auxLabel = solver.extendAlongArc(solver.res, copy(solver.prevLabelB), (solver.currSol.routes[r][j]+1, solver.currSol.routes[r][i] + 1))
            res = solver.concatenationCost(solver.res, j+1, auxLabel, solver.forwardLabels[r][j - 1])
        end
    end
    if res.cost >= Inf
        return 1
    else
        return 0
    end
end

function computeViolIntraShift20(solver::Solver, r::Int)
    return solver.currSol.resViolation[r]

end

function computeViolIntraSwap11()
    return 0 
end

function computeViol2opt()
    return 0 
end

function computeViolInterShift10(solver::Solver, r1::Int, r2::Int, i::Int, j::Int)
    feas = computeViolRemove1(solver, r1, i)
    feas += computeViolInsertion1(solver, r2, solver.currSol.routes[r1][i], j)
    return feas
    if solver.currSol.feasibles[r1] == length(solver.currSol.routes[r1]) - 2
        removLabel = concatenationCost(solver.res, i, solver.forwardLabels[r1][i-1], solver.backwardLabels[r1][length(solver.currSol.routes[r1]) + 1 - i - 1])
        if removLabel.cost < Inf
            dFeas = -2
        else
            dFeas = -length(solver.currSol.routes[r1])-1
        end
        dFeas += computeViolInsertion(solver, r2, i, j)
    else

    end
    sleep(1000)
    concatCost1 = concatenationCost(solver.res, i, solver.forwardLabels[r1][i-1], solver.backwardLabels[r1][length(solver.currSol.routes[r1]) + 1 - i - 1])
    viol1 = 0
    if concatCost1.cost >= Inf
        viol1 += 1
    end
    prev1 = extendAlongArc(solver.res, copy(solver.forwardLabels[r2][j-1]), (solver.currSol.routes[r2][j-1] + 1, solver.currSol.routes[r1][i] + 1))
    concatCost2 = concatenationCost(solver.res, i, prev1, solver.backwardLabels[r2][length(solver.currSol.routes[r2]) + 1 - j])
    viol2 = 0
    if concatCost2.cost >= Inf
        viol2 += 1
    end

    return viol1, viol2
end

function computeViolInterShift20(solver::Solver, r1::Int, r2::Int, i::Int, j::Int)
    concatCost1 = concatenationCost(solver.res, i, solver.forwardLabels[r1][i-1], solver.backwardLabels[r1][length(solver.currSol.routes[r1]) + 1 - i - 2])
    viol1 = 0
    if concatCost1.cost >= Inf
        viol1 += 1
    end
    prev1 = extendAlongArc(solver.res, copy(solver.forwardLabels[r2][j-1]), (solver.currSol.routes[r2][j-1] + 1, solver.currSol.routes[r1][i] + 1))
    prev1 = extendAlongArc(solver.res, prev1, (solver.currSol.routes[r1][i] + 1, solver.currSol.routes[r1][i+1] + 1))
    concatCost2 = concatenationCost(solver.res, i, prev1, solver.backwardLabels[r2][length(solver.currSol.routes[r2]) + 1 - j])
    viol2 = 0
    if concatCost2.cost >= Inf
        viol2 += 1
    end

    return viol1, viol2
end

function computeViolInterSwap11(solver::Solver, r1::Int, r2::Int, i::Int, j::Int)
    feas = computeViolRemove1(solver, r1, i)
    # @show computeViolRemove(solver, r1, i)
    feas += computeViolInsertion1(solver, r1, solver.currSol.routes[r2][j], i)
    # @show computeViolInsertion(solver, r1, j, i)
    feas += computeViolRemove1(solver, r2, j)
    # @show computeViolRemove(solver, r2, j)
    feas += computeViolInsertion1(solver, r2, solver.currSol.routes[r1][i], j)
    # @show computeViolInsertion(solver, r2, i, j)
    # @show feas
    return feas
    prev1 = extendAlongArc(solver.res, copy(solver.forwardLabels[r1][i-1]), (solver.currSol.routes[r1][i-1] + 1, solver.currSol.routes[r2][j] + 1))
    concatCost1 = concatenationCost(solver.res, i, prev1, solver.backwardLabels[r1][length(solver.currSol.routes[r1]) + 1 - i - 1])
    viol1 = 0
    if concatCost1.cost >= Inf
        viol1 += 1
    end
    prev2 = extendAlongArc(solver.res, copy(solver.forwardLabels[r2][j-1]), (solver.currSol.routes[r2][j-1] + 1, solver.currSol.routes[r1][i] + 1))
    concatCost2 = concatenationCost(solver.res, i, prev2, solver.backwardLabels[r2][length(solver.currSol.routes[r2]) + 1 - j - 1])
    viol2 = 0
    if concatCost2.cost >= Inf
        viol2 += 1
    end

    return viol1, viol2
end