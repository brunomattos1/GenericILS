function computeViolInsertion1(solver::Solver, r::Int, customer::Int, pos::Int)
    insertionLabelForward = solver.extendAlongArc(solver.res, copy(solver.forwardLabels[r][pos-1]), (solver.currSol.routes[r][pos-1]+1, customer + 1))
    concatCost = solver.concatenationCost(solver.res, customer, insertionLabelForward, solver.backwardLabels[r][length(solver.currSol.routes[r]) + 1 - pos])
    # route = deepcopy(solver.currSol.routes[r])
    # insert!(route, pos, customer)
    # @show concatCost
    if concatCost.cost < Inf
        # println("Concat Insertion1: $(concatCost)")
        return length(solver.currSol.routes[r]) - 2 + 1
    end

    # FORWARD
    if solver.currSol.feasiblesF[r] == length(solver.currSol.routes[r]) - 2
        concatCost = solver.concatenationCost(solver.res, customer, insertionLabelForward, solver.backwardLabels[r][length(solver.currSol.routes[r]) + 1 - pos])
        if insertionLabelForward.cost == Inf
            # dFeasForward = - length(solver.currSol.routes[r]) - 2
            dFeasForward = 0 - length(solver.currSol.routes[r]) + pos
            if pos == length(solver.currSol.routes[r])
                dFeasForward = - 1
            end
            # no caso que a rota eh viavel, dFeas é -length + pos
        elseif insertionLabelForward.cost < Inf && concatCost.cost < Inf
            dFeasForward = 1
        elseif insertionLabelForward.cost < Inf && concatCost.cost == Inf
            # dFeasForward = - length(solver.currSol.routes[r]) - 1
            dFeasForward = 1 - length(solver.currSol.routes[r]) + pos
            # println("< inf, == inf")
        end
        # if route != concatCost.path
        #     @show route
        #     @show insertionLabelForward
        #     @show concatCost
        #     @show r, customer, pos
        #     @show solver.currSol.routes[r]
        #     @show solver.backwardLabels[r]
        #     sleep(100)
        # end
        # @show concatCost, dFeasForward
    else
        insertionLabelForward2 = solver.extendAlongArc(solver.res, copy(insertionLabelForward), (customer + 1, solver.currSol.routes[r][pos]+1))
        if pos < solver.currSol.lastFeasibleF[r] + 1
            dFeasForward = -1
            insertionLabelForward.cost < Inf ? dFeasForward += 1 : 0
            insertionLabelForward2.cost < Inf ? dFeasForward += 1 : 0   
        end
        if pos == solver.currSol.lastFeasibleF[r] + 1
            dFeasForward = 0
            insertionLabelForward.cost < Inf ? dFeasForward += 1 : 0
            insertionLabelForward2.cost < Inf ? dFeasForward += 1 : 0
        end
        # TO DO
        if pos > solver.currSol.lastFeasibleF[r] + 1
            # println("here")
            dFeasForward = -typemax(Int)
        end
        # @show insertionLabelForward2, dFeasForward
    end

    # BACKWARD
    if solver.currSol.feasiblesB[r] == length(solver.currSol.routes[r]) - 2
        insertionLabelBackward = solver.extendAlongArc(solver.res, copy(solver.backwardLabels[r][length(solver.currSol.routes[r]) - pos + 1]), (solver.currSol.routes[r][pos]+1, customer+1))
        concatCost = solver.concatenationCost(solver.res, customer, insertionLabelBackward, solver.forwardLabels[r][pos - 1])
        if insertionLabelBackward.cost == Inf
            dFeasBackward = 0 - length(solver.currSol.routes[r]) - pos - 2
        elseif insertionLabelBackward.cost < Inf && concatCost.cost < Inf
            dFeasBackward = 1
        elseif insertionLabelBackward.cost < Inf && concatCost.cost == Inf
            dFeasBackward = 1 - length(solver.currSol.routes[r]) - pos - 2
        end
        # if reverse(route) != concatCost.path
        #     @show reverse(route)
        #     @show concatCost
        #     @show r, customer, pos
        #     @show insertionLabelBackward
        #     @show solver.backwardLabels[r]
        #     sleep(100)
        # end
        # @show insertionLabelBackward
        # @show concatCost, dFeasBackward
    else
        insertionLabelBackward = solver.extendAlongArc(solver.res, copy(solver.backwardLabels[r][length(solver.currSol.routes[r]) - pos + 1]), (solver.currSol.routes[r][pos]+1, customer+1))
        insertionLabelBackward2 = solver.extendAlongArc(solver.res, copy(insertionLabelBackward), (customer + 1, solver.currSol.routes[r][pos-1]+1))
        if pos > length(solver.currSol.routes[r]) - solver.currSol.lastFeasibleB[r] + 1
            dFeasBackward = -1
            insertionLabelBackward.cost < Inf ? dFeasBackward += 1 : 0
            insertionLabelBackward2.cost < Inf ? dFeasBackward += 1 : 0   
        end
        if pos == length(solver.currSol.routes[r]) - solver.currSol.lastFeasibleB[r] + 1
            dFeasBackward = 0
            insertionLabelBackward.cost < Inf ? dFeasBackward += 1 : 0
            insertionLabelBackward2.cost < Inf ? dFeasBackward += 1 : 0
        end
        if pos < length(solver.currSol.routes[r]) - solver.currSol.lastFeasibleB[r] + 1
            # println("here B")
            dFeasBackward = -typemax(Int)
        end
        # @show insertionLabelBackward2, dFeasBackward
    end
    # usar quantidade de viaveis ate o ponto de inserção + dFeas, tanto forward quanto backward
    # return max(solver.currSol.feasiblesF[r] + dFeasForward, solver.currSol.feasiblesB[r] + dFeasBackward)
    # @show solver.currSol.lastFeasibleF[r]-1, dFeasForward, solver.currSol.lastFeasibleB[r]-1, dFeasBackward
    forw = dFeasForward
    if pos <= solver.currSol.lastFeasibleF[r]
        forw += pos - 1
    else
        forw += solver.currSol.lastFeasibleF[r] - 1
    end

    backw = dFeasBackward
    if pos >= length(solver.currSol.routes[r]) + 1 - solver.currSol.lastFeasibleB[r]
        # backw += length(solver.currSol.routes[r]) + 1 - solver.currSol.lastFeasibleB[r] - 1
        backw += length(solver.currSol.routes[r]) - pos -1
    else
        backw += solver.currSol.lastFeasibleB[r] - 1
    end
    
    # println("pos $pos, lastFeasibleB: $(length(solver.currSol.routes[r]) + 1 - solver.currSol.lastFeasibleB[r])")
    # println("dfeasForw $(dFeasForward), dfeasBackw $(dFeasBackward)")
    # println("backw $backw, forw $forw")
    return max(forw, backw)
end

function computeViolRemove1(solver::Solver, r::Int, pos::Int)
    concat = solver.concatenationCost(solver.res, pos, solver.forwardLabels[r][pos-1], solver.backwardLabels[r][length(solver.currSol.routes[r]) + 1 - pos - 1])
    removLabelForward = nothing
    if concat.cost < Inf
        # println("Concat Remove1: $(concat)")
        return length(solver.currSol.routes[r]) - 2 - 1
    end
    # FORWARD
    if solver.currSol.feasiblesF[r] == length(solver.currSol.routes[r]) - 2
        removLabelForward = solver.concatenationCost(solver.res, pos, solver.forwardLabels[r][pos-1], solver.backwardLabels[r][length(solver.currSol.routes[r]) + 1 - pos - 1])
        if removLabelForward.cost < Inf
            dFeasForward = -1
        else
            dFeasForward = -length(solver.currSol.routes[r])
        end
    else
        if pos < solver.currSol.lastFeasibleF[r] + 1
            dFeasForward = - 2
            removLabelForward = solver.extendAlongArc(solver.res, copy(solver.forwardLabels[r][pos-1]), (solver.currSol.routes[r][pos-1]+1, solver.currSol.routes[r][pos+1]+1))
            if removLabelForward.cost < Inf
                dFeasForward += 1
            else
                dFeasForward += 0
            end
        elseif pos == solver.currSol.lastFeasibleF[r] + 1
            removLabelForward = solver.extendAlongArc(solver.res, copy(solver.forwardLabels[r][pos-1]), (solver.currSol.routes[r][pos-1]+1, solver.currSol.routes[r][pos+1]+1))
            if removLabelForward.cost < Inf
                dFeasForward = 1
            else
                dFeasForward = 0
            end
        elseif pos > solver.currSol.lastFeasibleF[r] + 1
            dFeasForward = -typemax(Int)
        end
    end

    # BACKWARD
    if solver.currSol.feasiblesB[r] == length(solver.currSol.routes[r]) - 2
        removLabelBackward = solver.concatenationCost(solver.res, pos, solver.forwardLabels[r][pos], solver.backwardLabels[r][length(solver.currSol.routes[r]) + 1 - pos - 1])
        if removLabelBackward.cost < Inf
            dFeasBackward = -1
        else
            dFeasBackward = -length(solver.currSol.routes[r])
        end
    else
        removLabelBackward = solver.extendAlongArc(solver.res, copy(solver.backwardLabels[r][pos]), (solver.currSol.routes[r][pos+1]+1, solver.currSol.routes[r][pos-1]+1))
        if pos > length(solver.currSol.routes[r]) - solver.currSol.lastFeasibleB[r] + 1
            dFeasBackward = - 2
            if removLabelBackward.cost < Inf
                dFeasBackward += 1
            else
                dFeasBackward += 0
            end
        elseif pos == length(solver.currSol.routes[r]) - solver.currSol.lastFeasibleB[r] + 1
            # println("==")
            # @show removLabelBackward
            # @show solver.backwardLabels[r][pos]
            if removLabelBackward.cost < Inf
                dFeasBackward = 1
            else
                dFeasBackward = 0
            end
        elseif pos < length(solver.currSol.routes[r]) - solver.currSol.lastFeasibleB[r] + 1
            dFeasBackward = -typemax(Int)
        end
    end
    # forw = dFeasForward + solver.currSol.feasiblesF[r]
    # backw = dFeasBackward + solver.currSol.feasiblesB[r]
    
    forw = dFeasForward
    if pos <= solver.currSol.lastFeasibleF[r]
        forw += pos - 2
    else
        forw += solver.currSol.lastFeasibleF[r] - 1 - 1
    end

    backw = dFeasBackward
    if pos > length(solver.currSol.routes[r]) + 1 - solver.currSol.lastFeasibleB[r]
        # backw += length(solver.currSol.routes[r]) + 1 - solver.currSol.lastFeasibleB[r] - 2
        backw += length(solver.currSol.routes[r]) - pos + 1 - 1
    end
    if pos == length(solver.currSol.routes[r]) + 1 - solver.currSol.lastFeasibleB[r]
        backw += length(solver.currSol.routes[r]) - pos - 2
    end
    if pos < length(solver.currSol.routes[r]) + 1 - solver.currSol.lastFeasibleB[r]
        backw += solver.currSol.lastFeasibleB[r] - 1 - 1
    end
    # println("$pos, $(length(solver.currSol.routes[r]) + 1 - solver.currSol.lastFeasibleB[r])")
    # println("dfeasForw: $(dFeasForward), dFeasBackward: $(dFeasBackward), forw: $(forw), backw: $(backw)")
    return max(forw, backw)
    # return max(solver.currSol.feasiblesF[r] + dFeasForward, solver.currSol.feasiblesB[r] + dFeasBackward)
end

function computeViolSwap11(solver::Solver, r::Int, pos::Int, customer::Int)
    # FORWARD
    # @show r, pos, customer
    if solver.currSol.feasiblesF[r] == length(solver.currSol.routes[r]) - 2
        swapLabelForward = solver.extendAlongArc(solver.res, copy(solver.forwardLabels[r][pos-1]), (solver.currSol.routes[r][pos-1]+1, customer + 1))
        concatCost = solver.concatenationCost(solver.res, customer, swapLabelForward, solver.backwardLabels[r][length(solver.currSol.routes[r]) - pos])
        if swapLabelForward.cost == Inf
            dFeasForward = 0 - length(solver.currSol.routes[r]) + pos
        elseif swapLabelForward.cost < Inf && concatCost.cost < Inf
            dFeasForward = 0
        elseif swapLabelForward.cost < Inf && concatCost.cost == Inf
            dFeasForward = 1 - length(solver.currSol.routes[r]) + pos
        end
        # println("case 1 F: $(swapLabelForward) $(concatCost)")
    else
        # tentar um concat solver.forwardLabels[r][pos-1] + customer + backward
        # só faz o resto se der inviavel
        swapLabelForward = solver.extendAlongArc(solver.res, copy(solver.forwardLabels[r][pos-1]), (solver.currSol.routes[r][pos-1]+1, customer + 1))
        swapLabelForward2 = solver.extendAlongArc(solver.res, copy(swapLabelForward), (customer + 1, solver.currSol.routes[r][pos+1]+1))
        if pos < solver.currSol.lastFeasibleF[r] + 1
            dFeasForward = -1
            swapLabelForward.cost < Inf ? dFeasForward += 1 : 0
            swapLabelForward2.cost < Inf ? dFeasForward += 1 : 0   
        end
        if pos == solver.currSol.lastFeasibleF[r] + 1
            dFeasForward = -1
            swapLabelForward.cost < Inf ? dFeasForward += 1 : 0
            swapLabelForward2.cost < Inf ? dFeasForward += 1 : 0
        end
        # TO DO
        if pos > solver.currSol.lastFeasibleF[r] + 1
            dFeasForward = -typemax(Int)
        end
        # println("case 2 F: $(swapLabelForward2)")
    end

    # BACKWARD
    if solver.currSol.feasiblesB[r] == length(solver.currSol.routes[r]) - 2
        # Corrigir as extensoes
        # prev = solver.extendAlongArc(solver.res, copy(solver.backwardLabels[r][pos-1]), (solver.currSol.routes[r][pos-1]+1, customer + 1))
        swapLabelBackward = solver.extendAlongArc(solver.res, copy(solver.backwardLabels[r][length(solver.currSol.routes[r]) - pos]), (solver.currSol.routes[r][pos]+1, customer+1))
        concatCost = solver.concatenationCost(solver.res, customer, swapLabelBackward, solver.forwardLabels[r][pos-1])
        # @show concatCost
        if swapLabelBackward.cost == Inf
            dFeasBackward = 0 - length(solver.currSol.routes[r]) + pos
        elseif swapLabelBackward.cost < Inf && concatCost.cost < Inf
            dFeasBackward = 0
        elseif swapLabelBackward.cost < Inf && concatCost.cost == Inf
            dFeasBackward = 1 - length(solver.currSol.routes[r]) + pos
        end
        # println("case 1 B: $(concatCost)")
    else
        swapLabelBackward = solver.extendAlongArc(solver.res, copy(solver.backwardLabels[r][length(solver.currSol.routes[r]) - pos]), (solver.currSol.routes[r][pos]+1, customer+1))
        swapLabelBackward2 = solver.extendAlongArc(solver.res, copy(swapLabelBackward), (customer + 1, solver.currSol.routes[r][pos-1]+1))
        if pos > length(solver.currSol.routes[r]) - solver.currSol.lastFeasibleB[r] + 1
            dFeasBackward = -1
            swapLabelBackward.cost < Inf ? dFeasBackward += 1 : 0
            swapLabelBackward2.cost < Inf ? dFeasBackward += 1 : 0   
        end
        if pos == length(solver.currSol.routes[r]) - solver.currSol.lastFeasibleB[r] + 1
            dFeasBackward = 0
            swapLabelBackward.cost < Inf ? dFeasBackward += 1 : 0
            swapLabelBackward2.cost < Inf ? dFeasBackward += 1 : 0
        end
        if pos < length(solver.currSol.routes[r]) - solver.currSol.lastFeasibleB[r] + 1
            dFeasBackward = -typemax(Int)
        end
        # println("case 2 B: $(swapLabelBackward2)")
    end
    # println("dFeasForw: $(dFeasForward), dFeasBackward: $(dFeasBackward)")
    forw = dFeasForward
    if pos < solver.currSol.lastFeasibleF[r]
        forw += pos - 1
    end
    if pos >= solver.currSol.lastFeasibleF[r]
        forw += solver.currSol.lastFeasibleF[r] - 1
    end

    backw = dFeasBackward
    if pos >= length(solver.currSol.routes[r]) + 1 - solver.currSol.lastFeasibleB[r]
        backw += length(solver.currSol.routes[r]) - pos -1
    else
        backw += solver.currSol.lastFeasibleB[r] - 1
    end
    # println("pos: $(pos), $(length(solver.currSol.routes[r]) + 1 - solver.currSol.lastFeasibleB[r])")
    # println("forw: $(forw), backw: $(backw)")
    return max(forw, backw)
end

function computeViolTwoOptStar(solver::Solver, r1::Int, r2::Int, i::Int, j::Int)
    # FORWARD
    label1 = solver.concatenationCost(solver.res, 1, solver.forwardLabels[r1][i], solver.backwardLabels[r2][length(solver.currSol.routes[r2]) - j])
    @show label1
    label2 = solver.concatenationCost(solver.res, 1, solver.forwardLabels[r2][j], solver.backwardLabels[r1][length(solver.currSol.routes[r1]) - i])
    @show label2
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

# TO DO
#=
function computeViolRemove2(solver::Solver, r::Int, i::Int)
    # FORWARD
    if solver.currSol.feasiblesF[r] == length(solver.currSol.routes[r]) - 2
        removLabel = concatenationCost(solver.res, i, solver.forwardLabels[r][i-1], solver.backwardLabels[r][length(solver.currSol.routes[r]) + 1 - i - 1])
        if removLabel.cost < Inf
            dFeasForward = -2
        else
            dFeasForward = -length(solver.currSol.routes[r])-1
        end
        # println("FEASIBLE ROUTE: $(removLabel)")
    else
        if i < solver.currSol.lastFeasibleF[r] + 1
            dFeasForward = - 2
            removLabel = solver.extendAlongArc(solver.res, copy(solver.forwardLabels[r][i-1]), (solver.currSol.routes[r][i-1]+1, solver.currSol.routes[r][i+1]+1))
            if removLabel.cost < Inf
                dFeasForward += 1
            else
                dFeasForward += 0
            end
            # println("i < : $(removLabel)")

        elseif i == solver.currSol.lastFeasibleF[r] + 1
            removLabel = solver.extendAlongArc(solver.res, copy(solver.forwardLabels[r][i-1]), (solver.currSol.routes[r][i-1]+1, solver.currSol.routes[r][i+1]+1))
            if removLabel.cost < Inf
                dFeasForward = 1
            else
                dFeasForward = 0
            end
            # println("i == : $(removLabel)")

        elseif i > solver.currSol.lastFeasibleF[r] + 1
            # println("i > ")
            dFeasForward = -typemax(Int)
        end
    end
    
    # BACKWARD
    if solver.currSol.feasiblesB[r] == length(solver.currSol.routes[r]) - 2
        removLabel = concatenationCost(solver.res, i, solver.forwardLabels[r][i-1], solver.backwardLabels[r][length(solver.currSol.routes[r]) + 1 - i - 1])
        if removLabel.cost < Inf
            dFeasBackward = -2
        else
            dFeasBackward = -length(solver.currSol.routes[r])-1
        end
        # println("FEASIBLE ROUTE: $(removLabel)")
    else
        removLabel = solver.extendAlongArc(solver.res, copy(solver.backwardLabels[r][i-1]), (solver.currSol.routes[r][i+1]+1, solver.currSol.routes[r][i-1]+1))
        if i > length(solver.currSol.routes[r]) - solver.currSol.lastFeasibleB[r] + 1
            dFeasBackward = - 2
            if removLabel.cost < Inf
                dFeasBackward += 1
            else
                dFeasBackward += 0
            end
            # println("i < : ")

        elseif i == length(solver.currSol.routes[r]) - solver.currSol.lastFeasibleB[r] + 1
            if removLabel.cost < Inf
                dFeasBackward = 1
            else
                dFeasBackward = 0
            end
            # println("i == : ")

        elseif i < length(solver.currSol.routes[r]) - solver.currSol.lastFeasibleB[r] + 1
            # println("i > ")
            dFeasBackward = -typemax(Int)
        end
    end
    return max(solver.currSol.feasiblesF[r] + dFeasForward, solver.currSol.feasiblesB[r] + dFeasForward)
end
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
    remove1Feas = computeViolRemove1(solver, r1, i)
    # println("Feas after remove1: $(remove1Feas)")
    insertion1Feas = computeViolInsertion1(solver, r2, solver.currSol.routes[r1][i], j)
    # println("Feas after insertion1: $(insertion1Feas)")

    return remove1Feas + insertion1Feas
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
    feasR1 = computeViolSwap11(solver, r1, i, solver.currSol.routes[r2][j])
    # println("Feas R1: $(feasR1)")
    feasR2 = computeViolSwap11(solver, r2, j, solver.currSol.routes[r1][i])
    # println("Feas R2: $(feasR2)")

    return feasR1 + feasR2
end

