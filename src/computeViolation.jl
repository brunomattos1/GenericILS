function computeViolInsertion1(solver::Solver, sol::Solution, r::Int, customer::Int, pos::Int)

    # Primeiro eu tento uma concatenação, em que estendo $sol.routes[r][1:pos-1] -> $customer gerando um label forward
    # e depois estendo $customer <- $sol.routes[r][pos, end] gerando um label backward
    insertionLabelForward = myExtendAlongArc(solver.res.customResource, sol.forwardLabels[r][pos-1], (sol.routes[r][pos-1]+1, customer + 1))
    insertionLabelBackward = myExtendAlongArc(solver.res.customResource, sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos], (sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos].last +1, customer + 1))
    concatCost = myConcatenationCost(solver.res.customResource, insertionLabelBackward.last, insertionLabelForward, insertionLabelBackward)

    if concatCost.cost < Inf
        return length(sol.routes[r]) - 1 + 1
    end

    # FORWARD
    if sol.feasiblesF[r] == length(sol.routes[r]) - 1
        # label de $r[pos:end]
        backwardLabel = sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos]
        # estendo o $insertionLabel (que ja possui o $customer) para o proximo cliente da rota (backwardLabel.last)
        insertionLabelForward2 = myExtendAlongArc(solver.res.customResource, insertionLabelForward, (insertionLabelForward.last + 1, backwardLabel.last + 1))
        concatCost = myConcatenationCost(solver.res.customResource, backwardLabel.last, insertionLabelForward2, backwardLabel)

        # Se a inserção é numa parte inviável da rota, não há perda de viabilidade
        if pos > sol.lastFeasibleF[r]
            dFeasForward = 0
        else # Se a inserção é numa parte viavel da rota, eu perco 1 arco viável que conecta $sol.routes[r][pos-1] ao $sol.routes[r][pos]
            dFeasForward = - 1
        end
        # if pos < sol.lastFeasibleF[r]
        #     dFeasForward = -1
        # elseif pos == sol.lastFeasibleF[r] - 1
        #     dFeasForward = -1
        # elseif pos == length(sol.routes[r])
        #     dFeasForward = - 1
        # else #pos > sol.lastFeasibleF[r] - 1
        #     dFeasForward = 0
        # end
        # @show pos, sol.lastFeasibleF[r]
        # @show dFeasForward
        if insertionLabelForward.cost < Inf
            dFeasForward += 1
        end
        if insertionLabelForward2.cost < Inf
            dFeasForward += 1
        end
        # if insertionLabelForward2.cost == Inf
        #     dFeasForward = 0 - length(sol.routes[r]) + pos
        #     if pos == length(sol.routes[r])
        #         dFeasForward = - 1
        #     end
        # elseif insertionLabelForward2.cost < Inf && concatCost.cost < Inf
        #     dFeasForward = 1
        # elseif insertionLabelForward2.cost < Inf && concatCost.cost == Inf
        #     dFeasForward = 1 - length(sol.routes[r]) + pos
        # end

        forw = dFeasForward
        # @show insertionLabelForward
        # @show insertionLabelForward2
        # @show concatCost
        # @show dFeasForward
        # @show dFeasForward
        if pos <= sol.lastFeasibleF[r]
            forw += pos - 1
        else
            forw += sol.lastFeasibleF[r] - 1
        end
        # @show forw

    else
        insertionLabelForward2 = myExtendAlongArc(solver.res.customResource, insertionLabelForward, (customer + 1, sol.routes[r][pos]+1))
        if pos < sol.lastFeasibleF[r] + 1
            dFeasForward = -1
            insertionLabelForward.cost < Inf ? dFeasForward += 1 : 0
            insertionLabelForward2.cost < Inf ? dFeasForward += 1 : 0   
        end
        if pos == sol.lastFeasibleF[r] + 1
            dFeasForward = 0
            insertionLabelForward.cost < Inf ? dFeasForward += 1 : 0
            insertionLabelForward2.cost < Inf ? dFeasForward += 1 : 0
        end
        
        if pos > sol.lastFeasibleF[r] + 1
            dFeasForward = 0#-typemax(Int)
        end
        # @show insertionLabelForward
        # @show insertionLabelForward2
        # @show dFeasForward
        forw = dFeasForward
        if pos <= sol.lastFeasibleF[r]
            forw += pos - 1
        else
            forw += sol.lastFeasibleF[r] - 1
        end
        # @show forw
    end
    # println("-"^100)
    # BACKWARD
    if sol.feasiblesB[r] == length(sol.routes[r]) - 1
        
        insertionLabelBackward2 = myExtendAlongArc(solver.res.customResource, insertionLabelBackward, (customer + 1, sol.routes[r][pos-1] + 1))
        # concatCost = myConcatenationCost(solver.res.customResource, insertionLabelBackward2.last, sol.forwardLabels[r][pos], insertionLabelBackward2)

        # A inserção é numa região víavel? Se sim, perco um arco de viabilidade
        if pos > length(sol.routes[r]) + 1 - sol.lastFeasibleB[r]
            dFeasBackward = -1
        else # Senão, não perco viabilidade
            dFeasBackward = 0
        end

        if insertionLabelBackward.cost < Inf
            dFeasBackward += 1
        end
        if insertionLabelBackward2.cost < Inf
            dFeasBackward += 1
        end
        
        # if insertionLabelBackward.cost == Inf
        #     dFeasBackward = -1#length(sol.routes[r]) - pos - 2
        # elseif insertionLabelBackward.cost < Inf && concatCost.cost < Inf
        #     dFeasBackward = 1#length(sol.routes[r]) - 1
        # elseif insertionLabelBackward.cost < Inf && concatCost.cost == Inf
        #     dFeasBackward = 0#length(sol.routes[r]) - pos - 2
        # end

        # @show insertionLabelBackward
        # @show insertionLabelBackward2
        # @show concatCost
        backw = dFeasBackward
        if pos > length(sol.routes[r]) + 1 - sol.lastFeasibleB[r]
            backw += length(sol.routes[r]) - pos + 1# - 1# + 1
        else
            backw += sol.lastFeasibleB[r] - 1
        end
    else
        # insertionLabelBackward = myExtendAlongArc(solver.res.customResource, sol.backwardLabels[r][length(sol.routes[r]) - pos + 1], (sol.routes[r][pos]+1, customer+1))
        # insertionLabelBackward2 = myExtendAlongArc(solver.res.customResource, insertionLabelBackward, (customer + 1, sol.routes[r][pos-1]+1))
        insertionLabelBackward2 = myExtendAlongArc(solver.res.customResource, insertionLabelBackward, (customer + 1, sol.routes[r][pos-1] + 1))
        # @show insertionLabelBackward2
        if pos > length(sol.routes[r]) - sol.lastFeasibleB[r] + 1
            dFeasBackward = -1
            insertionLabelBackward.cost < Inf ? dFeasBackward += 1 : 0
            insertionLabelBackward2.cost < Inf ? dFeasBackward += 1 : 0   
        end
        if pos == length(sol.routes[r]) - sol.lastFeasibleB[r] + 1
            dFeasBackward = 0
            insertionLabelBackward.cost < Inf ? dFeasBackward += 1 : 0
            insertionLabelBackward2.cost < Inf ? dFeasBackward += 1 : 0
        end
        if pos < length(sol.routes[r]) - sol.lastFeasibleB[r] + 1
            dFeasBackward = 0#-typemax(Int)
        end
        backw = dFeasBackward
        # @show dFeasBackward
        if pos > length(sol.routes[r]) + 1 - sol.lastFeasibleB[r]
            backw += length(sol.routes[r]) - pos + 1# - 1# + 1
        else
            backw += sol.lastFeasibleB[r] - 1
        end
    end
    # @show forw, backw
    return max(forw, backw)
end

function computeViolRemove1(solver::Solver, sol::Solution, r::Int, pos::Int)
    removeLabelForward = myExtendAlongArc(solver.res.customResource, sol.forwardLabels[r][pos-1], (sol.forwardLabels[r][pos-1].last+1, sol.routes[r][pos+1]+1))
    concat = myConcatenationCost(solver.res.customResource, sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos - 1].last, removeLabelForward, sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos - 1])
    if concat.cost < Inf
        return length(sol.routes[r]) - 1 - 1
    end
    # FORWARD
    if sol.feasiblesF[r] == length(sol.routes[r]) - 1
        concatForward = myConcatenationCost(solver.res.customResource, sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos - 1].last, removeLabelForward, sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos - 1])
        if concatForward.cost < Inf
            dFeasForward = -1
        else
            dFeasForward = -length(sol.routes[r])
        end
        forw = length(sol.routes[r]) - 1 - dFeasForward - 2
    else
        # removLabelForward = myExtendAlongArc(solver.res.customResource, sol.forwardLabels[r][pos-1], (sol.routes[r][pos-1]+1, sol.routes[r][pos+1]+1))
        if pos < sol.lastFeasibleF[r]
            dFeasForward = - 2
            if removeLabelForward.cost < Inf
                dFeasForward += 1
            else
                dFeasForward += 0
            end
        elseif pos == sol.lastFeasibleF[r]
            dFeasForward = - 1
            if removeLabelForward.cost < Inf
                dFeasForward += 1
            else
                dFeasForward += 0
            end
        elseif pos == sol.lastFeasibleF[r] + 1
            dFeasForward = 0
            if removeLabelForward.cost < Inf
                dFeasForward += 1
            else
                dFeasForward += 0
            end
        elseif pos > sol.lastFeasibleF[r]
            dFeasForward = 0
        end
        # @show removeLabelForward
        # @show dFeasForward
        forw = dFeasForward
        if pos < sol.lastFeasibleF[r]
            forw += pos - 1
        end
        if pos == sol.lastFeasibleF[r]
            forw += pos - 1
        end
        if pos > sol.lastFeasibleF[r]
            forw += sol.lastFeasibleF[r] - 1
        end
        # @show forw
    end

    # BACKWARD
    if sol.feasiblesB[r] == length(sol.routes[r]) - 1
        removeLabelBackward = myExtendAlongArc(solver.res.customResource, sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos - 1], (sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos - 1].last+1, sol.routes[r][pos-1]+1))
        if removeLabelBackward.cost < Inf
            dFeasBackward = -1
        else
            dFeasBackward = -length(sol.routes[r])
        end
        backw = length(sol.routes[r]) - 1 - dFeasBackward - 2
    else
        # pos_ = length(sol.routes[r]) - pos#+ 1# - 1
        # removLabelBackward = myExtendAlongArc(solver.res.customResource, sol.backwardLabels[r][pos_], (sol.routes[r][pos_]+1, sol.routes[r][pos-1]+1))
        removeLabelBackward = myExtendAlongArc(solver.res.customResource, sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos - 1], (sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos - 1].last+1, sol.routes[r][pos-1]+1))

        if pos > length(sol.routes[r]) - sol.lastFeasibleB[r] + 1
            dFeasBackward = - 2
            if removeLabelBackward.cost < Inf
                dFeasBackward += 1
            else
                dFeasBackward += 0
            end
        elseif pos == length(sol.routes[r]) - sol.lastFeasibleB[r]
            dFeasBackward = 0
            if removeLabelBackward.cost < Inf
                dFeasBackward += 1
            else
                dFeasBackward += 0
            end
        elseif pos == length(sol.routes[r]) - sol.lastFeasibleB[r] + 1
            dFeasBackward = - 1
            if removeLabelBackward.cost < Inf
                dFeasBackward += 1
            else
                dFeasBackward += 0
            end
        elseif pos < length(sol.routes[r]) - sol.lastFeasibleB[r] + 1
            dFeasBackward = 0
        end
        # @show dFeasBackward
        # @show removeLabelBackward
        backw = dFeasBackward
        if pos > length(sol.routes[r]) + 1 - sol.lastFeasibleB[r]
            backw += length(sol.routes[r]) - pos + 1# - 1
        end
        if pos == length(sol.routes[r]) + 1 - sol.lastFeasibleB[r]
            backw += length(sol.routes[r]) - pos
        end
        if pos < length(sol.routes[r]) + 1 - sol.lastFeasibleB[r]
            backw += sol.lastFeasibleB[r] - 1
        end
        # @show backw
    end
    return max(forw, backw)
end

function computeViolSwap11(solver::Solver, sol::Solution, r::Int, pos::Int, customer::Int)
    swapLabelForward = myExtendAlongArc(solver.res.customResource, sol.forwardLabels[r][pos-1], (sol.routes[r][pos-1]+1, customer + 1))
    swapLabelBackward = myExtendAlongArc(solver.res.customResource, sol.backwardLabels[r][length(sol.routes[r]) - pos], (sol.routes[r][pos+1]+1, customer + 1))
    concatCost = myConcatenationCost(solver.res.customResource, swapLabelBackward.last, swapLabelForward, swapLabelBackward)

    if concatCost.cost < Inf
        return length(sol.routes[r]) - 1
    end
    # FORWARD
    if sol.feasiblesF[r] == length(sol.routes[r]) - 1
        # swapLabelForward = myExtendAlongArc(solver.res.customResource, sol.forwardLabels[r][pos-1], (sol.routes[r][pos-1]+1, customer + 1))
        # concatCost = myConcatenationCost(solver.res.customResource, swapLabelBackward.last, swapLabelForward, swapLabelBackward)

        swapLabelForward2 = myExtendAlongArc(solver.res.customResource, swapLabelForward, (customer + 1, sol.routes[r][pos+1]+1))
        concatCost = myConcatenationCost(solver.res.customResource, sol.backwardLabels[r][length(sol.routes[r]) - pos].last, swapLabelForward2, sol.backwardLabels[r][length(sol.routes[r]) - pos])
        dFeasForward = 0
        if swapLabelForward.cost < Inf
            dFeasForward += 1
        end
        if swapLabelForward2.cost < Inf
            dFeasForward += 1
        end
        if 1==2#concatCost.cost < Inf
            forw = length(sol.routes[r]) - 1
        else
            forw = pos - 2 + dFeasForward
        end
        # @show swapLabelForward
        # @show swapLabelForward2
        # @show concatCost
        # @show r, pos, customer
        # @show dFeasForward
        # @show forw
        # println("-"^100)
        
        # if swapLabelForward.cost == Inf
        #     dFeasForward = - length(sol.routes[r]) + pos - 1
        # elseif swapLabelForward.cost < Inf && concatCost.cost < Inf
        #     dFeasForward = 0
        # elseif swapLabelForward.cost < Inf && concatCost.cost == Inf
        #     dFeasForward = - length(sol.routes[r]) + pos
        # end
        # forw = length(sol.routes[r]) - 1 + dFeasForward
        # @show swapLabelForward
        # @show swapLabelForward2
        # @show concatCost
        # @show forw
        # println("-"^100)
    else
        # swapLabelForward = myExtendAlongArc(solver.res.customResource, sol.forwardLabels[r][pos-1], (sol.routes[r][pos-1]+1, customer + 1))
        swapLabelForward2 = myExtendAlongArc(solver.res.customResource, swapLabelForward, (customer + 1, sol.routes[r][pos+1]+1))
        if pos < sol.lastFeasibleF[r]
            dFeasForward = -2
            swapLabelForward.cost < Inf ? dFeasForward += 1 : 0
            swapLabelForward2.cost < Inf ? dFeasForward += 1 : 0   
        end
        if pos == sol.lastFeasibleF[r]
            dFeasForward = -1
            swapLabelForward.cost < Inf ? dFeasForward += 1 : 0
            swapLabelForward2.cost < Inf ? dFeasForward += 1 : 0
        end
        if pos == sol.lastFeasibleF[r] + 1
            dFeasForward = 0
            swapLabelForward.cost < Inf ? dFeasForward += 1 : 0
            swapLabelForward2.cost < Inf ? dFeasForward += 1 : 0
        end
        if pos > sol.lastFeasibleF[r] + 1
            dFeasForward = 0
        end
        # @show swapLabelForward
        # @show swapLabelForward2
        # @show dFeasForward
        forw = dFeasForward
        if pos < sol.lastFeasibleF[r]
            forw += pos# - 1
        end
        if pos == sol.lastFeasibleF[r]
            forw += pos - 1
        end
        if pos == sol.lastFeasibleF[r] + 1
            forw += sol.lastFeasibleF[r] - 1
        end
        if pos > sol.lastFeasibleF[r] + 1
            forw += sol.lastFeasibleF[r] - 1
        end
        # @show forw
    end

    # BACKWARD
    if sol.feasiblesB[r] == length(sol.routes[r]) - 1
        swapLabelBackward2 = myExtendAlongArc(solver.res.customResource, swapLabelBackward, (customer+1, sol.routes[r][pos-1]+1))
        concatCost = myConcatenationCost(solver.res.customResource, swapLabelBackward2.last, sol.forwardLabels[r][pos-1], swapLabelBackward2)
        # @show sol.forwardLabels[r][pos-1]
        dFeasBackward = 0
        if swapLabelBackward.cost < Inf
            dFeasBackward += 1
        end
        if swapLabelBackward2.cost < Inf
            dFeasBackward += 1
        end
        if 1==2#concatCost.cost < Inf
            backw = length(sol.routes[r]) - 1
        else
            backw = length(sol.routes[r]) - pos - 1 + dFeasBackward# + 1
        end
        # if swapLabelBackward.cost == Inf
        #     dFeasBackward = - length(sol.routes[r]) + pos# + 1
        # elseif swapLabelBackward.cost < Inf && swapLabelBackward2.cost == Inf
        #     dFeasBackward = - length(sol.routes[r]) + pos - 1

        # elseif swapLabelBackward2.cost < Inf && concatCost.cost < Inf
        #     dFeasBackward = 0
        # elseif swapLabelBackward2.cost < Inf && concatCost.cost == Inf
        #     dFeasBackward = - length(sol.routes[r]) + pos# - 1
        # end

        # @show swapLabelBackward
        # @show swapLabelBackward2
        # @show concatCost
        # @show dFeasBackward
        # @show backw
        # backw = length(sol.routes[r]) - pos - 1 + dFeasBackward# + 1

        # backw = length(sol.routes[r]) - 1 + dFeasBackward# + 1
        # @show backw
    else
        # swapLabelBackward = myExtendAlongArc(solver.res.customResource, sol.backwardLabels[r][length(sol.routes[r]) - pos], (sol.routes[r][pos]+1, customer+1))
        swapLabelBackward2 = myExtendAlongArc(solver.res.customResource, swapLabelBackward, (customer + 1, sol.routes[r][pos-1]+1))

        # alterando dentro da zona viavel da rota
        if pos > length(sol.routes[r]) - sol.lastFeasibleB[r] + 1
            dFeasBackward = -2
            swapLabelBackward.cost < Inf ? dFeasBackward += 1 : 0
            swapLabelBackward2.cost < Inf ? dFeasBackward += 1 : 0   
        end
        # alterando o ultimo cliente viavel da rota
        if pos == length(sol.routes[r]) - sol.lastFeasibleB[r] + 1
            dFeasBackward = -1
            swapLabelBackward.cost < Inf ? dFeasBackward += 1 : 0
            swapLabelBackward2.cost < Inf ? dFeasBackward += 1 : 0
        end
        # alterando o primeiro cliente inviavel da rota
        if pos == length(sol.routes[r]) - sol.lastFeasibleB[r]
            dFeasBackward = 0
            swapLabelBackward.cost < Inf ? dFeasBackward += 1 : 0
            swapLabelBackward2.cost < Inf ? dFeasBackward += 1 : 0
        end
        # alterando regiao inviavel da rota
        if pos < length(sol.routes[r]) - sol.lastFeasibleB[r] + 1
            dFeasBackward = 0
        end
        # @show swapLabelBackward2
        # @show dFeasBackward
        backw = dFeasBackward
        if pos > length(sol.routes[r])  - sol.lastFeasibleB[r] + 1
            backw += length(sol.routes[r]) - pos# + 1
            if pos == length(sol.routes[r]) - 1
                backw += 1
            end
        end
        if pos == length(sol.routes[r]) - sol.lastFeasibleB[r] + 1
            backw += length(sol.routes[r]) - pos# + 1
        end
        if pos == length(sol.routes[r]) - sol.lastFeasibleB[r]
            backw += sol.lastFeasibleB[r] - 1
        end
        if pos < length(sol.routes[r]) - sol.lastFeasibleB[r]
            backw += sol.lastFeasibleB[r] - 1
        end
        # @show backw
    end
    return max(forw, backw)
end

function computeViolTwoOptStar(solver::Solver, sol::Solution, r1::Int, r2::Int, i::Int, j::Int)
    # FORWARD r1
    labelForward1 = myExtendAlongArc(solver.res.customResource, sol.forwardLabels[r1][i], (sol.routes[r1][i]+1, sol.routes[r2][j+1]+1))
    # concatForward1 = myConcatenationCost(solver.res.customResource, 1, labelForward1, solver.backwardLabels[r2][length(sol.routes[r2]) - j - 1])
    labelBackward = sol.backwardLabels[r2][length(sol.routes[r2]) - j]
    concat = myConcatenationCost(solver.res.customResource, labelBackward.last, labelForward1, labelBackward)
    if concat.cost < Inf
        feasR1 = i + length(sol.routes[r2]) - j - 1
    else
        forw1 = 0
        forw1 += labelForward1.cost < Inf ? 1 : 0

        if i < sol.lastFeasibleF[r1]
            forw1 += i - 1
        end
        if i == sol.lastFeasibleF[r1]
            forw1 += i - 1
        end
        if i > sol.lastFeasibleF[r1]
            forw1 += sol.lastFeasibleF[r1] - 1
        end
        # BACKWARD r1
        labelBackward1 = myExtendAlongArc(solver.res.customResource, sol.backwardLabels[r2][length(sol.routes[r2]) - j], (sol.routes[r2][j+1]+1, sol.routes[r1][i]+1))
        # concatBackward1 = myConcatenationCost(solver.res.customResource, 1, labelBackward1, sol.forwardLabels[r1][i-1])
        backw1 = 0
        backw1 += labelBackward1.cost < Inf ? 1 : 0
        if j > length(sol.routes[r2]) + 1 - sol.lastFeasibleB[r2]
            backw1 += length(sol.routes[r2]) - j - 1
        end
        if j == length(sol.routes[r2]) + 1 - sol.lastFeasibleB[r2]
            backw1 += length(sol.routes[r2]) - j - 1
        end
        if j < length(sol.routes[r2]) + 1 - sol.lastFeasibleB[r2]
            backw1 += sol.lastFeasibleB[r2] - 1
        end
        feasR1 = max(backw1, forw1)
    end
    # FORWARD r2
    labelForward2 = myExtendAlongArc(solver.res.customResource, sol.forwardLabels[r2][j], (sol.routes[r2][j]+1, sol.routes[r1][i+1]+1))
    concat = myConcatenationCost(solver.res.customResource, sol.backwardLabels[r1][length(sol.routes[r1]) - i].last, labelForward2, sol.backwardLabels[r1][length(sol.routes[r1]) - i])
    if concat.cost < Inf
        feasR2 = j + length(sol.routes[r1]) - i - 1
    else
        forw2 = 0
        forw2 += labelForward2.cost < Inf ? 1 : 0
        if j < sol.lastFeasibleF[r2]
            forw2 += j - 1
        end
        if j == sol.lastFeasibleF[r2]
            forw2 += j - 1
        end
        if j > sol.lastFeasibleF[r2]
            forw2 += sol.lastFeasibleF[r2] - 1
        end
        labelBackward2 = myExtendAlongArc(solver.res.customResource, sol.backwardLabels[r1][length(sol.routes[r1]) - i], (sol.routes[r1][i+1]+1, sol.routes[r2][j]+1))
        # concatBackward2 = myConcatenationCost(solver.res.customResource, 1, labelBackward2, sol.forwardLabels[r2][j-1])
        backw2 = 0
        backw2 += labelBackward2.cost < Inf ? 1 : 0
        if i > length(sol.routes[r1]) + 1 - sol.lastFeasibleB[r1]
            backw2 += length(sol.routes[r1]) - i - 1
        end
        if i == length(sol.routes[r1]) + 1 - sol.lastFeasibleB[r1]
            backw2 += length(sol.routes[r1]) - i - 1
        end
        if i < length(sol.routes[r1]) + 1 - sol.lastFeasibleB[r1]
            backw2 += sol.lastFeasibleB[r1] - 1
        end
        feasR2 = max(forw2, backw2)
    end
    return feasR1, feasR2#max(forw1, backw1) + max(forw2, backw2)
end

function computeViolIntraShift10(solver::Solver, sol::Solution, r::Int, i::Int, j::Int)
    if i < j
        if j == i+1
            solver.prevLabelF = myExtendAlongArc(solver.res.customResource, sol.forwardLabels[r][i-1], (sol.routes[r][i-1]+1, sol.routes[r][i+1] + 1))
            auxLabel = myExtendAlongArc(solver.res.customResource, solver.prevLabelF, (solver.prevLabelF.last+1, sol.routes[r][i] + 1))
            auxLabel = myExtendAlongArc(solver.res.customResource, auxLabel, (auxLabel.last + 1, sol.routes[r][j+1] + 1))
            res = myConcatenationCost(solver.res.customResource, sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1].last, auxLabel, sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1])
        else
            solver.prevLabelF = myExtendAlongArc(solver.res.customResource, solver.prevLabelF, (solver.prevLabelF.last + 1, sol.routes[r][j] + 1))
            auxLabel = myExtendAlongArc(solver.res.customResource, solver.prevLabelF, (solver.prevLabelF.last+1, sol.routes[r][i] + 1))
            auxLabel = myExtendAlongArc(solver.res.customResource, auxLabel, (auxLabel.last + 1, sol.routes[r][j+1] + 1))
            res = myConcatenationCost(solver.res.customResource, sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1].last, auxLabel, sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1])
            # @show res
        end
    else
        if j == i-1
            solver.prevLabelB = myExtendAlongArc(solver.res.customResource, sol.backwardLabels[r][length(sol.routes[r]) - i], (sol.routes[r][i+1]+1, sol.routes[r][i-1] + 1))
            auxLabel = myExtendAlongArc(solver.res.customResource, solver.prevLabelB, (sol.routes[r][i-1]+1, sol.routes[r][i] + 1))
            auxLabel = myExtendAlongArc(solver.res.customResource, auxLabel, (sol.routes[r][j]+1, sol.routes[r][j-1] + 1))
            res = myConcatenationCost(solver.res.customResource, auxLabel.last, sol.forwardLabels[r][j - 1], auxLabel)
        else
            solver.prevLabelB = myExtendAlongArc(solver.res.customResource, solver.prevLabelB, (solver.prevLabelB.last+1, sol.routes[r][j] + 1))
            auxLabel = myExtendAlongArc(solver.res.customResource, solver.prevLabelB, (sol.routes[r][j]+1, sol.routes[r][i] + 1))
            auxLabel = myExtendAlongArc(solver.res.customResource, auxLabel, (sol.routes[r][j]+1, sol.routes[r][j-1] + 1))
            res = myConcatenationCost(solver.res.customResource, auxLabel.last, sol.forwardLabels[r][j - 1], auxLabel)
        end
    end
    # if i < j
    #     return 1
    # end
    if res.cost >= Inf
        return 1
    else
        return 0
    end
end

function computeViolInterShift10(solver::Solver, sol::Solution, r1::Int, r2::Int, i::Int, j::Int)
    remove1Feas = computeViolRemove1(solver, sol, r1, i)
    insertion1Feas = computeViolInsertion1(solver, sol, r2, sol.routes[r1][i], j)
    return remove1Feas, insertion1Feas
end

function computeViolInterSwap11(solver::Solver, sol::Solution, r1::Int, r2::Int, i::Int, j::Int)
    feasR1 = computeViolSwap11(solver, sol, r1, i, sol.routes[r2][j])
    feasR2 = computeViolSwap11(solver, sol, r2, j, sol.routes[r1][i])

    return feasR1, feasR2
end

function computeStdViolIntraShift10(solver::Solver, sol::Solution, r::Int, i::Int, j::Int)
    if i < j
        if j == i+1
            # return 100.0
            # @show solver.prevLabelStdF, (sol.routes[r][i-1]+1, sol.routes[r][i+1] + 1)
            # solver.prevLabelStdF = ForwardLabel(solver.prevLabelF.custom_res, solver.prevLabelF.cost, extendAlongArc(solver.res.stdResource, sol.forwardLabels[r][i-1], (sol.routes[r][i-1]+1, sol.routes[r][i+1] + 1)), vcat(sol.forwardLabels[r][i-1].path, sol.routes[r][i+1]), sol.routes[r][i+1])
            # @show solver.prevLabelStdF
            # auxLabel = ForwardLabel(solver.prevLabelF.custom_res, solver.prevLabelF.cost, extendAlongArc(solver.res.stdResource, solver.prevLabelStdF, (sol.routes[r][j]+1, sol.routes[r][i] + 1)), vcat(solver.prevLabelStdF.path, sol.routes[r][i]), sol.routes[r][i])
            # @show auxLabel
            # auxLabel = ForwardLabel(auxLabel.custom_res, auxLabel.cost, extendAlongArc(solver.res.stdResource, auxLabel, (auxLabel.last+1, sol.routes[r][j+1]+1)), vcat(auxLabel.path, sol.routes[r][j+1]), sol.routes[r][j+1])
            # @show auxLabel
            solver.prevLabelStdF = ForwardLabel(solver.prevLabelF.custom_res, solver.prevLabelF.cost, extendAlongArc(solver.res.stdResource, sol.forwardLabels[r][i-1], (sol.routes[r][i-1] + 1, sol.routes[r][i+1] + 1)), sol.routes[r][i+1])
            auxLabel = ForwardLabel(solver.prevLabelF.custom_res, solver.prevLabelF.cost, extendAlongArc(solver.res.stdResource, solver.prevLabelStdF, (sol.routes[r][j] + 1, sol.routes[r][i] + 1)), sol.routes[r][i])
            auxLabel = ForwardLabel(auxLabel.custom_res, auxLabel.cost, extendAlongArc(solver.res.stdResource, auxLabel, (auxLabel.last + 1, sol.routes[r][j+1] + 1)), sol.routes[r][j+1])
            # @show sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1]

            res = concatenationCost(solver.res.stdResource, sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1].last, auxLabel, sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1])
            # println("-"^100)
            return res.stdWarp
        else
            # println("i<j")
            # return 100.0
            # @show solver.prevLabelStdF, (solver.prevLabelStdF.last+1, sol.routes[r][j] + 1)
            # solver.prevLabelStdF = ForwardLabel(solver.prevLabelF.custom_res, solver.prevLabelF.cost, extendAlongArc(solver.res.stdResource, solver.prevLabelStdF, (solver.prevLabelStdF.last+1, sol.routes[r][j] + 1)), vcat(solver.prevLabelStdF.path, sol.routes[r][j]), sol.routes[r][j])
            # @show solver.prevLabelStdF
            # auxLabel = ForwardLabel(solver.prevLabelF.custom_res, solver.prevLabelF.cost, extendAlongArc(solver.res.stdResource, solver.prevLabelStdF, (sol.routes[r][j]+1, sol.routes[r][i] + 1)), vcat(solver.prevLabelStdF.path, sol.routes[r][i]), sol.routes[r][i])
            # @show auxLabel
            # auxLabel = ForwardLabel(auxLabel.custom_res, auxLabel.cost, extendAlongArc(solver.res.stdResource, auxLabel, (auxLabel.last+1, sol.routes[r][j+1]+1)), vcat(auxLabel.path, sol.routes[r][j+1]), sol.routes[r][j+1])
            # @show auxLabel
            solver.prevLabelStdF = ForwardLabel(solver.prevLabelF.custom_res, solver.prevLabelF.cost, extendAlongArc(solver.res.stdResource, solver.prevLabelStdF, (sol.routes[r][i-1]+1, sol.routes[r][j] + 1)), sol.routes[r][j])
            auxLabel = ForwardLabel(solver.prevLabelF.custom_res, solver.prevLabelF.cost, extendAlongArc(solver.res.stdResource, solver.prevLabelStdF, (sol.routes[r][j]+1, sol.routes[r][i] + 1)),  sol.routes[r][i])
            auxLabel = ForwardLabel(auxLabel.custom_res, auxLabel.cost, extendAlongArc(solver.res.stdResource, auxLabel, (auxLabel.last+1, sol.routes[r][j+1]+1)), sol.routes[r][j+1])
            # @show sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1]
            res = concatenationCost(solver.res.stdResource, sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1].last, auxLabel, sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1])
            # println("-"^100)

           return res.stdWarp
        end
    else
        if j == i-1
            # return 100.0

            # solver.prevLabelStdB = BackwardLabel(solver.prevLabelStdB.custom_res, solver.prevLabelStdB.cost, extendAlongArc(solver.res.stdResource, sol.backwardLabels[r][length(sol.routes[r]) - i], (sol.routes[r][i+1]+1, sol.routes[r][i-1] + 1)), vcat(sol.routes[r][i-1], sol.backwardLabels[r][length(sol.routes[r]) - i].path), sol.routes[r][i-1])
            # @show solver.prevLabelStdB
            # auxLabel = BackwardLabel(solver.prevLabelB.custom_res, solver.prevLabelB.cost, extendAlongArc(solver.res.stdResource, solver.prevLabelStdB, (solver.prevLabelStdB.last+1, sol.routes[r][i] + 1)), vcat(sol.routes[r][i], solver.prevLabelStdB.path), sol.routes[r][i])
            # @show auxLabel
            # auxLabel = BackwardLabel(auxLabel.custom_res, auxLabel.cost, extendAlongArc(solver.res.stdResource, auxLabel, (sol.routes[r][j]+1, sol.routes[r][j-1]+1)), vcat(sol.routes[r][j-1], auxLabel.path), sol.routes[r][j-1])
            # @show auxLabel

            solver.prevLabelStdB = BackwardLabel(solver.prevLabelStdB.custom_res, solver.prevLabelStdB.cost, extendAlongArc(solver.res.stdResource, sol.backwardLabels[r][length(sol.routes[r]) - i], (sol.routes[r][i+1]+1, sol.routes[r][i-1] + 1)), sol.routes[r][i-1])
            auxLabel = BackwardLabel(solver.prevLabelB.custom_res, solver.prevLabelB.cost, extendAlongArc(solver.res.stdResource, solver.prevLabelStdB, (solver.prevLabelStdB.last+1, sol.routes[r][i] + 1)), sol.routes[r][i])
            auxLabel = BackwardLabel(auxLabel.custom_res, auxLabel.cost, extendAlongArc(solver.res.stdResource, auxLabel, (sol.routes[r][j]+1, sol.routes[r][j-1]+1)), sol.routes[r][j-1])
            
            # @show sol.forwardLabels[r][j - 1]
            res = concatenationCost(solver.res.stdResource, auxLabel.last, sol.forwardLabels[r][j - 1], auxLabel)

            return res.stdWarp
        else
            # return 100.0
            # solver.prevLabelStdB = BackwardLabel(solver.prevLabelStdB.custom_res, solver.prevLabelStdB.cost, extendAlongArc(solver.res.stdResource, solver.prevLabelStdB, (solver.prevLabelStdB.last+1, sol.routes[r][j] + 1)), vcat(sol.routes[r][j], solver.prevLabelStdB.path), sol.routes[r][j])
            # @show solver.prevLabelStdB
            # auxLabel = BackwardLabel(solver.prevLabelB.custom_res, solver.prevLabelB.cost, extendAlongArc(solver.res.stdResource, solver.prevLabelStdB, (solver.prevLabelStdB.last+1, sol.routes[r][i] + 1)), vcat(sol.routes[r][i], solver.prevLabelStdB.path), sol.routes[r][i])
            # @show auxLabel
            # auxLabel = BackwardLabel(auxLabel.custom_res, auxLabel.cost, extendAlongArc(solver.res.stdResource, auxLabel, (sol.routes[r][j]+1, sol.routes[r][j-1]+1)), vcat(sol.routes[r][j-1], auxLabel.path), sol.routes[r][j-1])
            # @show auxLabel
            solver.prevLabelStdB = BackwardLabel(solver.prevLabelStdB.custom_res, solver.prevLabelStdB.cost, extendAlongArc(solver.res.stdResource, solver.prevLabelStdB, (solver.prevLabelStdB.last+1, sol.routes[r][j] + 1)),  sol.routes[r][j])
            auxLabel = BackwardLabel(solver.prevLabelB.custom_res, solver.prevLabelB.cost, extendAlongArc(solver.res.stdResource, solver.prevLabelStdB, (solver.prevLabelStdB.last+1, sol.routes[r][i] + 1)), sol.routes[r][i])
            auxLabel = BackwardLabel(auxLabel.custom_res, auxLabel.cost, extendAlongArc(solver.res.stdResource, auxLabel, (sol.routes[r][j]+1, sol.routes[r][j-1]+1)), sol.routes[r][j-1])
            # @show sol.forwardLabels[r][j - 1]
            res = concatenationCost(solver.res.stdResource, auxLabel.last, sol.forwardLabels[r][j - 1], auxLabel)
            return res.stdWarp
        end
    end
end

function computeStdViolInsertion1(solver::Solver, sol::Solution, r::Int, customer::Int, pos::Int)
    forwardLabel = sol.forwardLabels[r][pos-1]
    backwardLabel = sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos]

    stdResForward = extendAlongArc(solver.res.stdResource, forwardLabel, (sol.routes[r][pos-1]+1, customer+1))
    # insertionLabelForward = ForwardLabel(forwardLabel.custom_res, forwardLabel.cost, stdResForward, vcat(forwardLabel.path, customer), customer)
    insertionLabelForward = ForwardLabel(forwardLabel.custom_res, forwardLabel.cost, stdResForward, customer)

    stdResBackward = extendAlongArc(solver.res.stdResource, backwardLabel, (sol.routes[r][pos]+1, customer+1))
    # insertionLabelBackward = BackwardLabel(backwardLabel.custom_res, backwardLabel.cost, stdResBackward, vcat(customer, backwardLabel.path), customer)
    insertionLabelBackward = BackwardLabel(backwardLabel.custom_res, backwardLabel.cost, stdResBackward, customer)

    concatCost = concatenationCost(solver.res.stdResource, insertionLabelBackward.last, insertionLabelForward, insertionLabelBackward)
    return concatCost.stdWarp
end

function computeStdViolRemove1(solver::Solver, sol::Solution, r::Int, pos::Int)
    forwardLabel = sol.forwardLabels[r][pos-1]
    backwardLabel = sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos - 1]
    stdResForward = extendAlongArc(solver.res.stdResource, forwardLabel, (sol.routes[r][pos-1]+1, sol.routes[r][pos+1]+1))
    # removalLabelForward = ForwardLabel(forwardLabel.custom_res, forwardLabel.cost, stdResForward, vcat(forwardLabel.path, sol.routes[r][pos+1]), sol.routes[r][pos+1])
    removalLabelForward = ForwardLabel(forwardLabel.custom_res, forwardLabel.cost, stdResForward, sol.routes[r][pos+1])

    concatCost = concatenationCost(solver.res.stdResource, backwardLabel.last, removalLabelForward, backwardLabel)
    return concatCost.stdWarp
end

function computeStdViolSwap11(solver::Solver, sol::Solution, r::Int, pos::Int, customer::Int)
    forwardLabel = sol.forwardLabels[r][pos-1]
    backwardLabel = sol.backwardLabels[r][length(sol.routes[r]) - pos]
    stdResForward = extendAlongArc(solver.res.stdResource, forwardLabel, (sol.routes[r][pos-1]+1, customer + 1))
    # swapLabelForward = ForwardLabel(forwardLabel.custom_res, forwardLabel.cost, stdResForward, vcat(forwardLabel.path, customer), customer)
    swapLabelForward = ForwardLabel(forwardLabel.custom_res, forwardLabel.cost, stdResForward, customer)

    stdResBackward = extendAlongArc(solver.res.stdResource, backwardLabel, (sol.routes[r][pos+1]+1, customer+1))
    # swapLabelBackward = BackwardLabel(backwardLabel.custom_res, backwardLabel.cost, stdResBackward, vcat(customer, backwardLabel.path), customer)
    swapLabelBackward = BackwardLabel(backwardLabel.custom_res, backwardLabel.cost, stdResBackward, customer)

    concatCost = concatenationCost(solver.res.stdResource, swapLabelBackward.last, swapLabelForward, swapLabelBackward)
    return concatCost.stdWarp
end

function computeStdViolTwoOptStar(solver::Solver, sol::Solution, r1::Int, r2::Int, i::Int, j::Int)
    # route r1
    forwardLabel1 = sol.forwardLabels[r1][i]
    backwardLabel1 = sol.backwardLabels[r2][length(sol.routes[r2]) - j]
    stdResForward = extendAlongArc(solver.res.stdResource, forwardLabel1, (sol.routes[r1][i]+1, sol.routes[r2][j+1] + 1))
    # forwardLabel1 = ForwardLabel(forwardLabel1.custom_res, forwardLabel1.cost, stdResForward, vcat(forwardLabel1.path, sol.routes[r2][j+1]), sol.routes[r2][j+1])
    forwardLabel1 = ForwardLabel(forwardLabel1.custom_res, forwardLabel1.cost, stdResForward, sol.routes[r2][j+1])

    concat1 = concatenationCost(solver.res.stdResource, backwardLabel1.last, forwardLabel1, backwardLabel1)

    # route r2
    forwardLabel2 = sol.forwardLabels[r2][j]
    backwardLabel2 = sol.backwardLabels[r1][length(sol.routes[r1]) - i]
    stdResForward = extendAlongArc(solver.res.stdResource, forwardLabel2, (sol.routes[r2][j]+1, sol.routes[r1][i+1] + 1))
    # forwardLabel2 = ForwardLabel(forwardLabel2.custom_res, forwardLabel2.cost, stdResForward, vcat(forwardLabel2.path, sol.routes[r1][i+1]), sol.routes[r1][i+1])
    forwardLabel2 = ForwardLabel(forwardLabel2.custom_res, forwardLabel2.cost, stdResForward,  sol.routes[r1][i+1])

    concat2 = concatenationCost(solver.res.stdResource, backwardLabel2.last, forwardLabel2, backwardLabel2)
    return concat1.stdWarp, concat2.stdWarp
end

# TO DO
#=
    function computeViolTwoOptStar2(solver::Solver, r1::Int, r2::Int, i::Int, j::Int)
        # FORWARD r1
        labelForward1 = myExtendAlongArc(solver.res, solver.forwardLabels[r1][i], (solver.currSol.routes[r1][i]+1, solver.currSol.routes[r2][j+1]+1))
        # concatForward1 = myConcatenationCost(solver.res, 1, labelForward1, solver.backwardLabels[r2][length(solver.currSol.routes[r2]) - j - 1])
        if i < solver.currSol.lastFeasibleF[r1]
            forw1 = -1
            forw1 += labelForward1.cost < Inf ? 1 : 0
        elseif i == solver.currSol.lastFeasibleF[r1]
            forw1 = 0
            forw1 += labelForward1.cost < Inf ? 1 : 0
        else
            forw1 = -typemax(Int)
        end
        # forw1 += labelForward1.cost < Inf ? 1 : 0

        if i <= solver.currSol.lastFeasibleF[r1]
            forw1 += i - 1
        else
            forw1 += solver.currSol.lastFeasibleF[r1] - 1
        end
        # BACKWARD r1
        labelBackward1 = myExtendAlongArc(solver.res, solver.backwardLabels[r2][length(solver.currSol.routes[r2]) - j], (solver.currSol.routes[r2][j+1]+1, solver.currSol.routes[r1][i]+1))
        # concatBackward1 = myConcatenationCost(solver.res, 1, labelBackward1, solver.forwardLabels[r1][i-1])
        if j >= length(solver.currSol.routes[r2]) - solver.currSol.lastFeasibleB[r2] + 1
            backw1 = -1
            backw1 += labelBackward1.cost < Inf ? 1 : 0
        elseif j == length(solver.currSol.routes[r2]) - solver.currSol.lastFeasibleB[r2] + 1
            backw1 = 0
            backw1 += labelBackward1.cost < Inf ? 1 : 0
        else
            backw1 = -typemax(Int)
        end
        # @show j, length(solver.currSol.routes[r2]) + 1 - solver.currSol.lastFeasibleB[r2]
        if j >= length(solver.currSol.routes[r2]) + 1 - solver.currSol.lastFeasibleB[r2]
            backw1 += length(solver.currSol.routes[r2]) - j - 1
        else
            backw1 += solver.currSol.lastFeasibleB[r2] - 1
        end

        # FORWARD r2
        labelForward2 = myExtendAlongArc(solver.res, solver.forwardLabels[r2][j], (solver.currSol.routes[r2][j]+1, solver.currSol.routes[r1][i+1]+1))
        # concatForward2 = myConcatenationCost(solver.res, 1, labelForward2, solver.backwardLabels[r1][length(solver.currSol.routes[r1]) - i - 1])
        # forw2 = 0
        if j < solver.currSol.lastFeasibleF[r2]
            forw2 = -1
            forw2 += labelForward2.cost < Inf ? 1 : 0
        elseif j == solver.currSol.lastFeasibleF[r2]
            forw2 = 0
            forw2 += labelForward2.cost < Inf ? 1 : 0
        else
            forw2 = -typemax(Int)
        end
        # if j == solver.currSol.lastFeasibleF[r2]
        #     forw2 += 1
        # end
        # forw2 += labelForward1.cost < Inf ? 1 : 0
        if j <= solver.currSol.lastFeasibleF[r2]
            forw2 += j - 1
        else
            forw2 += solver.currSol.lastFeasibleF[r2] - 1
        end
        labelBackward2 = myExtendAlongArc(solver.res, solver.backwardLabels[r1][length(solver.currSol.routes[r1]) - i], (solver.currSol.routes[r1][i+1]+1, solver.currSol.routes[r2][j]+1))
        # concatBackward2 = myConcatenationCost(solver.res, 1, labelBackward2, solver.forwardLabels[r2][j-1])
        backw2 = labelBackward2.cost < Inf ? 1 : 0
        if i >= length(solver.currSol.routes[r1]) - solver.currSol.lastFeasibleB[r1] + 1
            backw2 = -1
            backw2 += labelBackward2.cost < Inf ? 1 : 0
        elseif i == length(solver.currSol.routes[r1]) - solver.currSol.lastFeasibleB[r1] + 1
            backw2 = 0
            backw2 += labelBackward2.cost < Inf ? 1 : 0
        else
            backw2 = -typemax(Int)
        end
        # @show i, length(solver.currSol.routes[r1]) + 1 - solver.currSol.lastFeasibleB[r1]
        if i >= length(solver.currSol.routes[r1]) + 1 - solver.currSol.lastFeasibleB[r1]
            backw2 += length(solver.currSol.routes[r1]) - i - 1
        else
            backw2 += solver.currSol.lastFeasibleB[r1] - 1
        end
        @show labelForward1
        @show labelBackward1
        @show labelForward2
        @show labelBackward2
        @show forw1, backw1
        @show forw2, backw2

        return max(forw1, backw1) + max(forw2, backw2)
    end

    function computeViolInsertion2(solver::Solver, r::Int, i::Int, j::Int)
        # FORWARD
        if solver.currSol.feasiblesF[r] == length(solver.currSol.routes[r]) - 2
            prev = myExtendAlongArc(solver.res, solver.forwardLabels[r][j-1], (solver.currSol.routes[r][j-1]+1, i + 1))
            concatCost = myConcatenationCost(solver.res, i, prev, solver.backwardLabels[r][length(solver.currSol.routes[r]) + 1 - j])
            if prev.cost == Inf
                dFeasForward = - length(solver.currSol.routes[r]) - 2
            elseif prev.cost < Inf && concatCost.cost < Inf
                dFeasForward = 2
            elseif prev.cost < Inf && concatCost.cost == Inf
                dFeasForward = - length(solver.currSol.routes[r]) - 1
            end
        else
            prev = myExtendAlongArc(solver.res, solver.forwardLabels[r][j-1], (solver.currSol.routes[r][j-1]+1, i + 1))
            prev2 = myExtendAlongArc(solver.res, prev, (i + 1, solver.currSol.routes[r][j]+1))
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
            prev = myExtendAlongArc(solver.res, solver.forwardLabels[r][j-1], (solver.currSol.routes[r][j-1]+1, i + 1))
            concatCost = myConcatenationCost(solver.res, i, prev, solver.backwardLabels[r][length(solver.currSol.routes[r]) + 1 - j])
            if prev.cost == Inf
                dFeasBackward = - length(solver.currSol.routes[r]) - 2
            elseif prev.cost < Inf && concatCost.cost < Inf
                dFeasBackward = 2
            elseif prev.cost < Inf && concatCost.cost == Inf
                dFeasBackward = - length(solver.currSol.routes[r]) - 1
            end
        else
            prev = myExtendAlongArc(solver.res, solver.backwardLabels[r][length(solver.currSol.routes[r]) - j + 1], (solver.currSol.routes[r][j]+1, i+1))
            prev2 = myExtendAlongArc(solver.res, prev, (i + 1, solver.currSol.routes[r][j-1]+1))
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

    function computeViolRemove2(solver::Solver, r::Int, i::Int)
        # FORWARD
        if solver.currSol.feasiblesF[r] == length(solver.currSol.routes[r]) - 2
            removLabel = myConcatenationCost(solver.res, i, solver.forwardLabels[r][i-1], solver.backwardLabels[r][length(solver.currSol.routes[r]) + 1 - i - 1])
            if removLabel.cost < Inf
                dFeasForward = -2
            else
                dFeasForward = -length(solver.currSol.routes[r])-1
            end
            # println("FEASIBLE ROUTE: $(removLabel)")
        else
            if i < solver.currSol.lastFeasibleF[r] + 1
                dFeasForward = - 2
                removLabel = myExtendAlongArc(solver.res, solver.forwardLabels[r][i-1]), (solver.currSol.routes[r][i-1]+1, solver.currSol.routes[r][i+1]+1))
                if removLabel.cost < Inf
                    dFeasForward += 1
                else
                    dFeasForward += 0
                end
                # println("i < : $(removLabel)")

            elseif i == solver.currSol.lastFeasibleF[r] + 1
                removLabel = myExtendAlongArc(solver.res, solver.forwardLabels[r][i-1]), (solver.currSol.routes[r][i-1]+1, solver.currSol.routes[r][i+1]+1))
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
            removLabel = myConcatenationCost(solver.res, i, solver.forwardLabels[r][i-1], solver.backwardLabels[r][length(solver.currSol.routes[r]) + 1 - i - 1])
            if removLabel.cost < Inf
                dFeasBackward = -2
            else
                dFeasBackward = -length(solver.currSol.routes[r])-1
            end
            # println("FEASIBLE ROUTE: $(removLabel)")
        else
            removLabel = myExtendAlongArc(solver.res, solver.backwardLabels[r][i-1]), (solver.currSol.routes[r][i+1]+1, solver.currSol.routes[r][i-1]+1))
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


# function computeViolIntraShift20(solver::Solver, r::Int)
#     return solver.currSol.resViolation[r]
# end

# function computeViolIntraSwap11()
#     return 0 
# end

# function computeViol2opt()
#     return 0 
# end

# function computeViolInterShift20(solver::Solver, r1::Int, r2::Int, i::Int, j::Int)
#     concatCost1 = myConcatenationCost(solver.res, i, solver.forwardLabels[r1][i-1], solver.backwardLabels[r1][length(solver.currSol.routes[r1]) + 1 - i - 2])
#     viol1 = 0
#     if concatCost1.cost >= Inf
#         viol1 += 1
#     end
#     prev1 = myExtendAlongArc(solver.res, solver.forwardLabels[r2][j-1], (solver.currSol.routes[r2][j-1] + 1, solver.currSol.routes[r1][i] + 1))
#     prev1 = myExtendAlongArc(solver.res, prev1, (solver.currSol.routes[r1][i] + 1, solver.currSol.routes[r1][i+1] + 1))
#     concatCost2 = myConcatenationCost(solver.res, i, prev1, solver.backwardLabels[r2][length(solver.currSol.routes[r2]) + 1 - j])
#     viol2 = 0
#     if concatCost2.cost >= Inf
#         viol2 += 1
#     end

#     return viol1, viol2
# end



