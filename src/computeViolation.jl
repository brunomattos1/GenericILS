function computeViolInsertion1(solver::Solver, sol::Solution, r::Int, customer::Int, pos::Int)

    # Primeiro eu tento uma concatenação, em que estendo $sol.routes[r][1:pos-1] -> $customer gerando um label forward
    # e depois estendo $customer <- $sol.routes[r][pos, end] gerando um label backward
    insertionLabelForward = myExtendAlongArc(solver.res.customResource, sol.forwardLabels[r][pos-1], (sol.routes[r][pos-1]+1, customer + 1))
    insertionLabelBackward = myExtendAlongArc(solver.res.customResource, sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos], (sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos].last +1, customer + 1))
    concat = myConcatenationCost(solver.res.customResource, insertionLabelBackward.last, insertionLabelForward, insertionLabelBackward)

    if concat.cost < Inf
        return length(sol.routes[r]) - 1 + 1, concat.cost
    end

    # FORWARD
    if sol.feasiblesF[r] == length(sol.routes[r]) - 1
        # label de $r[pos:end]
        backwardLabel = sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos]
        # estendo o $insertionLabel (que ja possui o $customer) para o proximo cliente da rota (backwardLabel.last)
        insertionLabelForward2 = myExtendAlongArc(solver.res.customResource, insertionLabelForward, (insertionLabelForward.last + 1, backwardLabel.last + 1))
        # concatCost = myConcatenationCost(solver.res.customResource, backwardLabel.last, insertionLabelForward2, backwardLabel)

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
    return max(forw, backw), min(insertionLabelForward2.cost, insertionLabelBackward2.cost)
end

function computeViolRemove1(solver::Solver, sol::Solution, r::Int, pos::Int)
    removeLabelForward = myExtendAlongArc(solver.res.customResource, sol.forwardLabels[r][pos-1], (sol.forwardLabels[r][pos-1].last+1, sol.routes[r][pos+1]+1))
    concat = myConcatenationCost(solver.res.customResource, sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos - 1].last, removeLabelForward, sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos - 1])
    if concat.cost < Inf
        return length(sol.routes[r]) - 1 - 1, concat.cost
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
    return max(forw, backw), min(removeLabelForward.cost, removeLabelBackward.cost)
end

function computeViolSwap11(solver::Solver, sol::Solution, r::Int, pos::Int, customer::Int)
    swapLabelForward = myExtendAlongArc(solver.res.customResource, sol.forwardLabels[r][pos-1], (sol.routes[r][pos-1]+1, customer + 1))
    swapLabelBackward = myExtendAlongArc(solver.res.customResource, sol.backwardLabels[r][length(sol.routes[r]) - pos], (sol.routes[r][pos+1]+1, customer + 1))
    concat = myConcatenationCost(solver.res.customResource, swapLabelBackward.last, swapLabelForward, swapLabelBackward)

    if concat.cost < Inf
        return length(sol.routes[r]) - 1, concat.cost
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
    return max(forw, backw), min(swapLabelForward2.cost, swapLabelBackward2.cost)
end

function computeViolTwoOptStar(solver::Solver, sol::Solution, r1::Int, r2::Int, i::Int, j::Int)
    # FORWARD r1
    labelForward1 = myExtendAlongArc(solver.res.customResource, sol.forwardLabels[r1][i], (sol.routes[r1][i]+1, sol.routes[r2][j+1]+1))
    # concatForward1 = myConcatenationCost(solver.res.customResource, 1, labelForward1, solver.backwardLabels[r2][length(sol.routes[r2]) - j - 1])
    labelBackward = sol.backwardLabels[r2][length(sol.routes[r2]) - j]
    concat1 = myConcatenationCost(solver.res.customResource, labelBackward.last, labelForward1, labelBackward)
    if concat1.cost < Inf
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
    concat2 = myConcatenationCost(solver.res.customResource, sol.backwardLabels[r1][length(sol.routes[r1]) - i].last, labelForward2, sol.backwardLabels[r1][length(sol.routes[r1]) - i])
    if concat2.cost < Inf
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
    infeasR1 = i + length(sol.routes[r2]) - j - feasR1 - 1
    infeasR2 = j + length(sol.routes[r1]) - i - feasR2 - 1
    if concat1.cost < Inf
        labelCostR1 = concat1.cost
    else
        labelCostR1 = min(labelForward1.cost, labelBackward1.cost)
    end
    if concat2.cost < Inf
        labelCostR2 = concat2.cost
    else
        labelCostR2 = min(labelForward2.cost, labelBackward2.cost)
    end
    return ViolationInfo(infeasR1, infeasR2, labelCostR1, labelCostR2)
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
    if res.cost >= Inf
        return 1, res.cost
    else
        return 0, res.cost
    end
end

function computeViolInterShift10(solver::Solver, sol::Solution, r1::Int, r2::Int, i::Int, j::Int)
    remove1Feas, remove1LabelCost = computeViolRemove1(solver, sol, r1, i)
    insertion1Feas, insertion1LabelCost = computeViolInsertion1(solver, sol, r2, sol.routes[r1][i], j)
    infeasR1 = length(sol.routes[r1]) - 1 - remove1Feas - 1
    infeasR2 = length(sol.routes[r2]) - 1 - insertion1Feas + 1
    violInfo = ViolationInfo(infeasR1, infeasR2, remove1LabelCost, insertion1LabelCost)
    return violInfo
end

function computeViolInterSwap11(solver::Solver, sol::Solution, r1::Int, r2::Int, i::Int, j::Int)
    feasR1, labelCostR1 = computeViolSwap11(solver, sol, r1, i, sol.routes[r2][j])
    feasR2, labelCostR2 = computeViolSwap11(solver, sol, r2, j, sol.routes[r1][i])
    infeasR1 = length(sol.routes[r1]) - 1 - feasR1
    infeasR2 = length(sol.routes[r2]) - 1 - feasR2
    violInfo = ViolationInfo(infeasR1, infeasR2, labelCostR1, labelCostR2)
    return violInfo
end

function computeStdViolIntraShift10(solver::Solver, sol::Solution, r::Int, i::Int, j::Int)
    if i < j

        if j == i+1

            solver.prevLabelStdF = ForwardLabel(
                solver.prevLabelF.state,
                solver.prevLabelF.cost,
                extendAlongArc(solver.res.stdResource1,
                            sol.forwardLabels[r][i-1],
                            (sol.routes[r][i-1] + 1, sol.routes[r][i+1] + 1)),
                extendAlongArc(solver.res.stdResource2,
                            sol.forwardLabels[r][i-1],
                            (sol.routes[r][i-1] + 1, sol.routes[r][i+1] + 1)),
                sol.routes[r][i+1]
            )

            auxLabel = ForwardLabel(
                solver.prevLabelF.state,
                solver.prevLabelF.cost,
                extendAlongArc(solver.res.stdResource1,
                            solver.prevLabelStdF,
                            (sol.routes[r][j] + 1, sol.routes[r][i] + 1)),
                extendAlongArc(solver.res.stdResource2,
                            solver.prevLabelStdF,
                            (sol.routes[r][j] + 1, sol.routes[r][i] + 1)),
                sol.routes[r][i]
            )

            auxLabel = ForwardLabel(
                auxLabel.state,
                auxLabel.cost,
                extendAlongArc(solver.res.stdResource1,
                            auxLabel,
                            (auxLabel.last + 1, sol.routes[r][j+1] + 1)),
                extendAlongArc(solver.res.stdResource2,
                            auxLabel,
                            (auxLabel.last + 1, sol.routes[r][j+1] + 1)),
                sol.routes[r][j+1]
            )

            resStd1 = concatenationCost(
                solver.res.stdResource1,
                sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1].last,
                auxLabel,
                sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1]
            )

            resStd2 = concatenationCost(
                solver.res.stdResource2,
                sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1].last,
                auxLabel,
                sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1]
            )

            return resStd1.stdWarp, resStd2.stdWarp

        else

            solver.prevLabelStdF = ForwardLabel(
                solver.prevLabelF.state,
                solver.prevLabelF.cost,
                extendAlongArc(solver.res.stdResource1,
                            solver.prevLabelStdF,
                            (sol.routes[r][i-1]+1, sol.routes[r][j] + 1)),
                extendAlongArc(solver.res.stdResource2,
                            solver.prevLabelStdF,
                            (sol.routes[r][i-1]+1, sol.routes[r][j] + 1)),
                sol.routes[r][j]
            )

            auxLabel = ForwardLabel(
                solver.prevLabelF.state,
                solver.prevLabelF.cost,
                extendAlongArc(solver.res.stdResource1,
                            solver.prevLabelStdF,
                            (sol.routes[r][j]+1, sol.routes[r][i] + 1)),
                extendAlongArc(solver.res.stdResource2,
                            solver.prevLabelStdF,
                            (sol.routes[r][j]+1, sol.routes[r][i] + 1)),
                sol.routes[r][i]
            )

            auxLabel = ForwardLabel(
                auxLabel.state,
                auxLabel.cost,
                extendAlongArc(solver.res.stdResource1,
                            auxLabel,
                            (auxLabel.last+1, sol.routes[r][j+1]+1)),
                extendAlongArc(solver.res.stdResource2,
                            auxLabel,
                            (auxLabel.last+1, sol.routes[r][j+1]+1)),
                sol.routes[r][j+1]
            )

            resStd1 = concatenationCost(
                solver.res.stdResource1,
                sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1].last,
                auxLabel,
                sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1]
            )

            resStd2 = concatenationCost(
                solver.res.stdResource2,
                sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1].last,
                auxLabel,
                sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1]
            )

            return resStd1.stdWarp, resStd2.stdWarp
        end

    else

        if j == i-1

            solver.prevLabelStdB = BackwardLabel(
                solver.prevLabelStdB.state,
                solver.prevLabelStdB.cost,
                extendAlongArc(solver.res.stdResource1,
                            sol.backwardLabels[r][length(sol.routes[r]) - i],
                            (sol.routes[r][i+1]+1, sol.routes[r][i-1] + 1)),
                extendAlongArc(solver.res.stdResource2,
                            sol.backwardLabels[r][length(sol.routes[r]) - i],
                            (sol.routes[r][i+1]+1, sol.routes[r][i-1] + 1)),
                sol.routes[r][i-1]
            )

            auxLabel = BackwardLabel(
                solver.prevLabelB.state,
                solver.prevLabelB.cost,
                extendAlongArc(solver.res.stdResource1,
                            solver.prevLabelStdB,
                            (solver.prevLabelStdB.last+1, sol.routes[r][i] + 1)),
                extendAlongArc(solver.res.stdResource2,
                            solver.prevLabelStdB,
                            (solver.prevLabelStdB.last+1, sol.routes[r][i] + 1)),
                sol.routes[r][i]
            )

            auxLabel = BackwardLabel(
                auxLabel.state,
                auxLabel.cost,
                extendAlongArc(solver.res.stdResource1,
                            auxLabel,
                            (sol.routes[r][j]+1, sol.routes[r][j-1]+1)),
                extendAlongArc(solver.res.stdResource2,
                            auxLabel,
                            (sol.routes[r][j]+1, sol.routes[r][j-1]+1)),
                sol.routes[r][j-1]
            )

            resStd1 = concatenationCost(
                solver.res.stdResource1,
                auxLabel.last,
                sol.forwardLabels[r][j - 1],
                auxLabel
            )

            resStd2 = concatenationCost(
                solver.res.stdResource2,
                auxLabel.last,
                sol.forwardLabels[r][j - 1],
                auxLabel
            )

            return resStd1.stdWarp, resStd2.stdWarp

        else

            solver.prevLabelStdB = BackwardLabel(
                solver.prevLabelStdB.state,
                solver.prevLabelStdB.cost,
                extendAlongArc(solver.res.stdResource1,
                            solver.prevLabelStdB,
                            (solver.prevLabelStdB.last+1, sol.routes[r][j] + 1)),
                extendAlongArc(solver.res.stdResource2,
                            solver.prevLabelStdB,
                            (solver.prevLabelStdB.last+1, sol.routes[r][j] + 1)),
                sol.routes[r][j]
            )

            auxLabel = BackwardLabel(
                solver.prevLabelB.state,
                solver.prevLabelB.cost,
                extendAlongArc(solver.res.stdResource1,
                            solver.prevLabelStdB,
                            (solver.prevLabelStdB.last+1, sol.routes[r][i] + 1)),
                extendAlongArc(solver.res.stdResource2,
                            solver.prevLabelStdB,
                            (solver.prevLabelStdB.last+1, sol.routes[r][i] + 1)),
                sol.routes[r][i]
            )

            auxLabel = BackwardLabel(
                auxLabel.state,
                auxLabel.cost,
                extendAlongArc(solver.res.stdResource1,
                            auxLabel,
                            (sol.routes[r][j]+1, sol.routes[r][j-1]+1)),
                extendAlongArc(solver.res.stdResource2,
                            auxLabel,
                            (sol.routes[r][j]+1, sol.routes[r][j-1]+1)),
                sol.routes[r][j-1]
            )

            resStd1 = concatenationCost(
                solver.res.stdResource1,
                auxLabel.last,
                sol.forwardLabels[r][j - 1],
                auxLabel
            )

            resStd2 = concatenationCost(
                solver.res.stdResource2,
                auxLabel.last,
                sol.forwardLabels[r][j - 1],
                auxLabel
            )

            return resStd1.stdWarp, resStd2.stdWarp
        end
    end
end

function computeStdViolInsertion1(solver::Solver, sol::Solution, r::Int, customer::Int, pos::Int)
    forwardLabel = sol.forwardLabels[r][pos-1]
    backwardLabel = sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos]

    std1ResForward = extendAlongArc(solver.res.stdResource1, forwardLabel, (sol.routes[r][pos-1]+1, customer+1))
    std2ResForward = extendAlongArc(solver.res.stdResource2, forwardLabel, (sol.routes[r][pos-1]+1, customer+1))

    insertionLabelForward = ForwardLabel(forwardLabel.state, forwardLabel.cost, std1ResForward, std2ResForward, customer)

    std1ResBackward = extendAlongArc(solver.res.stdResource1, backwardLabel, (sol.routes[r][pos]+1, customer+1))
    std2ResBackward = extendAlongArc(solver.res.stdResource2, backwardLabel, (sol.routes[r][pos]+1, customer+1))

    insertionLabelBackward = BackwardLabel(backwardLabel.state, backwardLabel.cost, std1ResBackward, std2ResBackward, customer)

    concatCost1 = concatenationCost(solver.res.stdResource1, insertionLabelBackward.last, insertionLabelForward, insertionLabelBackward)
    concatCost2 = concatenationCost(solver.res.stdResource2, insertionLabelBackward.last, insertionLabelForward, insertionLabelBackward)

    return concatCost1.stdWarp, concatCost2.stdWarp
end

function computeStdViolRemove1(solver::Solver, sol::Solution, r::Int, pos::Int)
    forwardLabel = sol.forwardLabels[r][pos-1]
    backwardLabel = sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos - 1]
    stdRes1Forward = extendAlongArc(solver.res.stdResource1, forwardLabel, (sol.routes[r][pos-1]+1, sol.routes[r][pos+1]+1))
    stdRes2Forward = extendAlongArc(solver.res.stdResource2, forwardLabel, (sol.routes[r][pos-1]+1, sol.routes[r][pos+1]+1))

    # removalLabelForward = ForwardLabel(forwardLabel.state, forwardLabel.cost, stdResForward, vcat(forwardLabel.path, sol.routes[r][pos+1]), sol.routes[r][pos+1])
    removalLabelForward = ForwardLabel(forwardLabel.state, forwardLabel.cost, stdRes1Forward, stdRes2Forward, sol.routes[r][pos+1])

    concatStd1 = concatenationCost(solver.res.stdResource1, backwardLabel.last, removalLabelForward, backwardLabel)
    concatStd2 = concatenationCost(solver.res.stdResource2, backwardLabel.last, removalLabelForward, backwardLabel)

    return concatStd1.stdWarp, concatStd2.stdWarp
end

function computeStdViolSwap11(solver::Solver, sol::Solution, r::Int, pos::Int, customer::Int)
    forwardLabel = sol.forwardLabels[r][pos-1]
    backwardLabel = sol.backwardLabels[r][length(sol.routes[r]) - pos]
    stdRes1Forward = extendAlongArc(solver.res.stdResource1, forwardLabel, (sol.routes[r][pos-1]+1, customer + 1))
    stdRes2Forward = extendAlongArc(solver.res.stdResource2, forwardLabel, (sol.routes[r][pos-1]+1, customer + 1))

    swapLabelForward = ForwardLabel(forwardLabel.state, forwardLabel.cost, stdRes1Forward, stdRes2Forward, customer)

    stdRes1Backward = extendAlongArc(solver.res.stdResource1, backwardLabel, (sol.routes[r][pos+1]+1, customer+1))
    stdRes2Backward = extendAlongArc(solver.res.stdResource2, backwardLabel, (sol.routes[r][pos+1]+1, customer+1))

    swapLabelBackward = BackwardLabel(backwardLabel.state, backwardLabel.cost, stdRes1Backward, stdRes2Backward, customer)

    concatStd1 = concatenationCost(solver.res.stdResource1, swapLabelBackward.last, swapLabelForward, swapLabelBackward)
    concatStd2 = concatenationCost(solver.res.stdResource2, swapLabelBackward.last, swapLabelForward, swapLabelBackward)

    return concatStd1.stdWarp, concatStd2.stdWarp
end

function computeStdViolTwoOptStar(solver::Solver, sol::Solution, r1::Int, r2::Int, i::Int, j::Int)
    # route r1
    forwardLabel1 = sol.forwardLabels[r1][i]
    backwardLabel1 = sol.backwardLabels[r2][length(sol.routes[r2]) - j]
    stdRes1Forward = extendAlongArc(solver.res.stdResource1, forwardLabel1, (sol.routes[r1][i]+1, sol.routes[r2][j+1] + 1))
    stdRes2Forward = extendAlongArc(solver.res.stdResource2, forwardLabel1, (sol.routes[r1][i]+1, sol.routes[r2][j+1] + 1))

    forwardLabel1 = ForwardLabel(forwardLabel1.state, forwardLabel1.cost, stdRes1Forward, stdRes2Forward, sol.routes[r2][j+1])

    concat1Std1 = concatenationCost(solver.res.stdResource1, backwardLabel1.last, forwardLabel1, backwardLabel1)
    concat1Std2 = concatenationCost(solver.res.stdResource2, backwardLabel1.last, forwardLabel1, backwardLabel1)

    # route r2
    forwardLabel2 = sol.forwardLabels[r2][j]
    backwardLabel2 = sol.backwardLabels[r1][length(sol.routes[r1]) - i]
    stdRes1Forward = extendAlongArc(solver.res.stdResource1, forwardLabel2, (sol.routes[r2][j]+1, sol.routes[r1][i+1] + 1))
    stdRes2Forward = extendAlongArc(solver.res.stdResource2, forwardLabel2, (sol.routes[r2][j]+1, sol.routes[r1][i+1] + 1))

    # forwardLabel2 = ForwardLabel(forwardLabel2.state, forwardLabel2.cost, stdResForward, vcat(forwardLabel2.path, sol.routes[r1][i+1]), sol.routes[r1][i+1])
    forwardLabel2 = ForwardLabel(forwardLabel2.state, forwardLabel2.cost, stdRes1Forward, stdRes2Forward,  sol.routes[r1][i+1])

    concat2Std1 = concatenationCost(solver.res.stdResource1, backwardLabel2.last, forwardLabel2, backwardLabel2)
    concat2Std2 = concatenationCost(solver.res.stdResource1, backwardLabel2.last, forwardLabel2, backwardLabel2)

    return concat1Std1.stdWarp, concat1Std2.stdWarp, concat2Std1.stdWarp, concat2Std2.stdWarp
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



