function computeViolInsertion1(solver::Solver, sol::Solution, r::Int, customer::Int, pos::Int)
    insertionLabelForward = myExtendAlongArc(solver.res, sol.forwardLabels[r][pos-1], (sol.routes[r][pos-1]+1, customer + 1))
    concatCost = myConcatenationCost(solver.res, customer, insertionLabelForward, sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos])
    if concatCost.cost < Inf
        return length(sol.routes[r]) - 1 + 1
    end

    # FORWARD
    if sol.feasiblesF[r] == length(sol.routes[r]) - 1
        concatCost = myConcatenationCost(solver.res, customer, insertionLabelForward, sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos])
        if insertionLabelForward.cost == Inf
            dFeasForward = 0 - length(sol.routes[r]) + pos
            if pos == length(sol.routes[r])
                dFeasForward = - 1
            end
        elseif insertionLabelForward.cost < Inf && concatCost.cost < Inf
            dFeasForward = 1
        elseif insertionLabelForward.cost < Inf && concatCost.cost == Inf
            dFeasForward = 1 - length(sol.routes[r]) + pos
        end
        forw = dFeasForward
        if pos <= sol.lastFeasibleF[r]
            forw += pos - 1
        else
            forw += sol.lastFeasibleF[r] - 1
        end
    else
        insertionLabelForward2 = myExtendAlongArc(solver.res, insertionLabelForward, (customer + 1, sol.routes[r][pos]+1))
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
        # TO DO
        if pos > sol.lastFeasibleF[r] + 1
            dFeasForward = 0#-typemax(Int)
        end
        forw = dFeasForward
        if pos <= sol.lastFeasibleF[r]
            forw += pos - 1
        else
            forw += sol.lastFeasibleF[r] - 1
        end
    end

    # BACKWARD
    if sol.feasiblesB[r] == length(sol.routes[r]) - 1
        insertionLabelBackward = myExtendAlongArc(solver.res, sol.backwardLabels[r][length(sol.routes[r]) - pos + 1], (sol.routes[r][pos]+1, customer+1))
        concatCost = myConcatenationCost(solver.res, customer, sol.forwardLabels[r][pos - 1], insertionLabelBackward)
        if insertionLabelBackward.cost == Inf
            dFeasBackward = -1#length(sol.routes[r]) - pos - 2
        elseif insertionLabelBackward.cost < Inf && concatCost.cost < Inf
            dFeasBackward = 1#length(sol.routes[r]) - 1
        elseif insertionLabelBackward.cost < Inf && concatCost.cost == Inf
            dFeasBackward = 0#length(sol.routes[r]) - pos - 2
        end
        backw = dFeasBackward
        if pos > length(sol.routes[r]) + 1 - sol.lastFeasibleB[r]
            backw += length(sol.routes[r]) - pos + 1# - 1# + 1
        else
            backw += sol.lastFeasibleB[r] - 1
        end
    else
        insertionLabelBackward = myExtendAlongArc(solver.res, sol.backwardLabels[r][length(sol.routes[r]) - pos + 1], (sol.routes[r][pos]+1, customer+1))
        insertionLabelBackward2 = myExtendAlongArc(solver.res, insertionLabelBackward, (customer + 1, sol.routes[r][pos-1]+1))
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
        if pos > length(sol.routes[r]) + 1 - sol.lastFeasibleB[r]
            backw += length(sol.routes[r]) - pos + 1# - 1# + 1
        else
            backw += sol.lastFeasibleB[r] - 1
        end
    end
    return max(forw, backw)
end

function computeViolRemove1(solver::Solver, sol::Solution, r::Int, pos::Int)
    concat = myConcatenationCost(solver.res, pos, sol.forwardLabels[r][pos-1], sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos - 1])
    removLabelForward = nothing
    if concat.cost < Inf
        return length(sol.routes[r]) - 1 - 1
    end
    # FORWARD
    if sol.feasiblesF[r] == length(sol.routes[r]) - 1
        removLabelForward = myConcatenationCost(solver.res, pos, sol.forwardLabels[r][pos-1], sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos - 1])
        if removLabelForward.cost < Inf
            dFeasForward = -1
        else
            dFeasForward = -length(sol.routes[r])
        end
        forw = length(sol.routes[r]) - 1 - dFeasForward - 2
    else
        removLabelForward = myExtendAlongArc(solver.res, sol.forwardLabels[r][pos-1], (sol.routes[r][pos-1]+1, sol.routes[r][pos+1]+1))
        if pos < sol.lastFeasibleF[r]
            dFeasForward = - 2
            if removLabelForward.cost < Inf
                dFeasForward += 1
            else
                dFeasForward += 0
            end
        elseif pos == sol.lastFeasibleF[r]
            dFeasForward = - 1
            if removLabelForward.cost < Inf
                dFeasForward += 1
            else
                dFeasForward += 0
            end
        elseif pos == sol.lastFeasibleF[r] + 1
            dFeasForward = 0
            if removLabelForward.cost < Inf
                dFeasForward += 1
            else
                dFeasForward += 0
            end
        elseif pos > sol.lastFeasibleF[r]
            dFeasForward = 0
        end
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
        # @show pos, sol.lastFeasibleF[r]
        # @show removLabelForward.path
    end

    # BACKWARD
    if sol.feasiblesB[r] == length(sol.routes[r]) - 1
        removLabelBackward = myConcatenationCost(solver.res, pos, sol.forwardLabels[r][pos-1], sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos - 1])
        if removLabelBackward.cost < Inf
            dFeasBackward = -1
        else
            dFeasBackward = -length(sol.routes[r])
        end
        backw = length(sol.routes[r]) - 1 - dFeasBackward - 2
    else
        pos_ = length(sol.routes[r]) - pos#+ 1# - 1
        removLabelBackward = myExtendAlongArc(solver.res, sol.backwardLabels[r][pos_], (sol.routes[r][pos_]+1, sol.routes[r][pos-1]+1))
        if pos > length(sol.routes[r]) - sol.lastFeasibleB[r] + 1
            dFeasBackward = - 2
            if removLabelBackward.cost < Inf
                dFeasBackward += 1
            else
                dFeasBackward += 0
            end
        elseif pos == length(sol.routes[r]) - sol.lastFeasibleB[r]
            dFeasBackward = 0
            if removLabelBackward.cost < Inf
                dFeasBackward += 1
            else
                dFeasBackward += 0
            end
        elseif pos == length(sol.routes[r]) - sol.lastFeasibleB[r] + 1
            dFeasBackward = - 1
            if removLabelBackward.cost < Inf
                dFeasBackward += 1
            else
                dFeasBackward += 0
            end
        elseif pos < length(sol.routes[r]) - sol.lastFeasibleB[r] + 1
            dFeasBackward = 0
        end
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
        # @show removLabelBackward
        # @show dFeasBackward
        # @show pos, length(sol.routes[r]) - sol.lastFeasibleB[r] + 1
    end
    # @show forw, backw
    # @show concat.path
    return max(forw, backw)
end

function computeViolSwap11(solver::Solver, sol::Solution, r::Int, pos::Int, customer::Int)
    # FORWARD
    if sol.feasiblesF[r] == length(sol.routes[r]) - 1
        swapLabelForward = myExtendAlongArc(solver.res, sol.forwardLabels[r][pos-1], (sol.routes[r][pos-1]+1, customer + 1))
        concatCost = myConcatenationCost(solver.res, customer, swapLabelForward, sol.backwardLabels[r][length(sol.routes[r]) - pos])
        if swapLabelForward.cost == Inf
            dFeasForward = - length(sol.routes[r]) + pos - 1
        elseif swapLabelForward.cost < Inf && concatCost.cost < Inf
            dFeasForward = 0
        elseif swapLabelForward.cost < Inf && concatCost.cost == Inf
            dFeasForward = - length(sol.routes[r]) + pos
        end
        forw = length(sol.routes[r]) - 1 + dFeasForward
    else
        swapLabelForward = myExtendAlongArc(solver.res, sol.forwardLabels[r][pos-1], (sol.routes[r][pos-1]+1, customer + 1))
        swapLabelForward2 = myExtendAlongArc(solver.res, swapLabelForward, (customer + 1, sol.routes[r][pos+1]+1))
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
    end

    # BACKWARD
    if sol.feasiblesB[r] == length(sol.routes[r]) - 1
        swapLabelBackward = myExtendAlongArc(solver.res, sol.backwardLabels[r][length(sol.routes[r]) - pos], (sol.routes[r][pos]+1, customer+1))
        concatCost = myConcatenationCost(solver.res, customer, sol.forwardLabels[r][pos-1], swapLabelBackward)

        if swapLabelBackward.cost == Inf
            dFeasBackward = - length(sol.routes[r]) + pos# + 1
        elseif swapLabelBackward.cost < Inf && concatCost.cost < Inf
            dFeasBackward = 0
        elseif swapLabelBackward.cost < Inf && concatCost.cost == Inf
            dFeasBackward = - length(sol.routes[r]) + pos - 1
        end
        backw = length(sol.routes[r]) - 1 + dFeasBackward
    else
        swapLabelBackward = myExtendAlongArc(solver.res, sol.backwardLabels[r][length(sol.routes[r]) - pos], (sol.routes[r][pos]+1, customer+1))
        swapLabelBackward2 = myExtendAlongArc(solver.res, swapLabelBackward, (customer + 1, sol.routes[r][pos-1]+1))
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
    end
    return max(forw, backw)
end

function computeViolTwoOptStar(solver::Solver, sol::Solution, r1::Int, r2::Int, i::Int, j::Int)
    # FORWARD r1
    labelForward1 = myExtendAlongArc(solver.res, sol.forwardLabels[r1][i], (sol.routes[r1][i]+1, sol.routes[r2][j+1]+1))
    # concatForward1 = myConcatenationCost(solver.res, 1, labelForward1, solver.backwardLabels[r2][length(sol.routes[r2]) - j - 1])
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
    labelBackward1 = myExtendAlongArc(solver.res, sol.backwardLabels[r2][length(sol.routes[r2]) - j], (sol.routes[r2][j+1]+1, sol.routes[r1][i]+1))
    # concatBackward1 = myConcatenationCost(solver.res, 1, labelBackward1, sol.forwardLabels[r1][i-1])
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
    # FORWARD r2
    labelForward2 = myExtendAlongArc(solver.res, sol.forwardLabels[r2][j], (sol.routes[r2][j]+1, sol.routes[r1][i+1]+1))
    # concatForward2 = myConcatenationCost(solver.res, 1, labelForward2, sol.backwardLabels[r1][length(sol.routes[r1]) - i - 1])
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
    labelBackward2 = myExtendAlongArc(solver.res, sol.backwardLabels[r1][length(sol.routes[r1]) - i], (sol.routes[r1][i+1]+1, sol.routes[r2][j]+1))
    # concatBackward2 = myConcatenationCost(solver.res, 1, labelBackward2, sol.forwardLabels[r2][j-1])
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
    return feasR1, feasR2#max(forw1, backw1) + max(forw2, backw2)
end

function computeViolIntraShift10(solver::Solver, sol::Solution, r::Int, i::Int, j::Int)
    if i < j
        if j == i+1
            solver.prevLabelF = myExtendAlongArc(solver.res, sol.forwardLabels[r][i-1], (sol.routes[r][i-1]+1, sol.routes[r][i+1] + 1))
            auxLabel = myExtendAlongArc(solver.res, solver.prevLabelF, (sol.routes[r][j]+1, sol.routes[r][i] + 1))
            res = myConcatenationCost(solver.res, i, auxLabel, sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1])
        else
            solver.prevLabelF = myExtendAlongArc(solver.res, solver.prevLabelF, (sol.routes[r][i-1]+1, sol.routes[r][j] + 1))
            auxLabel = myExtendAlongArc(solver.res, solver.prevLabelF, (sol.routes[r][j]+1, sol.routes[r][i] + 1))
            res = myConcatenationCost(solver.res, i, auxLabel, sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1])
        end
    else
        if j == i-1
            solver.prevLabelB = myExtendAlongArc(solver.res, sol.backwardLabels[r][length(sol.routes[r]) - i], (sol.routes[r][i+1]+1, sol.routes[r][j] + 1))
            auxLabel = myExtendAlongArc(solver.res, solver.prevLabelB, (sol.routes[r][j]+1, sol.routes[r][j+1] + 1))
            # res = myConcatenationCost(solver.res, j+1, auxLabel, sol.forwardLabels[r][j - 1])
            res = myConcatenationCost(solver.res, j+1, sol.forwardLabels[r][j - 1], auxLabel)
        else
            solver.prevLabelB = myExtendAlongArc(solver.res, solver.prevLabelB, (sol.routes[r][i+1]+1, sol.routes[r][j] + 1))
            auxLabel = myExtendAlongArc(solver.res, solver.prevLabelB, (sol.routes[r][j]+1, sol.routes[r][i] + 1))
            # res = myConcatenationCost(solver.res, j+1, auxLabel, sol.forwardLabels[r][j - 1])
            res = myConcatenationCost(solver.res, j+1, sol.forwardLabels[r][j - 1], auxLabel)

        end
    end
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


function computeStdViolInsertion1(solver::Solver, sol::Solution, r::Int, customer::Int, pos::Int)
    forwardLabel = sol.forwardLabels[r][pos-1]
    backwardLabel = sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos]
    stdRes = extendAlongArc(solver.res.stdResource, forwardLabel, (sol.routes[r][pos-1]+1, customer+1))
    insertionLabelForward = ForwardLabel(forwardLabel.custom_res, forwardLabel.cost, stdRes, customer)
    # insertionLabelForward = ForwardLabel(forwardLabel.custom_res, forwardLabel.cost, stdRes, vcat(forwardLabel.path, customer), customer)
    # @show insertionLabelForward
    # @show backwardLabel
    # @show backwardLabel.last
    concatCost = concatenationCost(solver.res.stdResource, backwardLabel.last, insertionLabelForward, backwardLabel)
    # @show concatCost
    return concatCost.stdWarp
end

function computeStdViolRemove1(solver::Solver, sol::Solution, r::Int, pos::Int)
    forwardLabel = sol.forwardLabels[r][pos-1]
    backwardLabel = sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos - 1]
    concatCost = concatenationCost(solver.res.stdResource, backwardLabel.last, forwardLabel, backwardLabel)
    return concatCost.stdWarp
end

function computeStdViolSwap11(solver::Solver, sol::Solution, r::Int, pos::Int, customer::Int)
    forwardLabel = sol.forwardLabels[r][pos-1]
    backwardLabel = sol.backwardLabels[r][length(sol.routes[r]) - pos]
    stdRes = extendAlongArc(solver.res.stdResource, forwardLabel, (sol.routes[r][pos-1]+1, customer + 1))
    swapLabelForward = ForwardLabel(forwardLabel.custom_res, forwardLabel.cost, stdRes, customer)
    # swapLabelForward = ForwardLabel(forwardLabel.custom_res, forwardLabel.cost, stdRes, vcat(forwardLabel.path, customer), customer)

    concatCost = concatenationCost(solver.res.stdResource, backwardLabel.last, swapLabelForward, backwardLabel)
    return concatCost.stdWarp
end

function computeStdViolTwoOptStar(solver::Solver, sol::Solution, r1::Int, r2::Int, i::Int, j::Int)
    # route r1
    forwardLabel1 = sol.forwardLabels[r1][i]
    backwardLabel1 = sol.backwardLabels[r2][length(sol.routes[r2]) - j]
    concatForward1 = concatenationCost(solver.res.stdResource, backwardLabel1.last, forwardLabel1, backwardLabel1)
    # @show forwardLabel1
    # @show backwardLabel1
    # route r2
    forwardLabel2 = sol.forwardLabels[r2][j]
    backwardLabel2 = sol.backwardLabels[r1][length(sol.routes[r1]) - i]
    # @show forwardLabel2
    # @show backwardLabel2
    concatForward2 = concatenationCost(solver.res.stdResource, backwardLabel1.last, forwardLabel2, backwardLabel2)
    return concatForward1.stdWarp, concatForward2.stdWarp
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



