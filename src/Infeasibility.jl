function infeasArcsRemovalK(solver::Solver, sol::Solution, r::Int, pos::Int, k::Int)
    lenR   = length(sol.routes[r])
    bwdIdx = lenR - pos - k + 1

    bridgeFwd = myExtendAlongArc(solver.res.customResource,
        sol.forwardLabels[r][pos-1],
        (sol.forwardLabels[r][pos-1].last + 1, sol.routes[r][pos+k] + 1))

    bridgeBwd = myExtendAlongArc(solver.res.customResource,
        sol.backwardLabels[r][bwdIdx],
        (sol.backwardLabels[r][bwdIdx].last + 1, sol.routes[r][pos-1] + 1))

    L = myConcatenationCost(solver.res.customResource,
        sol.backwardLabels[r][bwdIdx].last, bridgeFwd, sol.backwardLabels[r][bwdIdx])
    L.cost < Inf && return 0, L.cost

    nFwd = min(pos - 2,        sol.lastFeasibleF[r] - 1)
    bridgeFwd.cost < Inf && (nFwd += 1)
    nBwd = min(lenR - k - pos, sol.lastFeasibleB[r] - 1)
    bridgeBwd.cost < Inf && (nBwd += 1)

    return (lenR - k - 1) - max(nFwd, nBwd), min(bridgeFwd.cost, bridgeBwd.cost)
end

function infeasArcsInsertionK(solver::Solver, sol::Solution, r::Int, customers::AbstractVector{Int}, pos::Int)
    k      = length(customers)
    lenR   = length(sol.routes[r])
    res    = solver.res.customResource
    bwdIdx = lenR + 1 - pos

    nFwd = min(pos - 2,    sol.lastFeasibleF[r] - 1)
    nBwd = min(lenR - pos, sol.lastFeasibleB[r] - 1)

    # forward chain: forwardLabels[pos-1] → c[1] → … → c[k] → v_pos
    currFwd = myExtendAlongArc(res, sol.forwardLabels[r][pos-1],
                  (sol.routes[r][pos-1]+1, customers[1]+1))
    currFwd.cost < Inf && (nFwd += 1)
    for m in 2:k
        currFwd = myExtendAlongArc(res, currFwd, (customers[m-1]+1, customers[m]+1))
        currFwd.cost < Inf && (nFwd += 1)
    end
    finalFwd = myExtendAlongArc(res, currFwd, (customers[k]+1, sol.routes[r][pos]+1))
    finalFwd.cost < Inf && (nFwd += 1)

    # backward chain: backwardLabels[bwdIdx] → c[k] → … → c[1] → v_{pos-1}
    currBwd = myExtendAlongArc(res, sol.backwardLabels[r][bwdIdx],
                  (sol.routes[r][pos]+1, customers[k]+1))
    currBwd.cost < Inf && (nBwd += 1)
    for m in 2:k
        currBwd = myExtendAlongArc(res, currBwd, (customers[k-m+2]+1, customers[k-m+1]+1))
        currBwd.cost < Inf && (nBwd += 1)
    end
    finalBwd = myExtendAlongArc(res, currBwd, (customers[1]+1, sol.routes[r][pos-1]+1))
    finalBwd.cost < Inf && (nBwd += 1)

    L = myConcatenationCost(res, sol.backwardLabels[r][bwdIdx].last,
                            finalFwd, sol.backwardLabels[r][bwdIdx])
    L.cost < Inf && return 0, L.cost

    return (lenR + k - 1) - max(nFwd, nBwd), min(finalFwd.cost, finalBwd.cost)
end

function infeasArcsReplaceBlockK(solver::Solver, sol::Solution, r::Int, pos::Int, k_remove::Int, new_customers::AbstractVector{Int})
    k_insert = length(new_customers)
    lenR   = length(sol.routes[r])
    res    = solver.res.customResource
    bwdIdx = lenR - pos - k_remove + 1

    nFwd = min(pos - 2,               sol.lastFeasibleF[r] - 1)
    nBwd = min(lenR - pos - k_remove, sol.lastFeasibleB[r] - 1)

    # forward chain: forwardLabels[pos-1] → new_customers[1..k_insert] → v_{pos+k_remove}
    currFwd = myExtendAlongArc(res, sol.forwardLabels[r][pos-1],
                  (sol.routes[r][pos-1]+1, new_customers[1]+1))
    currFwd.cost < Inf && (nFwd += 1)
    for m in 2:k_insert
        currFwd = myExtendAlongArc(res, currFwd,
                      (new_customers[m-1]+1, new_customers[m]+1))
        currFwd.cost < Inf && (nFwd += 1)
    end
    finalFwd = myExtendAlongArc(res, currFwd,
                   (new_customers[k_insert]+1, sol.routes[r][pos+k_remove]+1))
    finalFwd.cost < Inf && (nFwd += 1)

    # backward chain: backwardLabels[bwdIdx] → new_customers[k_insert..1] → v_{pos-1}
    currBwd = myExtendAlongArc(res, sol.backwardLabels[r][bwdIdx],
                  (sol.routes[r][pos+k_remove]+1, new_customers[k_insert]+1))
    currBwd.cost < Inf && (nBwd += 1)
    for m in 2:k_insert
        currBwd = myExtendAlongArc(res, currBwd,
                      (new_customers[k_insert-m+2]+1, new_customers[k_insert-m+1]+1))
        currBwd.cost < Inf && (nBwd += 1)
    end
    finalBwd = myExtendAlongArc(res, currBwd,
                   (new_customers[1]+1, sol.routes[r][pos-1]+1))
    finalBwd.cost < Inf && (nBwd += 1)

    L = myConcatenationCost(res, sol.backwardLabels[r][bwdIdx].last,
                            finalFwd, sol.backwardLabels[r][bwdIdx])
    L.cost < Inf && return 0, L.cost

    total_arcs = (lenR - 1) + (k_insert - k_remove)
    return total_arcs - max(nFwd, nBwd), min(finalFwd.cost, finalBwd.cost)
end

function infeasArcs2optStar(solver::Solver, sol::Solution,
                            rHead::Int, rTail::Int,
                            posHead::Int, posTail::Int)
    lenRTail = length(sol.routes[rTail])

    vecL = myExtendAlongArc(solver.res.customResource,
        sol.forwardLabels[rHead][posHead],
        (sol.routes[rHead][posHead] + 1, sol.routes[rTail][posTail+1] + 1))

    cevL = myExtendAlongArc(solver.res.customResource,
        sol.backwardLabels[rTail][lenRTail - posTail],
        (sol.routes[rTail][posTail+1] + 1, sol.routes[rHead][posHead] + 1))

    L = myConcatenationCost(solver.res.customResource,
        sol.backwardLabels[rTail][lenRTail - posTail].last,
        vecL,
        sol.backwardLabels[rTail][lenRTail - posTail])

    if L.cost < Inf
        return 0, L.cost
    end

    nFwd = min(posHead - 1, sol.lastFeasibleF[rHead] - 1)
    vecL.cost < Inf && (nFwd += 1)

    nBwd = min(lenRTail - 1 - posTail, sol.lastFeasibleB[rTail] - 1)
    cevL.cost < Inf && (nBwd += 1)

    totalArcs = posHead + lenRTail - posTail - 1
    return totalArcs - max(nFwd, nBwd), min(vecL.cost, cevL.cost)
end

function computeStdViolIntraShift10(solver::Solver, sol::Solution, r::Int, i::Int, j::Int)
    if i < j

        if j == i+1

            solver.prevLabelStdF = ForwardLabel(
                solver.prevLabelF.state, solver.prevLabelF.cost,
                extendAlongArc(solver.res.stdResource1, sol.forwardLabels[r][i-1], (sol.routes[r][i-1] + 1, sol.routes[r][i+1] + 1)),
                extendAlongArc(solver.res.stdResource2, sol.forwardLabels[r][i-1], (sol.routes[r][i-1] + 1, sol.routes[r][i+1] + 1)),
                sol.routes[r][i+1])

            auxLabel = ForwardLabel(
                solver.prevLabelF.state, solver.prevLabelF.cost,
                extendAlongArc(solver.res.stdResource1, solver.prevLabelStdF, (sol.routes[r][j] + 1, sol.routes[r][i] + 1)),
                extendAlongArc(solver.res.stdResource2, solver.prevLabelStdF, (sol.routes[r][j] + 1, sol.routes[r][i] + 1)),
                sol.routes[r][i])

            auxLabel = ForwardLabel(
                auxLabel.state, auxLabel.cost,
                extendAlongArc(solver.res.stdResource1, auxLabel, (auxLabel.last + 1, sol.routes[r][j+1] + 1)),
                extendAlongArc(solver.res.stdResource2, auxLabel, (auxLabel.last + 1, sol.routes[r][j+1] + 1)),
                sol.routes[r][j+1])

            resStd1 = concatenationCost(solver.res.stdResource1, sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1].last, auxLabel, sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1])
            resStd2 = concatenationCost(solver.res.stdResource2, sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1].last, auxLabel, sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1])
            return resStd1.stdWarp, resStd2.stdWarp

        else

            solver.prevLabelStdF = ForwardLabel(
                solver.prevLabelF.state, solver.prevLabelF.cost,
                extendAlongArc(solver.res.stdResource1, solver.prevLabelStdF, (sol.routes[r][i-1]+1, sol.routes[r][j] + 1)),
                extendAlongArc(solver.res.stdResource2, solver.prevLabelStdF, (sol.routes[r][i-1]+1, sol.routes[r][j] + 1)),
                sol.routes[r][j])

            auxLabel = ForwardLabel(
                solver.prevLabelF.state, solver.prevLabelF.cost,
                extendAlongArc(solver.res.stdResource1, solver.prevLabelStdF, (sol.routes[r][j]+1, sol.routes[r][i] + 1)),
                extendAlongArc(solver.res.stdResource2, solver.prevLabelStdF, (sol.routes[r][j]+1, sol.routes[r][i] + 1)),
                sol.routes[r][i])

            auxLabel = ForwardLabel(
                auxLabel.state, auxLabel.cost,
                extendAlongArc(solver.res.stdResource1, auxLabel, (auxLabel.last+1, sol.routes[r][j+1]+1)),
                extendAlongArc(solver.res.stdResource2, auxLabel, (auxLabel.last+1, sol.routes[r][j+1]+1)),
                sol.routes[r][j+1])

            resStd1 = concatenationCost(solver.res.stdResource1, sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1].last, auxLabel, sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1])
            resStd2 = concatenationCost(solver.res.stdResource2, sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1].last, auxLabel, sol.backwardLabels[r][length(sol.routes[r]) + 1 - j - 1])
            return resStd1.stdWarp, resStd2.stdWarp
        end

    else

        if j == i-1

            solver.prevLabelStdB = BackwardLabel(
                solver.prevLabelStdB.state, solver.prevLabelStdB.cost,
                extendAlongArc(solver.res.stdResource1, sol.backwardLabels[r][length(sol.routes[r]) - i], (sol.routes[r][i+1]+1, sol.routes[r][i-1] + 1)),
                extendAlongArc(solver.res.stdResource2, sol.backwardLabels[r][length(sol.routes[r]) - i], (sol.routes[r][i+1]+1, sol.routes[r][i-1] + 1)),
                sol.routes[r][i-1])

            auxLabel = BackwardLabel(
                solver.prevLabelB.state, solver.prevLabelB.cost,
                extendAlongArc(solver.res.stdResource1, solver.prevLabelStdB, (solver.prevLabelStdB.last+1, sol.routes[r][i] + 1)),
                extendAlongArc(solver.res.stdResource2, solver.prevLabelStdB, (solver.prevLabelStdB.last+1, sol.routes[r][i] + 1)),
                sol.routes[r][i])

            auxLabel = BackwardLabel(
                auxLabel.state, auxLabel.cost,
                extendAlongArc(solver.res.stdResource1, auxLabel, (sol.routes[r][j]+1, sol.routes[r][j-1]+1)),
                extendAlongArc(solver.res.stdResource2, auxLabel, (sol.routes[r][j]+1, sol.routes[r][j-1]+1)),
                sol.routes[r][j-1])

            resStd1 = concatenationCost(solver.res.stdResource1, auxLabel.last, sol.forwardLabels[r][j - 1], auxLabel)
            resStd2 = concatenationCost(solver.res.stdResource2, auxLabel.last, sol.forwardLabels[r][j - 1], auxLabel)
            return resStd1.stdWarp, resStd2.stdWarp

        else

            solver.prevLabelStdB = BackwardLabel(
                solver.prevLabelStdB.state, solver.prevLabelStdB.cost,
                extendAlongArc(solver.res.stdResource1, solver.prevLabelStdB, (solver.prevLabelStdB.last+1, sol.routes[r][j] + 1)),
                extendAlongArc(solver.res.stdResource2, solver.prevLabelStdB, (solver.prevLabelStdB.last+1, sol.routes[r][j] + 1)),
                sol.routes[r][j])

            auxLabel = BackwardLabel(
                solver.prevLabelB.state, solver.prevLabelB.cost,
                extendAlongArc(solver.res.stdResource1, solver.prevLabelStdB, (solver.prevLabelStdB.last+1, sol.routes[r][i] + 1)),
                extendAlongArc(solver.res.stdResource2, solver.prevLabelStdB, (solver.prevLabelStdB.last+1, sol.routes[r][i] + 1)),
                sol.routes[r][i])

            auxLabel = BackwardLabel(
                auxLabel.state, auxLabel.cost,
                extendAlongArc(solver.res.stdResource1, auxLabel, (sol.routes[r][j]+1, sol.routes[r][j-1]+1)),
                extendAlongArc(solver.res.stdResource2, auxLabel, (sol.routes[r][j]+1, sol.routes[r][j-1]+1)),
                sol.routes[r][j-1])

            resStd1 = concatenationCost(solver.res.stdResource1, auxLabel.last, sol.forwardLabels[r][j - 1], auxLabel)
            resStd2 = concatenationCost(solver.res.stdResource2, auxLabel.last, sol.forwardLabels[r][j - 1], auxLabel)
            return resStd1.stdWarp, resStd2.stdWarp
        end
    end
end

function computeStdViolRemoveK(solver::Solver, sol::Solution, r::Int, pos::Int, k::Int)
    forwardLabel  = sol.forwardLabels[r][pos-1]
    backwardLabel = sol.backwardLabels[r][length(sol.routes[r]) - pos - k + 1]
    std1Fwd = extendAlongArc(solver.res.stdResource1, forwardLabel,
                  (sol.routes[r][pos-1]+1, sol.routes[r][pos+k]+1))
    std2Fwd = extendAlongArc(solver.res.stdResource2, forwardLabel,
                  (sol.routes[r][pos-1]+1, sol.routes[r][pos+k]+1))
    bridgeLabel = ForwardLabel(forwardLabel.state, forwardLabel.cost,
                      std1Fwd, std2Fwd, sol.routes[r][pos+k])
    c1 = concatenationCost(solver.res.stdResource1, backwardLabel.last, bridgeLabel, backwardLabel)
    c2 = concatenationCost(solver.res.stdResource2, backwardLabel.last, bridgeLabel, backwardLabel)
    return c1.stdWarp, c2.stdWarp
end

function computeStdViolInsertionK(solver::Solver, sol::Solution, r::Int, customers::AbstractVector{Int}, pos::Int)
    forwardLabel  = sol.forwardLabels[r][pos-1]
    backwardLabel = sol.backwardLabels[r][length(sol.routes[r]) + 1 - pos]
    lbl = forwardLabel
    for c in customers
        s1  = extendAlongArc(solver.res.stdResource1, lbl, (lbl.last+1, c+1))
        s2  = extendAlongArc(solver.res.stdResource2, lbl, (lbl.last+1, c+1))
        lbl = ForwardLabel(lbl.state, lbl.cost, s1, s2, c)
    end
    c1 = concatenationCost(solver.res.stdResource1, backwardLabel.last, lbl, backwardLabel)
    c2 = concatenationCost(solver.res.stdResource2, backwardLabel.last, lbl, backwardLabel)
    return c1.stdWarp, c2.stdWarp
end

function computeStdViolTwoOptStar(solver::Solver, sol::Solution, r1::Int, r2::Int, i::Int, j::Int)
    forwardLabel1  = sol.forwardLabels[r1][i]
    backwardLabel1 = sol.backwardLabels[r2][length(sol.routes[r2]) - j]
    stdRes1Fwd = extendAlongArc(solver.res.stdResource1, forwardLabel1, (sol.routes[r1][i]+1, sol.routes[r2][j+1]+1))
    stdRes2Fwd = extendAlongArc(solver.res.stdResource2, forwardLabel1, (sol.routes[r1][i]+1, sol.routes[r2][j+1]+1))
    forwardLabel1 = ForwardLabel(forwardLabel1.state, forwardLabel1.cost, stdRes1Fwd, stdRes2Fwd, sol.routes[r2][j+1])
    concat1Std1 = concatenationCost(solver.res.stdResource1, backwardLabel1.last, forwardLabel1, backwardLabel1)
    concat1Std2 = concatenationCost(solver.res.stdResource2, backwardLabel1.last, forwardLabel1, backwardLabel1)

    forwardLabel2  = sol.forwardLabels[r2][j]
    backwardLabel2 = sol.backwardLabels[r1][length(sol.routes[r1]) - i]
    stdRes1Fwd = extendAlongArc(solver.res.stdResource1, forwardLabel2, (sol.routes[r2][j]+1, sol.routes[r1][i+1]+1))
    stdRes2Fwd = extendAlongArc(solver.res.stdResource2, forwardLabel2, (sol.routes[r2][j]+1, sol.routes[r1][i+1]+1))
    forwardLabel2 = ForwardLabel(forwardLabel2.state, forwardLabel2.cost, stdRes1Fwd, stdRes2Fwd, sol.routes[r1][i+1])
    concat2Std1 = concatenationCost(solver.res.stdResource1, backwardLabel2.last, forwardLabel2, backwardLabel2)
    concat2Std2 = concatenationCost(solver.res.stdResource1, backwardLabel2.last, forwardLabel2, backwardLabel2)

    return concat1Std1.stdWarp, concat1Std2.stdWarp, concat2Std1.stdWarp, concat2Std2.stdWarp
end