function infeasArcsRemovalK(solver::Solver, sol::Solution, r::Int, pos::Int, k::Int)
    rt     = sol.routes[r]
    lenR   = length(rt.visits)
    bwdIdx = lenR - pos - k + 1

    bridgeFwd = myExtendAlongArc(solver.res,
        rt.forwardLabels[pos-1],
        (rt.forwardLabels[pos-1].last + 1, rt.visits[pos+k] + 1))

    bridgeBwd = myExtendAlongArc(solver.res,
        rt.backwardLabels[bwdIdx],
        (rt.backwardLabels[bwdIdx].last + 1, rt.visits[pos-1] + 1))

    L = myConcatenationCost(solver.res,
        rt.backwardLabels[bwdIdx].last, bridgeFwd, rt.backwardLabels[bwdIdx])
    warpStd1, warpStd2 = L.std1State.stdWarp, L.std2State.stdWarp
    L.cost < Inf && return 0, L.cost, warpStd1, warpStd2

    nFwd = min(pos - 2,        rt.lastFeasibleF - 1)
    bridgeFwd.cost < Inf && (nFwd += 1)
    nBwd = min(lenR - k - pos, rt.lastFeasibleB - 1)
    bridgeBwd.cost < Inf && (nBwd += 1)

    return (lenR - k - 1) - max(nFwd, nBwd), min(bridgeFwd.cost, bridgeBwd.cost), warpStd1, warpStd2
end

function infeasArcsInsertionK(solver::Solver, sol::Solution, r::Int, customers::AbstractVector{Int}, pos::Int)
    rt     = sol.routes[r]
    k      = length(customers)
    lenR   = length(rt.visits)
    res    = solver.res
    bwdIdx = lenR + 1 - pos

    nFwd = min(pos - 2,    rt.lastFeasibleF - 1)
    nBwd = min(lenR - pos, rt.lastFeasibleB - 1)

    # forward chain: forwardLabels[pos-1] → c[1] → … → c[k] → vPos
    currFwd = myExtendAlongArc(res, rt.forwardLabels[pos-1],
                  (rt.visits[pos-1]+1, customers[1]+1))
    currFwd.cost < Inf && (nFwd += 1)
    for m in 2:k
        currFwd = myExtendAlongArc(res, currFwd, (customers[m-1]+1, customers[m]+1))
        currFwd.cost < Inf && (nFwd += 1)
    end
    finalFwd = myExtendAlongArc(res, currFwd, (customers[k]+1, rt.visits[pos]+1))
    finalFwd.cost < Inf && (nFwd += 1)

    # backward chain: backwardLabels[bwdIdx] → c[k] → … → c[1] → v_{pos-1}
    currBwd = myExtendAlongArc(res, rt.backwardLabels[bwdIdx],
                  (rt.visits[pos]+1, customers[k]+1))
    currBwd.cost < Inf && (nBwd += 1)
    for m in 2:k
        currBwd = myExtendAlongArc(res, currBwd, (customers[k-m+2]+1, customers[k-m+1]+1))
        currBwd.cost < Inf && (nBwd += 1)
    end
    finalBwd = myExtendAlongArc(res, currBwd, (customers[1]+1, rt.visits[pos-1]+1))
    finalBwd.cost < Inf && (nBwd += 1)

    L = myConcatenationCost(res, rt.backwardLabels[bwdIdx].last,
                            finalFwd, rt.backwardLabels[bwdIdx])
    warpStd1, warpStd2 = L.std1State.stdWarp, L.std2State.stdWarp
    L.cost < Inf && return 0, L.cost, warpStd1, warpStd2

    return (lenR + k - 1) - max(nFwd, nBwd), min(finalFwd.cost, finalBwd.cost), warpStd1, warpStd2
end

function infeasArcsReplaceBlockK(solver::Solver, sol::Solution, r::Int, pos::Int, kRemove::Int, newCustomers::AbstractVector{Int})
    rt       = sol.routes[r]
    kInsert = length(newCustomers)
    lenR     = length(rt.visits)
    res      = solver.res
    bwdIdx   = lenR - pos - kRemove + 1

    nFwd = min(pos - 2,               rt.lastFeasibleF - 1)
    nBwd = min(lenR - pos - kRemove, rt.lastFeasibleB - 1)

    # forward chain: forwardLabels[pos-1] → newCustomers[1..kInsert] → v_{pos+kRemove}
    currFwd = myExtendAlongArc(res, rt.forwardLabels[pos-1],
                  (rt.visits[pos-1]+1, newCustomers[1]+1))
    currFwd.cost < Inf && (nFwd += 1)
    for m in 2:kInsert
        currFwd = myExtendAlongArc(res, currFwd,
                      (newCustomers[m-1]+1, newCustomers[m]+1))
        currFwd.cost < Inf && (nFwd += 1)
    end
    finalFwd = myExtendAlongArc(res, currFwd,
                   (newCustomers[kInsert]+1, rt.visits[pos+kRemove]+1))
    finalFwd.cost < Inf && (nFwd += 1)

    # backward chain: backwardLabels[bwdIdx] → newCustomers[kInsert..1] → v_{pos-1}
    currBwd = myExtendAlongArc(res, rt.backwardLabels[bwdIdx],
                  (rt.visits[pos+kRemove]+1, newCustomers[kInsert]+1))
    currBwd.cost < Inf && (nBwd += 1)
    for m in 2:kInsert
        currBwd = myExtendAlongArc(res, currBwd,
                      (newCustomers[kInsert-m+2]+1, newCustomers[kInsert-m+1]+1))
        currBwd.cost < Inf && (nBwd += 1)
    end
    finalBwd = myExtendAlongArc(res, currBwd,
                   (newCustomers[1]+1, rt.visits[pos-1]+1))
    finalBwd.cost < Inf && (nBwd += 1)

    L = myConcatenationCost(res, rt.backwardLabels[bwdIdx].last,
                            finalFwd, rt.backwardLabels[bwdIdx])
    warpStd1, warpStd2 = L.std1State.stdWarp, L.std2State.stdWarp
    L.cost < Inf && return 0, L.cost, warpStd1, warpStd2

    totalArcs = (lenR - 1) + (kInsert - kRemove)
    return totalArcs - max(nFwd, nBwd), min(finalFwd.cost, finalBwd.cost), warpStd1, warpStd2
end

function infeasArcs2optStar(solver::Solver, sol::Solution,
                            rHead::Int, rTail::Int,
                            posHead::Int, posTail::Int)
    rtHead   = sol.routes[rHead]
    rtTail   = sol.routes[rTail]
    lenRTail = length(rtTail.visits)

    vecL = myExtendAlongArc(solver.res,
        rtHead.forwardLabels[posHead],
        (rtHead.visits[posHead] + 1, rtTail.visits[posTail+1] + 1))

    cevL = myExtendAlongArc(solver.res,
        rtTail.backwardLabels[lenRTail - posTail],
        (rtTail.visits[posTail+1] + 1, rtHead.visits[posHead] + 1))

    L = myConcatenationCost(solver.res,
        rtTail.backwardLabels[lenRTail - posTail].last,
        vecL,
        rtTail.backwardLabels[lenRTail - posTail])

    warpStd1, warpStd2 = L.std1State.stdWarp, L.std2State.stdWarp
    if L.cost < Inf
        return 0, L.cost, warpStd1, warpStd2
    end

    nFwd = min(posHead - 1, rtHead.lastFeasibleF - 1)
    vecL.cost < Inf && (nFwd += 1)

    nBwd = min(lenRTail - 1 - posTail, rtTail.lastFeasibleB - 1)
    cevL.cost < Inf && (nBwd += 1)

    totalArcs = posHead + lenRTail - posTail - 1
    return totalArcs - max(nFwd, nBwd), min(vecL.cost, cevL.cost), warpStd1, warpStd2
end

