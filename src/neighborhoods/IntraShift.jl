struct IntraShift end

function intraShift10Cost(currCost::Float64, costMatrix::Matrix{Float64}, route::Vector{Int}, i::Int, j::Int)
    if j == i+1
        newCost = currCost - costMatrix[route[i-1]+1, route[i]+1] - costMatrix[route[i]+1, route[i+1]+1] - costMatrix[route[j]+1, route[j+1]+1]
        newCost += costMatrix[route[i-1]+1, route[j]+1] + costMatrix[route[j]+1, route[i]+1] + costMatrix[route[i]+1, route[j+1]+1]
    else
        newCost = currCost - costMatrix[route[i-1]+1, route[i]+1] - costMatrix[route[i]+1, route[i+1]+1] - costMatrix[route[j-1]+1, route[j]+1]
        newCost += costMatrix[route[i-1]+1, route[i+1]+1] + costMatrix[route[j-1]+1, route[i]+1] + costMatrix[route[i]+1, route[j]+1]
    end
    return newCost
end

function computeViolIntraShift10(solver::Solver, sol::Solution, r::Int, i::Int, j::Int)
    rt = sol.routes[r]
    if i < j
        if j == i+1
            solver.prevLabelF = myExtendAlongArc(solver.res.customResource, rt.forwardLabels[i-1], (rt.visits[i-1]+1, rt.visits[i+1] + 1))
            auxLabel = myExtendAlongArc(solver.res.customResource, solver.prevLabelF, (solver.prevLabelF.last+1, rt.visits[i] + 1))
            auxLabel = myExtendAlongArc(solver.res.customResource, auxLabel, (auxLabel.last + 1, rt.visits[j+1] + 1))
            res = myConcatenationCost(solver.res.customResource, rt.backwardLabels[length(rt.visits) + 1 - j - 1].last, auxLabel, rt.backwardLabels[length(rt.visits) + 1 - j - 1])
        else
            solver.prevLabelF = myExtendAlongArc(solver.res.customResource, solver.prevLabelF, (solver.prevLabelF.last + 1, rt.visits[j] + 1))
            auxLabel = myExtendAlongArc(solver.res.customResource, solver.prevLabelF, (solver.prevLabelF.last+1, rt.visits[i] + 1))
            auxLabel = myExtendAlongArc(solver.res.customResource, auxLabel, (auxLabel.last + 1, rt.visits[j+1] + 1))
            res = myConcatenationCost(solver.res.customResource, rt.backwardLabels[length(rt.visits) + 1 - j - 1].last, auxLabel, rt.backwardLabels[length(rt.visits) + 1 - j - 1])
        end
    else
        if j == i-1
            solver.prevLabelB = myExtendAlongArc(solver.res.customResource, rt.backwardLabels[length(rt.visits) - i], (rt.visits[i+1]+1, rt.visits[i-1] + 1))
            auxLabel = myExtendAlongArc(solver.res.customResource, solver.prevLabelB, (rt.visits[i-1]+1, rt.visits[i] + 1))
            auxLabel = myExtendAlongArc(solver.res.customResource, auxLabel, (rt.visits[j]+1, rt.visits[j-1] + 1))
            res = myConcatenationCost(solver.res.customResource, auxLabel.last, rt.forwardLabels[j - 1], auxLabel)
        else
            solver.prevLabelB = myExtendAlongArc(solver.res.customResource, solver.prevLabelB, (solver.prevLabelB.last+1, rt.visits[j] + 1))
            auxLabel = myExtendAlongArc(solver.res.customResource, solver.prevLabelB, (rt.visits[j]+1, rt.visits[i] + 1))
            auxLabel = myExtendAlongArc(solver.res.customResource, auxLabel, (rt.visits[j]+1, rt.visits[j-1] + 1))
            res = myConcatenationCost(solver.res.customResource, auxLabel.last, rt.forwardLabels[j - 1], auxLabel)
        end
    end
    if res.cost >= Inf
        return 1, res.cost
    else
        return 0, res.cost
    end
end

function evalIntraShift10(solver::Solver, sol::Solution, shift::Shift)
    r = shift.routeFrom
    i = shift.fromIdx
    j = shift.toIdx
    dist     = intraShift10Cost(sol.dist, solver.data.costMatrix, sol.routes[r].visits, i, j)
    resViol, labelCost = computeViolIntraShift10(solver, sol, r, i, j)
    warpStd1, warpStd2 = computeStdViolIntraShift10(solver, sol, r, i, j)
    cost = objectiveValue(solver, sol, r, dist, 0, labelCost, warpStd1, warpStd2)
    return dist, cost, resViol
end

function search!(neigh::IntraShift, solver::Solver, sol::Solution)
    improved = false
    copy_solution!(solver.bufferSol, sol)
    oldSol = solver.bufferSol
    neighborhoodId = neigh_index(typeof(neigh))
    for r = 1:length(sol.routes)
        rt = sol.routes[r]
        if sol.lastEval[neighborhoodId, r, r] > rt.lastModif
            continue
        end
        bestMove = BestMove(cost = sol.cost, dist = sol.dist)
        if rt.feasibleF >= length(rt.visits) - 1
            for i = 2:length(rt.visits)-1
                for j = i+1:length(rt.visits)-1
                    dist, cost, resViol = evalIntraShift10(solver, sol, Shift(r, r, i, j))
                    if resViol == 0 && cost < bestMove.cost - 1e-6
                        bestMove = BestMove(cost, dist, r, 0, i, j)
                    end
                end
            end
        end
        if rt.feasibleB >= length(rt.visits) - 1
            for i = length(rt.visits)-1:-1:2
                for j = i-1:-1:2
                    dist, cost, resViol = evalIntraShift10(solver, sol, Shift(r, r, i, j))
                    if resViol == 0 && cost < bestMove.cost - 1e-6
                        bestMove = BestMove(cost, dist, r, 0, i, j)
                    end
                end
            end
        end
        sol.timeStamp += 1
        sol.lastEval[neighborhoodId, r, r] = sol.timeStamp
        if bestMove.firstIdx > 0
            prevCost = oldSol.cost
            apply!(neigh, solver, sol, bestMove)
            if sol.cost < prevCost - 1e-6
                improved = true
                copy_solution!(oldSol, sol)
            else
                copy_solution!(sol, oldSol)
            end
        end
    end
    return improved
end

function apply!(::IntraShift, solver::Solver, solution::Solution, move::BestMove)
    r  = move.firstRoute
    i, j = move.firstIdx, move.secondIdx
    rt = solution.routes[r]

    solution.dist            = move.dist
    solution.totalInfeas    -= rt.infeas
    solution.totalWarpStd1  -= rt.warpStd1
    solution.totalWarpStd2  -= rt.warpStd2
    solution.totalLabelCost -= rt.labelCost

    route = rt.visits
    if i < j && j == i + 1
        route[i], route[j] = route[j], route[i]
    else
        move_blocks_intra!(route, i, 1, j, solver.bufferRoute)
    end

    computeLabels(solver, solution, r)

    rt.infeas = length(rt.visits) - max(rt.feasibleF, rt.feasibleB) - 1
    solution.totalInfeas += rt.infeas

    rt.warpStd1 = rt.forwardLabels[end].std1State.stdWarp
    solution.totalWarpStd1 += rt.warpStd1

    rt.warpStd2 = rt.forwardLabels[end].std2State.stdWarp
    solution.totalWarpStd2 += rt.warpStd2

    rt.labelCost = min(rt.forwardLabels[end].cost, rt.backwardLabels[end].cost)
    solution.totalLabelCost += rt.labelCost

    solution.cost = objectiveValue(solver, solution)
    rt.lastModif = solution.timeStamp
end
