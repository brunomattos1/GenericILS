struct Insertion <: Move
    route::Int
    customer::Int
    pos::Int
end

function constructSol!(solver::Solver; r::Int = 0)
    bestParallelInsertion(solver)
end


function bestParallelInsertion(solver::Solver; r::Int = 0)
    vertices = deepcopy(solver.data.vertices)
    nbRoutes = min(length(vertices), solver.data.maxNbRoutes + r)
    costMatrix = getCostMatrix(solver)
    solver.outerCurrSol = new_solution(solver)
    for r = 1:nbRoutes
        selectedIdx = rand(solver.seed, r:length(vertices))
        selected = vertices[selectedIdx]
        push!(solver.outerCurrSol.routes, new_route(solver, Int[0, selected.id, 0]))
        solver.outerCurrSol.dist += costMatrix[1, selected.id+1] + costMatrix[selected.id+1, 1]
        solver.outerCurrSol.cost += costMatrix[1, selected.id+1] + costMatrix[selected.id+1, 1]
        vertices[selectedIdx], vertices[r] = vertices[r], vertices[selectedIdx]
    end
    computeLabels(solver, solver.outerCurrSol)
    for r = 1:length(solver.outerCurrSol.routes)
        rt = solver.outerCurrSol.routes[r]
        rt.labelCost = min(rt.forwardLabels[end].cost, rt.backwardLabels[end].cost)
    end
    solver.outerCurrSol.totalLabelCost = sum(rt.labelCost for rt in solver.outerCurrSol.routes)
    solver.outerCurrSol.cost = objectiveValue(solver, solver.outerCurrSol)
    for k = nbRoutes+1:length(vertices)
        bestI = 0
        bestInsertion = BestInsertion()
        for i = k:length(vertices)
            for r = 1:nbRoutes
                for j = 2:length(solver.outerCurrSol.routes[r].visits)
                    dist, cost, infeas, warpStd1, warpStd2 = evalBestInsertion(solver, solver.outerCurrSol, Insertion(r, vertices[i].id, j))
                    if cost < bestInsertion.cost - 1e-6
                        bestInsertion = BestInsertion(cost, dist, r, vertices[i].id, j, infeas, warpStd1, warpStd2)
                        bestI = i
                    end
                end
            end
        end
        if bestInsertion.pos > 0
            applyMoveInsertion(solver, solver.outerCurrSol, bestInsertion)
            vertices[bestI], vertices[k] = vertices[k], vertices[bestI]
        end
    end
    R = length(solver.outerCurrSol.routes)
    nVizinhas = length(NEIGHBORHOODS)
    solver.outerCurrSol.timeStamp = 0
    solver.outerCurrSol.lastEval = zeros(Int, nVizinhas, R, R)

    for r in 1:R
        solver.outerCurrSol.lastEval[1, r, r] = 0
    end
    for move = 2:nVizinhas
        for r1 in 1:R-1
            for r2 in r1+1:R
                solver.outerCurrSol.lastEval[move, r1, r2] = 0
            end
        end
    end
end

function applyMoveInsertion(solver::Solver, solution::Solution, move::BestInsertion)
    r  = move.route
    i  = move.pos
    c  = move.customer
    rt = solution.routes[r]

    solution.dist = move.dist
    solution.totalInfeas   -= rt.infeas
    solution.totalInfeas   += move.infeas
    rt.infeas               = move.infeas

    solution.totalWarpStd1 -= rt.warpStd1
    solution.totalWarpStd1 += move.warpStd1
    rt.warpStd1             = move.warpStd1
    solution.totalWarpStd2 -= rt.warpStd2
    solution.totalWarpStd2 += move.warpStd2
    rt.warpStd2             = move.warpStd2

    insert!(rt.visits, i, c)
    computeLabels(solver, solution, r)

    solution.totalInfeas -= rt.infeas
    rt.infeas = length(rt.visits) - max(rt.feasibleF, rt.feasibleB) - 1
    solution.totalInfeas += rt.infeas

    solution.totalLabelCost -= rt.labelCost
    rt.labelCost = min(rt.forwardLabels[end].cost, rt.backwardLabels[end].cost)
    solution.totalLabelCost += rt.labelCost
    solution.cost = objectiveValue(solver, solution)
end

function bestInsertionCost(currCost::Float64, costMatrix::Matrix{Float64}, route::Vector{Int}, customer::Int, j::Int)
    newCost = currCost - costMatrix[route[j-1]+1, route[j]+1] + costMatrix[route[j-1]+1, customer+1] + costMatrix[customer+1, route[j]+1]
    return newCost
end

function evalBestInsertion(solver::Solver, sol::Solution, insertion::Insertion)
    currCost = sol.dist
    r        = insertion.route
    customer = insertion.customer
    j        = insertion.pos
    dist = bestInsertionCost(currCost, solver.data.costMatrix, sol.routes[r].visits, customer, j)
    infeas, labelCost, warpStd1, warpStd2 = infeasArcsInsertionK(solver, sol, r, [customer], j)
    cost = objectiveValue(solver, sol, r, dist, infeas, labelCost, warpStd1, warpStd2)
    return dist, cost, infeas, warpStd1, warpStd2
end
