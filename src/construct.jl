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
    # solver.currSol = Solution()
    solver.outerCurrSol = Solution()
    for r = 1:nbRoutes
        selectedIdx = rand(solver.seed, r:length(vertices))
        selected = vertices[selectedIdx]
        push!(solver.outerCurrSol.routes, Int[0, selected.id, 0])
        solver.outerCurrSol.dist += costMatrix[1, selected.id+1] + costMatrix[selected.id+1, 1]
        solver.outerCurrSol.cost += costMatrix[1, selected.id+1] + costMatrix[selected.id+1, 1]

        push!(solver.outerCurrSol.warpsStd1, 0.0)
        push!(solver.outerCurrSol.warpsStd2, 0.0)

        push!(solver.outerCurrSol.infeas, 0)
        push!(solver.outerCurrSol.lastModif, 0)
        vertices[selectedIdx], vertices[r] = vertices[r], vertices[selectedIdx]
    end
    computeLabels(solver, solver.outerCurrSol)
    for r = 1:length(solver.outerCurrSol.routes)
        push!(solver.outerCurrSol.labelCosts, min(solver.outerCurrSol.forwardLabels[r][end].cost, solver.outerCurrSol.backwardLabels[r][end].cost))
    end
    solver.outerCurrSol.totalLabelCost = sum(solver.outerCurrSol.labelCosts)
    solver.outerCurrSol.cost = objectiveValue(solver, solver.outerCurrSol)
    # if isCostResource()
    #     solver.outerCurrSol.cost += solver.outerCurrSol.totalLabelCost
    # end
    for k = nbRoutes+1:length(vertices)
        bestI = 0
        bestInsertion = BestInsertion()
        for i = k:length(vertices)
            for r = 1:nbRoutes
                for j = 2:length(solver.outerCurrSol.routes[r])
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
    r = move.route
    i = move.pos
    c = move.customer

    # atualizar distÃ¢ncia e custo
    solution.dist = move.dist
    solution.cost = move.cost
    # atualizar infeasibility
    solution.totalInfeas -= solution.infeas[r]
    solution.totalInfeas += move.infeas
    solution.infeas[r] = move.infeas

    # atualizar warp
    solution.totalWarpStd1 -= solution.warpsStd1[r]
    solution.totalWarpStd1 += move.warpStd1
    solution.warpsStd1[r] = move.warpStd1
    solution.totalWarpStd2 -= solution.warpsStd2[r]
    solution.totalWarpStd2 += move.warpStd2
    solution.warpsStd2[r] = move.warpStd2
    # inserir cliente
    insert!(solution.routes[r], i, c)
    computeLabels(solver, solution, r)

    solution.totalInfeas -= solution.infeas[r]
    solution.infeas[r] = length(solution.routes[r]) - max(solution.feasiblesF[r], solution.feasiblesB[r]) - 1
    solution.totalInfeas += solution.infeas[r]

    # solution.totalWarp -= solution.warps[r]
    # solution.warps[r] = solution.forwardLabels[r][end].std_res.stdWarp
    # solution.totalWarp += solution.warps[r]
    solution.totalLabelCost -= solution.labelCosts[r]
    solution.labelCosts[r] = min(solution.forwardLabels[r][end].cost, solution.backwardLabels[r][end].cost)
    solution.totalLabelCost += solution.labelCosts[r]
    solution.cost = objectiveValue(solver, solution)

end

function bestInsertionCost(currCost::Float64, costMatrix::Matrix{Float64}, route::Vector{Int}, customer::Int, j::Int)
    newCost = currCost - costMatrix[route[j-1]+1, route[j]+1] + costMatrix[route[j-1]+1, customer+1] + costMatrix[customer+1, route[j]+1]
    return newCost
end

function evalBestInsertion(solver::Solver, sol::Solution, insertion::Insertion)
    currCost = sol.dist
    routes = sol.routes
    r = insertion.route
    customer = insertion.customer
    j = insertion.pos
    dist = bestInsertionCost(currCost, solver.data.costMatrix, routes[r], customer, j)
    warpStd1, warpStd2 = computeStdViolInsertionK(solver, sol, r, [customer], j)
    # feas, labelCost = computeViolInsertion1(solver, sol, r, customer, j)
    # infeas = length(sol.routes[r]) - 1 - feas + 1
    infeas, labelCost = infeasArcsInsertionK(solver, sol, r, [customer], j)
    cost = objectiveValue(solver, sol, r, dist, infeas, labelCost, warpStd1, warpStd2)
    return dist, cost, infeas, warpStd1, warpStd2
end