


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

        push!(solver.outerCurrSol.warps, 0.0)
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
                    dist, cost, infeas, warp = evalBestInsertion(solver, solver.outerCurrSol, Insertion(r, vertices[i].id, j))
                    if cost < bestInsertion.cost - 1e-6
                        bestInsertion = BestInsertion(cost, dist, r, vertices[i].id, j, infeas, warp)
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
    nVizinhas = 4  # exemplo: :intraShift, :interShift, :interSwap, :twoOptStar

    solver.outerCurrSol.lastEval = zeros(Int, nVizinhas, R, R)

    for r in 1:R
        solver.outerCurrSol.lastEval[1, r, r] = solver.timeStamp
    end
    for move = 2:4
        for r1 in 1:R-1
            for r2 in r1+1:R
                solver.outerCurrSol.lastEval[move, r1, r2] = solver.timeStamp
                # solver.outerCurrSol.lastEval[(:interShift, r1, r2)] = solver.timeStamp
                # solver.outerCurrSol.lastEval[(:interSwap, r1, r2)] = solver.timeStamp
                # solver.outerCurrSol.lastEval[(:twoOptStar, r1, r2)] = solver.timeStamp
            end
        end
    end
end