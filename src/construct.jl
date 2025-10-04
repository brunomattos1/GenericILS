


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
        vertices[selectedIdx], vertices[r] = vertices[r], vertices[selectedIdx]
    end
    computeLabels(solver, solver.outerCurrSol)
    for k = nbRoutes+1:length(vertices)
        bestI = 0
        bestInsertion = BestInsertion()
        improvement = false
        for i = k:length(vertices)
            for r = 1:nbRoutes
                for j = 2:length(solver.outerCurrSol.routes[r])
                    dist, cost, infeas, warp = evalBestInsertion(solver, solver.outerCurrSol, Insertion(r, vertices[i].id, j))
                    if cost < bestInsertion.cost - 1e-6
                        improvement = true
                    else
                        improvement = false
                    end
                    if improvement
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
end