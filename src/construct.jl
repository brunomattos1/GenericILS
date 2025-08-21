function constructSol!(solver::Solver)
    bestParallelInsertion(solver::Solver)
end


function bestParallelInsertion(solver::Solver)
    vertices = copy(solver.data.vertices)
    nbRoutes = min(length(vertices), solver.data.maxNbRoutes)
    costMatrix = getCostMatrix(solver)
    solver.currSol = Solution()
    for r = 1:nbRoutes
        selectedIdx = rand(solver.seed, r:length(vertices))
        selected = vertices[selectedIdx]
        push!(solver.currSol.routes, Int[0, selected.id, 0])
        solver.currSol.cost += costMatrix[1, selected.id+1] + costMatrix[selected.id+1, 1]
        vertices[selectedIdx], vertices[r] = vertices[r], vertices[selectedIdx]
    end
    computeLabels(solver)
    for k = nbRoutes+1:length(vertices)
        bestI = 0
        bestJ = 0
        bestR = 0
        currCost = solver.currSol.cost
        bestCost = Inf
        bestInfeas = typemax(Int)
        for i = k:length(vertices)
            for r = 1:nbRoutes
                for j = 2:length(solver.currSol.routes[r])
                    cost, feas = evalBestInsertion(currCost, solver.currSol.routes, solver, r, vertices[i].id, j)
                    infeas = length(solver.currSol.routes[r]) - 2 - feas
                    # println("Inserindo cliente $(vertices[i].id) na posição $j da rota $(solver.currSol.routes[r]) \nFeas: $(feas), Infeas: $(infeas)\n")
                    # if infeas < 0
                    #     println("$(solver.currSol.routes[r]), $(vertices[i].id), $j")
                    #     println("$feas, $(length(solver.currSol.routes[r]))")
                    #     println("$(solver.currSol.lastFeasibleF[r]) $(solver.currSol.lastFeasibleB[r])")
                    #     sleep(1000)
                    # end
                    improvement = improved(cost, bestCost, infeas, bestInfeas)
                    if improvement
                        bestCost = cost
                        bestI = i
                        bestJ = j
                        bestR = r
                        bestInfeas = infeas
                    end
                end
            end
        end
        if bestI > 0
            applyMoveInsertion(solver, bestCost, bestR, vertices[bestI].id, bestJ)
            computeLabels(solver, [bestR])
            vertices[bestI], vertices[k] = vertices[k], vertices[bestI]
        end
    end
end

function bestParallelInsertion2(solver::Solver)
    vertices = copy(solver.data.vertices)
    nbRoutes = min(length(vertices), solver.data.maxNbRoutes)
    costMatrix = getCostMatrix(solver)
    solver.currSol = Solution()
    currSol = solver.currSol
    for r = 1:nbRoutes
        selectedIdx = rand(solver.seed, 1:length(vertices))
        selected = vertices[selectedIdx]
        push!(currSol.routes, Int[0, 0])
        computeLabels(solver)
        currSol.cost += costMatrix[1, selected.id+1] + costMatrix[selected.id+1, 1]
        # currSol.resViolation += computeViolInsertion(solver, r, selected.id, 2)
        push!(currSol.resViolation, computeViolInsertion(solver, r, selected.id, 2))
        insert!(currSol.routes[r], 2, selected.id)
        setdiff!(vertices, [vertices[selectedIdx]])
        # vertices[selectedIdx], vertices[r] = vertices[r], vertices[selectedIdx]
    end
    computeLabels(solver)
    while length(vertices) > 0
        bestI = 0
        bestJ = 0
        bestR = 0
        currCost = currSol.cost
        bestCost = Inf
        bestResViol = 999
        for i = 1:length(vertices)
            for r = 1:nbRoutes
                for j = 2:length(currSol.routes[r])
                    cost, resViol = evalBestInsertion(currCost, solver.currSol.routes, solver, r, vertices[i].id, j)
                    # if resViol > 0
                    #     continue
                    # end
                    improvement = improved(bestCost, cost, bestResViol, resViol)
                    if improvement
                        bestCost = cost
                        bestI = i
                        bestJ = j
                        bestR = r
                        bestResViol = resViol
                        # initRouteI = initRoute[i.id]
                    end
                end
            end
        end
        if bestI > 0
            applyMoveInsertion(solver, bestCost, bestResViol, bestR, vertices[bestI].id, bestJ)
            computeLabels(solver)
            # vertices[bestI], vertices[k] = vertices[k], vertices[bestI]
            setdiff!(vertices, [vertices[bestI]])
        end
    end
end