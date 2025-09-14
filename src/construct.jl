


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
        push!(solver.outerCurrSol.warps, 0.0)
        push!(solver.outerCurrSol.infeas, 0)
        vertices[selectedIdx], vertices[r] = vertices[r], vertices[selectedIdx]
    end
    computeLabels(solver, solver.outerCurrSol)
    for k = nbRoutes+1:length(vertices)
        bestI = 0
        bestJ = 0
        bestR = 0
        sol = solver.outerCurrSol
        # currCost = solver.outerCurrSol.cost#solver.currSol.cost
        currDist = solver.outerCurrSol.dist#solver.currSol.cost
        bestDist = Inf
        bestCost = Inf
        bestInfeas = typemax(Int)
        bestWarp = Inf
        bestInsertion = BestInsertion()
        improvement = false
        for i = k:length(vertices)
            for r = 1:nbRoutes
                for j = 2:length(solver.outerCurrSol.routes[r])
                    # dist, cost, feas, warp = evalBestInsertion(currDist, sol, solver.outerCurrSol.routes, solver, r, vertices[i].id, j)
                    dist, cost, infeas, warp = evalBestInsertion(solver, solver.outerCurrSol, Insertion(r, vertices[i].id, j))

                    # infeas = length(solver.outerCurrSol.routes[r]) - 1 - feas + 1
                    # cost = objectiveValue(solver, solver.outerCurrSol, r, dist, infeas, warp)
                    # cost = dist + solver.params.penaltyInfeas * infeas + solver.params.penaltyStandard * (solver.outerCurrSol.totalWarp - solver.outerCurrSol.warps[r] + warp)
                    #=
                        # println("Inserindo cliente $(vertices[i].id) na posição $j da rota $(solver.outerCurrSol.routes[r]) \nFeas: $(feas), Infeas: $(infeas), warp: $(warp)\n")
                        if infeas < 0
                            println("$(solver.outerCurrSol.routes[r]), $(vertices[i].id), $j")
                            println("$feas, $(length(solver.outerCurrSol.routes[r]))")
                            println("$(solver.outerCurrSol.lastFeasibleF[r]) $(solver.outerCurrSol.lastFeasibleB[r])")
                            sleep(1000)
                        end
                        auxSol = deepcopy(sol)
                        insert!(auxSol.routes[r], j, vertices[i].id)
                        computedInfeas, accDemand1F, accDemand1B = checkInfeasibles(solver, auxSol.routes[r])
                        currComputedInfeas, currAccDemand1F, currAccDemand1B = checkInfeasibles(solver, solver.outerCurrSol.routes[r])
                        demand = 0
                        for i = 1:length(auxSol.routes[r])-2
                            demand += solver.res.customResource.d[auxSol.routes[r][i]+1, auxSol.routes[r][i+1]+1]
                        end
                        if warp != max(demand - solver.res.customResource.Q, 0)
                            @show warp, demand - solver.res.customResource.Q
                            throw()
                        end
                        # @show computedInfeas
                        if infeas < computedInfeas# && infeas > -50
                            # if j != 2 
                                println("cust: $(solver.outerCurrSol.routes[r][i]) pos: $j")
                                println("curr r1: $(solver.outerCurrSol.routes[r])")
                                println("acum D r1 F: $(currAccDemand1F)")
                                println("acum D r1 B: $(currAccDemand1B)")
                                println()
                                println("r1: $(auxSol.routes[r])")
                                println("acum D r1 F: $(accDemand1F)")
                                println("acum D r1 B: $(accDemand1B)")
                                println("Infeas: $(infeas), Computed Infeas: $(computedInfeas)")
                                sleep(1000)
                            # end
                        end
                    =#
                    if cost < bestInsertion.cost - 1e-6
                        improvement = true
                    else
                        improvement = false
                    end
                    # improvement = improved(cost, bestCost, infeas, bestInfeas)
                    if improvement
                        bestInsertion = BestInsertion(cost, dist, r, vertices[i].id, j, infeas, warp)
                        bestI = i
                    end
                end
            end
        end
        if bestInsertion.pos > 0
            applyMoveInsertion(solver, solver.outerCurrSol, bestInsertion)
            computeLabels(solver, solver.outerCurrSol, [bestInsertion.route])
            vertices[bestI], vertices[k] = vertices[k], vertices[bestI]
            solver.outerCurrSol.infeas[bestInsertion.route] = length(solver.outerCurrSol.routes[bestInsertion.route]) - max(solver.outerCurrSol.feasiblesF[bestInsertion.route], solver.outerCurrSol.feasiblesB[bestInsertion.route]) - 1           
            solver.outerCurrSol.totalInfeas = sum(solver.outerCurrSol.infeas)
        end
    end
end