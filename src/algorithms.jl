
function NILS(solver::Solver)
    solver.outerBestSol.cost = Inf
    solver.bestSol = Solution()
    total_algorithm_time = 0.0 # Podemos capturar o tempo de brinde
    header_time = 20.0
    bestFeasSol = Solution()
    bestFeasSol.cost = Inf
    println("-"^144)

    # Atualizamos o cabeçalho para refletir a nova precisão
    @printf("| %8s | %10s | %16s | %12s | %12s | %21s | %23s | %6s | %10s |\n",
                        "Restart", "Iteration", "Best Feas. Cost", "Best Cost", "Curr. Cost", "Penalty Custom Res.", "Penalty Standard Res.", "Pool", "Time (s)")
    println("-"^144)
    ts = time()
    for r = 1:solver.params.restarts
        constructSol!(solver)
        push!(solver, solver.outerCurrSol)
        ILS(solver, solver.outerCurrSol)

        if acceptSol(solver, solver.outerCurrSol, solver.outerBestSol)
            copy_solution!(solver.outerBestSol, solver.outerCurrSol)
            if (solver.outerBestSol.totalInfeas == 0) && (solver.outerBestSol.totalWarp <= 1e-6)
                copy_solution!(bestFeasSol, solver.outerBestSol)
            end
        end
        outerIter = 0
        while outerIter < solver.params.outerIterMax
            outerIter += 1
            outerPerturb!(solver, solver.outerCurrSol)
            ILS(solver, solver.outerCurrSol)
            push!(solver, solver.outerCurrSol)
            if acceptSol(solver, solver.outerCurrSol, solver.outerBestSol)
                copy_solution!(solver.outerBestSol, solver.outerCurrSol)
                if (solver.outerBestSol.totalInfeas == 0) && (solver.outerBestSol.totalWarp <= 1e-6)
                    copy_solution!(bestFeasSol, solver.outerBestSol)
                    computeLabels(solver, bestFeasSol)
                    outerIter = 0
                end
            elseif (solver.outerCurrSol.cost < bestFeasSol.cost - 1e-6) && (solver.outerCurrSol.totalInfeas == 0 && solver.outerCurrSol.totalWarp <= 1e-6)
                copy_solution!(bestFeasSol, solver.outerCurrSol)
                outerIter = 0
            end
            total_algorithm_time = time() - ts

            if total_algorithm_time >= header_time
                println("-"^144)
                @printf("| %8s | %10s | %16s | %12s | %12s | %21s | %23s | %6s | %10s |\n",
                        "Restart", "Iteration", "Best Feas. Cost", "Best Cost", "Curr. Cost", "Penalty Custom Res.", "Penalty Standard Res.", "Pool", "Time (s)")
                println("-"^144)
                header_time += 20.0
            end
            @printf("| %8d | %10d | %16.2f | %12.2f | %12.2f | %21.2f | %23.2f | %6d | %10.4f |\n",
                r,
                outerIter,
                bestFeasSol.cost,
                solver.outerBestSol.cost,
                solver.outerCurrSol.cost,
                solver.params.penaltyCustom,
                solver.params.penaltyStandard,
                length(solver.route_storage),
                total_algorithm_time
            )

        end
    end
    # for (route, cost) in solver.pool
    #     if cost >= 1.02*bestFeasCost
    #         delete!(solver.pool, route)
    #     end
    # end
    println("-"^144)
    setPartitioning(solver, bestFeasSol.cost)
    solver.outerBestSol = deepcopy(solver.currSol)
    # solver.outerBestSol = bestFeasSol
end

function ILS(solver::Solver, sol::Solution)
    it = 0
    copy_solution!(solver.bestSol, sol)
    while it < solver.params.innerIterMax
        it += 1
        RVND!(solver, sol)
        if acceptSol(solver, sol, solver.bestSol)
            copy_solution!(solver.bestSol, sol)
            it = 0
        end
        innerPerturb!(solver, sol)
    end
    copy_solution!(solver.outerCurrSol, solver.bestSol)
end

function classicILS(solver::Solver)
    solver.outerBestSol.cost = Inf
    for r = 1:solver.params.restarts
        constructSol!(solver)
        RVND!(solver, solver.outerCurrSol)

        push!(solver, solver.outerCurrSol)
        if r == 1#acceptSol(solver, solver.outerCurrSol, solver.outerBestSol)
            solver.outerBestSol = deepcopy(solver.outerCurrSol)
        end
        iter = 0
        while iter < solver.params.outerIterMax
            iter += 1
            outerPerturb!(solver, solver.outerCurrSol)

            # sleep(1000)
            # @show solver.outerCurrSol.infeas
            # @show solver.outerCurrSol.totalInfeas
            # sleep(1000)
            # checkCVRP(solver, solver.currSol)
            cost_perturb = solver.outerCurrSol.cost
            viol_perturb = length(solver.data.vertices) - max(sum(solver.outerCurrSol.feasiblesF), sum(solver.outerCurrSol.feasiblesB))
            # println("RVND")
            RVND!(solver, solver.outerCurrSol)
            # checkCVRP(solver, solver.currSol)
            cost_rvnd = solver.outerCurrSol.cost
            viol_rvnd = length(solver.data.vertices) - max(sum(solver.outerCurrSol.feasiblesF), sum(solver.outerCurrSol.feasiblesB))
            push!(solver, solver.outerCurrSol)

            accepted = acceptSol(solver, solver.outerCurrSol, solver.outerBestSol)
            r_star = @sprintf("%04d", r) * (accepted ? "*" : " ")
            it_star = @sprintf("%04d", iter) * (accepted ? "*" : " ")
            @printf("%s| %s| %8.2f |%02s | %8.2f |%02s | %8.2f |%02s | %d\n",
            r_star,
            it_star,
            solver.outerBestSol.cost,
            length(solver.data.vertices) - max(sum(solver.outerBestSol.feasiblesF), sum(solver.outerBestSol.feasiblesB)),
            cost_perturb,
            viol_perturb,
            cost_rvnd,
            viol_rvnd,
            length(solver.pool))
            if accepted
                solver.outerBestSol = deepcopy(solver.outerCurrSol)
                println("Best solution updated at restart $r iteration $iter. Cost: $(solver.outerBestSol.cost).")
                iter = 0
            end
        end
    end

    # for (route, cost) in solver.pool
    #     if cost >= 1.03*solver.outerBestSol.cost
    #         delete!(solver.pool, route)
    #     end
    # end
    setPartitioning(solver, solver.outerBestSol.cost)
    # if acceptSol(solver, solver.currSol, solver.outerBestSol)
    #     solver.outerBestSol = deepcopy(solver.currSol)
    # end
end