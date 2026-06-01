
_is_feasible(sol::Solution) =
    sol.totalInfeas == 0 && sol.totalWarpStd1 <= 1e-6 && sol.totalWarpStd2 <= 1e-6

function NILS(solver::Solver,
              stopCrit   = IterationLimit(solver.parameters.outerIterMax),
              acceptCrit = BestOnly())
    solver.outerBestSol.cost = Inf
    solver.bestSol = Solution()
    total_algorithm_time = 0.0 # Podemos capturar o tempo de brinde
    header_time = 20.0
    solver.bestFeasSol = Solution()
    solver.bestFeasSol.cost = Inf
    println("-"^135)

    @printf("| %7s | %6s | %10s | %12s | %12s | %10s | %15s | %15s | %6s | %10s |\n",
        "Restart",
        "Iter.",
        "Best Feas",
        "Best",
        "Curr",
        "Pen. Custom",
        "Pen. Standard 1",
        "Pen. Standard 2",
        "Pool",
        "Time (s)"
    )

    println("-"^135)
    ts = time()
    for r = 1:solver.parameters.restarts
        constructSol!(solver)
        push!(solver, solver.outerCurrSol)
        ILS(solver, solver.outerCurrSol)

        if accept!(acceptCrit, solver.outerCurrSol, solver.outerBestSol, solver.seed)
            copy_solution!(solver.outerBestSol, solver.outerCurrSol)
            if _is_feasible(solver.outerBestSol)
                copy_solution!(solver.bestFeasSol, solver.outerBestSol)
            end
        end

        reset!(stopCrit)
        reset!(acceptCrit)

        while !stop!(stopCrit) && !stop!(acceptCrit)
            tick!(stopCrit)
            outerPerturb!(solver, solver.outerCurrSol)
            ILS(solver, solver.outerCurrSol)
            push!(solver, solver.outerCurrSol)

            if accept!(acceptCrit, solver.outerCurrSol, solver.outerBestSol, solver.seed)
                copy_solution!(solver.outerBestSol, solver.outerCurrSol)
                if _is_feasible(solver.outerBestSol)
                    copy_solution!(solver.bestFeasSol, solver.outerBestSol)
                    improved!(stopCrit)
                    improved!(acceptCrit)
                end
            elseif _is_feasible(solver.outerCurrSol) &&
                   solver.outerCurrSol.cost < solver.bestFeasSol.cost - 1e-6
                copy_solution!(solver.bestFeasSol, solver.outerCurrSol)
                improved!(stopCrit)
                improved!(acceptCrit)
            end

            revert!(acceptCrit, solver.outerCurrSol, solver.outerBestSol)
            update!(acceptCrit)

            total_algorithm_time = time() - ts

            if total_algorithm_time >= header_time
                println("-"^135)
                @printf("| %7s | %6s | %10s | %12s | %12s | %10s | %15s | %15s | %6s | %10s |\n",
                    "Restart", "Iter.", "Best Feas", "Best", "Curr",
                    "Pen. Custom", "Pen. Standard 1", "Pen. Standard 2", "Pool", "Time (s)")
                println("-"^135)
                header_time += 20.0
            end

            iter_display = stopCrit isa IterationLimit ? stopCrit.iter :
                           acceptCrit isa SimulatedAnnealing ? round(Int, acceptCrit.T) : 0
            @printf("| %7d | %6d | %10.2f | %12.2f | %12.2f | %11.2f | %15.2f | %15.2f | %6d | %10.4f |\n",
                r, iter_display,
                solver.bestFeasSol.cost, solver.outerBestSol.cost, solver.outerCurrSol.cost,
                solver.parameters.penaltyCustom,
                solver.parameters.penaltyStandard1, solver.parameters.penaltyStandard2,
                length(solver.route_storage), total_algorithm_time)

            if total_algorithm_time >= solver.timeLimitILS
                @goto SP
            end
        end
    end
    # for (route, cost) in solver.pool
    #     if cost >= 1.02*bestFeasCost
    #         delete!(solver.pool, route)
    #     end
    # end
    @label SP
    println("-"^144)
    if total_algorithm_time >= solver.timeLimitILS
        println("Search finished due to time limit! Executing Set Partitioning model...")
    else
        println("Search finished! Executing Set Partitioning model...")
    end
    println("-"^144)
    setPartitioning(solver, solver.bestFeasSol.cost)
    solver.outerBestSol = deepcopy(solver.bestFeasSol)
end

function ILS(solver::Solver, sol::Solution)
    it = 0
    copy_solution!(solver.bestSol, sol)
    while it < solver.parameters.innerIterMax
        it += 1
        RVND!(solver, sol)
        if solver.aggressivePool
            push!(solver, sol)
        end
        if acceptSol(solver, sol, solver.bestSol)
            copy_solution!(solver.bestSol, sol)
            it = 0
        end
        copy_solution!(sol, solver.bestSol)
        innerPerturb!(solver, sol)
    end
    copy_solution!(solver.outerCurrSol, solver.bestSol)
end

function classicILS(solver::Solver)
    solver.outerBestSol.cost = Inf
    for r = 1:solver.parameters.restarts
        constructSol!(solver)
        RVND!(solver, solver.outerCurrSol)

        push!(solver, solver.outerCurrSol)
        if r == 1#acceptSol(solver, solver.outerCurrSol, solver.outerBestSol)
            solver.outerBestSol = deepcopy(solver.outerCurrSol)
        end
        iter = 0
        while iter < solver.parameters.outerIterMax
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