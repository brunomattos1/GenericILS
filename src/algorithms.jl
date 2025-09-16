
function NILS(solver::Solver)
    solver.outerBestSol.cost = Inf
    solver.bestSol = Solution()
    bestFeasCost = Inf
    for r = 1:solver.params.restarts
        constructSol!(solver)
        ILS(solver, solver.outerCurrSol)
        push!(solver, solver.outerCurrSol)
        if acceptSol(solver, solver.outerCurrSol, solver.outerBestSol)
            copy_solution!(solver.outerBestSol, solver.outerCurrSol)
            if (solver.outerBestSol.totalInfeas == 0) && (solver.outerBestSol.totalWarp <= 1e-6)
                bestFeasCost = solver.outerBestSol.cost
            end
        end
        outerIter = 0
        while outerIter < solver.params.outerIterMax
            outerIter += 1
            outerPerturb!(solver, solver.outerCurrSol)
            ILS(solver, solver.outerCurrSol)
            push!(solver, solver.outerCurrSol)

            @printf("%3s| %3s | %8.2f | %8.2f | %8.2f | %8.2f | %8.2f | %d\n",
            r,
            outerIter,
            bestFeasCost, 
            solver.outerBestSol.cost,
            solver.outerCurrSol.cost,
            solver.params.penaltyCustom,
            solver.params.penaltyStandard,
            length(solver.pool))

            if acceptSol(solver, solver.outerCurrSol, solver.outerBestSol)
                copy_solution!(solver.outerBestSol, solver.outerCurrSol)
                if (solver.outerBestSol.totalInfeas == 0) && (solver.outerBestSol.totalWarp <= 1e-6)
                    bestFeasCost = solver.outerBestSol.cost
                    outerIter = 0
                end
            elseif (solver.outerCurrSol.cost < bestFeasCost - 1e-6) && (solver.outerCurrSol.totalInfeas == 0 && solver.outerCurrSol.totalWarp <= 1e-6)
                bestFeasCost = solver.outerCurrSol.cost
                outerIter = 0
            end
        end
    end

    # for (route, cost) in solver.pool
    #     if cost >= 1.02*bestFeasCost
    #         delete!(solver.pool, route)
    #     end
    # end
    setPartitioning(solver, bestFeasCost)
    solver.outerBestSol = deepcopy(solver.currSol)
    # if acceptSol(solver, solver.currSol, solver.outerBestSol)
    #     solver.outerBestSol = deepcopy(solver.currSol)
    # end
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