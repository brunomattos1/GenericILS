
function ILS(solver::Solver)
    solver.bestSol.cost = Inf
    # solver.bestSol.resViolation = [1e6 for _ = 1:solver.data.maxNbRoutes]
    for r = 1:solver.params.restarts
        constructSol!(solver)
        # checkCVRP(solver, solver.currSol)
        # println("RVND")
        RVND!(solver)
        push!(solver)
        it = 0
        if acceptSol(solver)
            solver.bestSol = deepcopy(solver.currSol)
            println("Best solution updated at restart $r iteration $it. Cost: $(solver.bestSol.cost).")
        end
        while it < solver.params.iterMax
            it += 1
            perturb!(solver)
            # checkCVRP(solver, solver.currSol)
            cost_perturb = solver.currSol.cost
            viol_perturb = length(solver.data.vertices) - max(sum(solver.currSol.feasiblesF), sum(solver.currSol.feasiblesB))
            # println("RVND")
            RVND!(solver)
            # checkCVRP(solver, solver.currSol)
            cost_rvnd = solver.currSol.cost
            viol_rvnd = length(solver.data.vertices) - max(sum(solver.currSol.feasiblesF), sum(solver.currSol.feasiblesB))
            push!(solver)

            accepted = acceptSol(solver)
            r_star = @sprintf("%04d", r) * (accepted ? "*" : " ")
            it_star = @sprintf("%04d", it) * (accepted ? "*" : " ")
            @printf("%s| %s| %8.2f |%02s | %8.2f |%02s | %8.2f |%02s | %d\n",
            r_star,
            it_star,
            solver.bestSol.cost,
            length(solver.data.vertices) - max(sum(solver.bestSol.feasiblesF), sum(solver.bestSol.feasiblesB)),
            cost_perturb,
            viol_perturb,
            cost_rvnd,
            viol_rvnd,
            length(solver.pool))
            if accepted
                solver.bestSol = deepcopy(solver.currSol)
                it = 0
            end

        end
    end
    setPartitioning(solver)
    if acceptSol(solver)
        solver.bestSol = deepcopy(solver.currSol)
    end
end