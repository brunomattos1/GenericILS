function innerLoop!(solver::Solver, sol::Solution)
    algo = solver.algorithm
    it = 0
    copy_solution!(algo.bestSol, sol)
    while it < algo.innerIterMax
        it += 1
        RVND!(solver, sol)
        if algo.aggressivePool
            registerRoutePool!(solver, sol)
        end
        if acceptSol(solver, sol, algo.bestSol)
            copy_solution!(algo.bestSol, sol)
            it = 0
        end
        copy_solution!(sol, algo.bestSol)
        innerPerturb!(solver, sol)
    end
    copy_solution!(algo.outerCandidateSol, algo.bestSol)
end
