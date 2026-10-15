"""
Inner loop of NILS: repeat RVND and a light perturbation, restarting the counter whenever the best
inner solution improves, until `innerIterMax` iterations pass without improvement.
"""
function innerLoop!(solver::Solver, sol::Solution)
    algo = solver.algorithm
    it = 0
    copySolution!(algo.bestSol, sol)
    while it < algo.innerIterMax
        it += 1
        RVND!(solver, sol)
        if algo.aggressivePool
            registerRoutePool!(solver, sol)
        end
        if acceptSol(solver, sol, algo.bestSol)
            copySolution!(algo.bestSol, sol)
            it = 0
        end
        copySolution!(sol, algo.bestSol)
        innerPerturb!(solver, sol)
    end
    copySolution!(algo.outerCandidateSol, algo.bestSol)
end
