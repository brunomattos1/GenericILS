"""Number shown in the progress table for the stopping criterion (temperature or iteration counter)."""
getStopInfo(algo::ILSAlgorithm, ::Any) = NaN
getStopInfo(algo::ILSAlgorithm, c::ByIterMax) = algo.iter
getStopInfo(algo::ILSAlgorithm, c::ByTemperature) = algo.acceptCriteria.temperature

"""Print one line of the progress table (a new header is printed every 10 seconds)."""
function printInfo(algo::ILSAlgorithm, solver::Solver)
    totalAlgorithmTime = time() - algo.startTime
    stopInfo = getStopInfo(algo, algo.stopCriteria)
    if mod(totalAlgorithmTime, 10.0) == 0
        println("-"^135)
        @printf("| %10s | %10s | %12s | %12s | %10s | %15s | %15s | %6s | %10s |\n",
            "Temp.", "Best Feas", "Best", "Candidate",
            "Pen. Custom", "Pen. Standard 1", "Pen. Standard 2", "Pool", "Time (s)")
        println("-"^135)
    end

    @printf("| %10.6f | %10.2f | %12.2f | %12.2f | %11.2f | %15.2f | %15.2f | %6d | %10.4f |\n",
        stopInfo,
        solver.bestFeasSol.cost, algo.bestSol.cost, algo.candidateSol.cost,
        solver.penaltyManager.penaltyCustom,
        solver.penaltyManager.penaltyStandard1, solver.penaltyManager.penaltyStandard2,
        length(algo.routeStorage), totalAlgorithmTime)
end

"""
Run the ILS: construct a solution and apply RVND, then repeat perturb + RVND + accept until the stopping
criterion (or the time limit) triggers; finish with Set Partitioning.
"""
function run!(algo::ILSAlgorithm, solver::Solver)
    assertGroupsSupported(solver.data)
    algo.bestSol.cost = Inf
    algo.startTime = time()
    solver.bestFeasSol = newSolution(solver)
    solver.bestFeasSol.cost = Inf

    algo.currSol = constructSol!(solver)
    registerRoutePool!(solver, algo.currSol)
    RVND!(solver, algo.currSol)
    copySolution!(algo.candidateSol, algo.currSol)

    accept!(AcceptBest(), solver, algo.bestSol, algo.currSol, algo.candidateSol)
    while !(stop(algo.stopCriteria, solver))
        algo.iter += 1
        copySolution!(algo.candidateSol, algo.currSol)
        perturb!(solver, algo.candidateSol)
        RVND!(solver, algo.candidateSol)
        updatePenalty(solver.penaltyManager, algo.candidateSol)
        registerRoutePool!(solver, algo.candidateSol)
        accept!(algo.acceptCriteria, solver, algo.bestSol, algo.currSol, algo.candidateSol)
        if totalTime(algo) >= algo.timeLimitILS
            @goto SP
        end
    end
    @label SP
    registerBestFeasibleBefSP!(solver)
    setPartitioning(solver, solver.bestFeasSol.cost)
    algo.bestSol = deepcopy(solver.bestFeasSol)
    registerPoolSize!(solver)
    registerTotalTime!(solver)
end
