get_stop_info(algo::NILSAlgorithm, ::Any) = NaN  # default (não tem temperatura)
get_stop_info(algo::NILSAlgorithm, c::ByIterMax) = algo.iter
get_stop_info(algo::NILSAlgorithm, c::ByTemperature) = algo.acceptCriteria.temperature

function printInfo(algo::NILSAlgorithm, solver::Solver)
    total_algorithm_time = time() - algo.startTime
    stopInfo = get_stop_info(algo, algo.stopCriteria)
    if mod(total_algorithm_time, 10.0) == 0
        println("-"^135)
        @printf("| %10s | %10s | %12s | %12s | %10s | %15s | %15s | %6s | %10s |\n",
            "Temp.", "Best Feas", "Best", "Candidate",
            "Pen. Custom", "Pen. Standard 1", "Pen. Standard 2", "Pool", "Time (s)")
        println("-"^135)
    end

    @printf("| %10.6f | %10.2f | %12.2f | %12.2f | %11.2f | %15.2f | %15.2f | %6d | %10.4f |\n",
        stopInfo,
        solver.bestFeasSol.cost, algo.outerBestSol.cost, algo.outerCandidateSol.cost,
        solver.penaltyManager.penaltyCustom,
        solver.penaltyManager.penaltyStandard1, solver.penaltyManager.penaltyStandard2,
        length(algo.route_storage), total_algorithm_time)
end

function run!(algo::NILSAlgorithm, solver::Solver)
    algo.outerBestSol.cost = Inf
    algo.bestSol = new_solution(solver)
    algo.startTime = time()
    solver.bestFeasSol = new_solution(solver)
    solver.bestFeasSol.cost = Inf

    algo.outerCurrSol = constructSol!(solver)
    registerRoutePool!(solver, algo.outerCurrSol)
    innerLoop!(solver, algo.outerCurrSol)

    accept!(AcceptBest(), solver, algo.outerBestSol, algo.outerCurrSol, algo.outerCandidateSol)
    while !(stop(algo.stopCriteria, solver))
        algo.iter += 1
        copy_solution!(algo.outerCandidateSol, algo.outerCurrSol)
        outerPerturb!(solver, algo.outerCandidateSol)
        innerLoop!(solver, algo.outerCandidateSol)
        updatePenalty(solver.penaltyManager, algo.outerCandidateSol)
        registerRoutePool!(solver, algo.outerCandidateSol)
        accept!(algo.acceptCriteria, solver, algo.outerBestSol, algo.outerCurrSol, algo.outerCandidateSol)
        if totalTime(algo) >= algo.timeLimitILS
            @goto SP
        end
    end
    @label SP
    registerBestFeasibleBefSP!(solver)
    setPartitioning(solver, solver.bestFeasSol.cost)
    algo.outerBestSol = deepcopy(solver.bestFeasSol)
    registerPoolSize!(solver)
    registerTotalTime!(solver)
end
