get_stop_info(algo::ILSAlgorithm, ::Any) = NaN  # default (não tem temperatura)
get_stop_info(algo::ILSAlgorithm, c::ByIterMax) = algo.iter
get_stop_info(algo::ILSAlgorithm, c::ByTemperature) = algo.acceptCriteria.temperature

function printInfo(algo::ILSAlgorithm, solver::Solver)
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
        solver.bestFeasSol.cost, algo.bestSol.cost, algo.candidateSol.cost,
        solver.penaltyManager.penaltyCustom,
        solver.penaltyManager.penaltyStandard1, solver.penaltyManager.penaltyStandard2,
        length(algo.route_storage), total_algorithm_time)
end

function run!(algo::ILSAlgorithm, solver::Solver)
    algo.bestSol.cost = Inf
    algo.startTime = time()
    solver.bestFeasSol = new_solution(solver)
    solver.bestFeasSol.cost = Inf

    algo.currSol = constructSol!(solver)
    registerRoutePool!(solver, algo.currSol)
    RVND!(solver, algo.currSol)
    copy_solution!(algo.candidateSol, algo.currSol)

    accept!(AcceptBest(), solver, algo.bestSol, algo.currSol, algo.candidateSol)
    while !(stop(algo.stopCriteria, solver))
        algo.iter += 1
        copy_solution!(algo.candidateSol, algo.currSol)
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
