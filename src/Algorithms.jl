# Shared machinery reused by outer-loop-style algorithms (NILS, ILS, ...):
# accept/stop criteria, perturbation, set partitioning.
include("Algorithms/NILS/AcceptCriteria.jl")
include("Algorithms/NILS/StopCriteria.jl")
include("Algorithms/NILS/Perturb.jl")
include("Algorithms/NILS/SetPartitioning.jl")

get_stop_info(algo::OuterLoopAlgorithm, ::Any) = NaN  # default (não tem temperatura)
get_stop_info(algo::OuterLoopAlgorithm, c::ByIterMax) = algo.iter
get_stop_info(algo::OuterLoopAlgorithm, c::ByTemperature) = algo.acceptCriteria.temperature

function printInfo(algo::OuterLoopAlgorithm, solver::Solver)
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
        solver.bestFeasSol.cost, algoBestSol(algo).cost, algoCandidateSol(algo).cost,
        solver.penaltyManager.penaltyCustom,
        solver.penaltyManager.penaltyStandard1, solver.penaltyManager.penaltyStandard2,
        length(algo.route_storage), total_algorithm_time)
end

include("Algorithms/NILS/ILS.jl")
include("Algorithms/NILS/NILS.jl")
include("Algorithms/ILS/ILS.jl")

run!(::NILSAlgorithm, solver::Solver) = NILS(solver)
run!(::ILSAlgorithm, solver::Solver) = ILS(solver)

solve!(solver::Solver) = run!(solver.algorithm, solver)
