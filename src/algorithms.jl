
function NILS(solver::Solver)
    solver.outerBestSol.cost = Inf
    solver.bestSol = new_solution(solver)
    solver.startTime = time()
    solver.bestFeasSol = new_solution(solver)
    solver.bestFeasSol.cost = Inf
    println("-"^135)
    @printf("| %10s | %10s | %12s | %12s | %10s | %15s | %15s | %6s | %10s |\n",
        "Temp.",
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
    constructSol!(solver)
    push!(solver, solver.outerCurrSol)
    ILS(solver, solver.outerCurrSol)
    accept!(AcceptBest(), solver, solver.outerBestSol, solver.outerCurrSol, solver.outerCandidateSol)
    while !(stop(solver.stopCriteria, solver))
        solver.iter += 1
        copy_solution!(solver.outerCandidateSol, solver.outerCurrSol)
        outerPerturb!(solver, solver.outerCandidateSol)
        ILS(solver, solver.outerCandidateSol)
        push!(solver, solver.outerCandidateSol)
        accept!(solver.acceptCriteria, solver, solver.outerBestSol, solver.outerCurrSol, solver.outerCandidateSol)
        printInfo(solver)
        if totalTime(solver) >= solver.timeLimitILS
            @goto SP
        end
    end
    @label SP
    println("-"^144)
    if totalTime(solver) >= solver.timeLimitILS
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
        updatePenalty(solver.penaltyManager, sol)
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
    copy_solution!(solver.outerCandidateSol, solver.bestSol)
end