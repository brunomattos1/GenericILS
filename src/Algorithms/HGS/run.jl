function initializePopulation!(solver::Solver, algo::HGSAlgorithm, numIndividuals::Int)
    for _ in 1:numIndividuals
        randperm!(solver.seed, algo.child.giantTour)
        splitGiantTour!(solver, algo.ws, algo.child.giantTour)
        rebuildRouteIndex!(solver, algo)
        RVND!(solver, algo.ws)
        registerRoutePool!(solver, algo.ws)
        updateBestFeasSol!(solver, algo.ws)

        newInd = getPooledIndividual!(algo.population, solver)
        resize!(newInd.giantTour, length(algo.child.giantTour))
        copyto!(newInd.giantTour, algo.child.giantTour)
        storeIndividual!(newInd, algo.ws)
        addIndividual!(algo.population, newInd)
    end
    return solver
end

function printInfo(algo::HGSAlgorithm, solver::Solver)
    total_algorithm_time = time() - algo.startTime
    if mod(total_algorithm_time, 10.0) == 0
        println("-"^120)
        @printf("| %10s | %10s | %8s | %8s | %8s | %15s | %15s | %10s |\n",
            "Gen", "Best Feas", "nFeas", "nInfeas", "Child", "Pen. Custom", "Pen. Standard 1", "Time (s)")
        println("-"^120)
    end

    pop = algo.population
    @printf("| %10d | %10.2f | %8d | %8d | %8.2f | %15.2f | %15.2f | %10.4f |\n",
        algo.generation, solver.bestFeasSol.cost,
        pop === nothing ? 0 : length(pop.feasible), pop === nothing ? 0 : length(pop.infeasible),
        algo.ws === nothing ? NaN : algo.ws.cost,
        solver.penaltyManager.penaltyCustom, solver.penaltyManager.penaltyStandard1,
        total_algorithm_time)
end

function run!(algo::HGSAlgorithm, solver::Solver)
    algo.startTime = time()
    algo.generation = 0
    algo.iterationsSinceImprovement = 0
    solver.bestFeasSol = new_solution(solver)
    solver.bestFeasSol.cost = Inf

    n = length(solver.data.vertices)
    resize!(algo.visited, n)
    fill!(algo.visited, false)
    resize!(algo.freeSlots, n)
    resize!(algo.customerRoute, n)
    resize!(algo.customerPos, n)
    resize!(algo.whenLastTested, n)
    fill!(algo.whenLastTested, 0)
    resize!(algo.indices, n)
    copyto!(algo.indices, 1:n)
    algo.granularNeighbors = buildGranularNeighbors!(solver, algo.granularK)

    algo.population = Population(algo.muMax, algo.lambda; nClosest = algo.nClosest, nElite = algo.nElite)
    algo.child = Individual(solver)
    algo.ws = new_solution(solver)

    initializePopulation!(solver, algo, 4 * algo.muMax)

    while !(stop(algo.stopCriteria, solver))
        algo.generation += 1

        parent1 = select(BinaryTournament(), algo.population, solver.seed)
        parent2 = select(BinaryTournament(), algo.population, solver.seed)
        orderCrossover!(algo.child, parent1, parent2, algo.visited, algo.freeSlots, solver.seed)

        splitGiantTour!(solver, algo.ws, algo.child.giantTour)
        rebuildRouteIndex!(solver, algo)
        RVND!(solver, algo.ws)
        registerRoutePool!(solver, algo.ws)
        updatePenalty(solver.penaltyManager, algo.ws)

        improved = updateBestFeasSol!(solver, algo.ws)

        newInd = getPooledIndividual!(algo.population, solver)
        resize!(newInd.giantTour, length(algo.child.giantTour))
        copyto!(newInd.giantTour, algo.child.giantTour)
        storeIndividual!(newInd, algo.ws)
        addIndividual!(algo.population, newInd)

        if improved
            registerBestFeasible!(solver, solver.bestFeasSol)
            algo.iterationsSinceImprovement = 0
        else
            algo.iterationsSinceImprovement += 1
        end
        printInfo(algo, solver)
        if algo.iterationsSinceImprovement >= algo.nbIterNonProd
            restart!(algo.population)
            initializePopulation!(solver, algo, 4 * algo.muMax)
            algo.iterationsSinceImprovement = 0
        end
    end

    registerBestFeasibleBefSP!(solver)
    setPartitioning(solver, solver.bestFeasSol.cost)
    registerPoolSize!(solver)
    registerTotalTime!(solver)
end
