function stop(criteria::ByGenerations, solver::Solver)
    return solver.algorithm.generation >= criteria.maxGenerations
end

function stop(criteria::ByTime, solver::Solver)
    return (time() - solver.algorithm.startTime) >= criteria.maxTime
end
