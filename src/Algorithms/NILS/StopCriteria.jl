function stop(criteria::ByTime, solver::Solver)
    return (time() - solver.algorithm.startTime) >= criteria.maxTime
end

function stop(criteria::ByTemperature, solver::Solver)
    return solver.algorithm.acceptCriteria.temperature <= criteria.minTemp
end

function stop(criteria::ByIterMax, solver::Solver)
    return solver.algorithm.iter >= criteria.maxIter
end
