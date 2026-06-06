function stop(criteria::ByTime, solver::Solver)
    return (time() - solver.startTime) >= criteria.maxTime
end

function stop(criteria::ByTemperature, solver::Solver)
    return solver.acceptCriteria.temperature <= criteria.minTemp
end

function stop(criteria::ByIterMax, solver::Solver)
    return solver.iter >= criteria.maxIter
end