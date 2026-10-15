"""Whether the elapsed time reached the limit."""
function stop(criteria::ByTime, solver::Solver)
    return (time() - solver.algorithm.startTime) >= criteria.maxTime
end

"""Whether the accept criterion cooled down to the minimum temperature."""
function stop(criteria::ByTemperature, solver::Solver)
    return solver.algorithm.acceptCriteria.temperature <= criteria.minTemp
end

"""Whether the iteration counter reached its maximum."""
function stop(criteria::ByIterMax, solver::Solver)
    return solver.algorithm.iter >= criteria.maxIter
end
