"""Whether `currSol` is strictly better than `bestSol`."""
function acceptSol(solver::Solver, currSol::Solution, bestSol::Solution)
    if currSol.cost < bestSol.cost - 1e-6
        return true
    end
    return false
end

"""Publish `sol` as best feasible solution if it qualifies, recording statistics and resetting the counter."""
function updateBestFeasible!(solver::Solver, sol::Solution)
    if updateBestFeasSol!(solver, sol)
        registerBestFeasible!(solver, sol)
        solver.algorithm.iter = 0
    end
end

"""Accept every candidate."""
function accept!(criteria::RandomWalk, solver::Solver, bestSol::Solution, currSol::Solution, candidateSol::Solution)
    if currSol.cost < bestSol.cost - 1e-6
        copySolution!(bestSol, currSol)
        updateBestFeasible!(solver, currSol)
    elseif currSol.cost < solver.bestFeasSol.cost - 1e-6
        updateBestFeasible!(solver, currSol)
    end
end

"""Accept the candidate only if it improves the best solution."""
function accept!(criteria::AcceptBest, solver::Solver, bestSol::Solution, currSol::Solution, candidateSol::Solution)
    updateBestFeasible!(solver, candidateSol)
    if candidateSol.cost < bestSol.cost - 1e-6
        copySolution!(bestSol, candidateSol)
    end
end

"""Metropolis acceptance, then cool down: `temperature *= alpha`."""
function accept!(criteria::Metropolis, solver::Solver, bestSol::Solution, currSol::Solution, candidateSol::Solution)
    Δ = candidateSol.cost - currSol.cost
    updateBestFeasible!(solver, candidateSol)
    if candidateSol.cost < bestSol.cost - 1e-6
        copySolution!(bestSol, candidateSol)
    end
    if Δ < -1e-6
        copySolution!(currSol, candidateSol)
    else
        # Metropolis
        if rand(solver.seed) < exp(-Δ / criteria.temperature)
            copySolution!(currSol, candidateSol)
        end
    end
    criteria.temperature *= criteria.alpha # cool down
end

"""Metropolis acceptance with a temperature that decays with the elapsed time."""
function accept!(criteria::MetropolisTimed, solver::Solver, bestSol::Solution, currSol::Solution, candidateSol::Solution)
    Δ = candidateSol.cost - currSol.cost
    updateBestFeasible!(solver, candidateSol)
    if candidateSol.cost < bestSol.cost - 1e-6
        copySolution!(bestSol, candidateSol)
    end
    if Δ < -1e-6
        copySolution!(currSol, candidateSol)
    else
        # Metropolis
        if rand(solver.seed) < exp(-Δ / criteria.temperature)
            copySolution!(currSol, candidateSol)
        end
    end
    totalTime = time() - solver.algorithm.startTime
    criteria.temperature = criteria.initialTemperature*(1-totalTime/criteria.maxTime)^criteria.p
    if totalTime >= criteria.maxTime
        criteria.temperature = 0.0
    end
end

"""Iteration-driven Metropolis acceptance; freezes (temperature 0) after enough cold iterations."""
function accept!(criteria::MetropolisTimedIter, solver::Solver, currSol::Solution, bestSol::Solution)
    Δ = currSol.cost - bestSol.cost
    
    if Δ < -1e-6
        # melhorou -> aceita sempre
        copySolution!(bestSol, currSol)
    else
        # Metropolis
        if rand(solver.seed) < exp(-Δ / criteria.temperature)
            # accept a worse solution (diversification)
            copySolution!(bestSol, currSol)
        else
            # rejeita -> volta pro best
            copySolution!(currSol, bestSol)
        end
    end
    updateBestFeasible!(solver, bestSol)
    criteria.temperature *= criteria.p
    if criteria.temperature < 5.0
        criteria.iter += 1
        if criteria.iter > 100
            criteria.temperature = 0.0
        end
    end
end