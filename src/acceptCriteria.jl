function acceptSol(solver::Solver)
    currInfeas = 0
    for r = 1:length(solver.currSol.routes)
        rt = solver.currSol.routes[r]
        currInfeas += length(rt.visits) - 2 - max(rt.feasibleF, rt.feasibleB)
    end
    bestInfeas = 0
    for r = 1:length(solver.bestSol.routes)
        rt = solver.bestSol.routes[r]
        bestInfeas += length(rt.visits) - 2 - max(rt.feasibleF, rt.feasibleB)
    end
    if currInfeas < bestInfeas
        return true
    end
    if currInfeas == bestInfeas && solver.currSol.cost < solver.bestSol.cost - 1e-5
        return true
    end
    return false
end

function acceptSol(solver::Solver, currSol::Solution, bestSol::Solution)
    if currSol.cost < bestSol.cost - 1e-6
        return true
    end
    return false
end

##############################
function updateBestFeasible!(solver::Solver, sol::Solution)
    if sol.cost < solver.bestFeasSol.cost - 1e-6
        if sol.totalInfeas == 0 && sol.totalWarpStd1 <= 1e-6 && sol.totalWarpStd1 <= 1e-6
            copy_solution!(solver.bestFeasSol, sol)
            solver.iter = 0 
        end
    end
end

function accept!(criteria::RandomWalk, solver::Solver, bestSol::Solution, currSol::Solution, candidateSol::Solution)
    if currSol.cost < bestSol.cost - 1e-6
        copy_solution!(bestSol, currSol)
        updateBestFeasible!(solver, currSol)
    elseif currSol.cost < solver.bestFeasSol.cost - 1e-6
        updateBestFeasible!(solver, currSol)
    end
end

function accept!(criteria::AcceptBest, solver::Solver, bestSol::Solution, currSol::Solution, candidateSol::Solution)
    updateBestFeasible!(solver, candidateSol)
    if candidateSol.cost < solver.outerBestSol.cost - 1e-6
        copy_solution!(bestSol, candidateSol)
    end
end

function accept!(criteria::Metropolis, solver::Solver, bestSol::Solution, currSol::Solution, candidateSol::Solution)
    Δ = candidateSol.cost - currSol.cost
    updateBestFeasible!(solver, candidateSol)
    if candidateSol.cost < solver.outerBestSol.cost - 1e-6
        copy_solution!(solver.outerBestSol, candidateSol)
    end
    if Δ < -1e-6
        copy_solution!(currSol, candidateSol)
    else
        # Metropolis
        if rand(solver.seed) < exp(-Δ / criteria.temperature)
            copy_solution!(currSol, candidateSol)
        end
    end
    criteria.temperature *= criteria.alpha # Atualiza a temperatura do critério
end

function accept!(criteria::MetropolisTimed, solver::Solver, bestSol::Solution, currSol::Solution, candidateSol::Solution)
    Δ = candidateSol.cost - currSol.cost
    updateBestFeasible!(solver, candidateSol)
    if candidateSol.cost < bestSol.cost - 1e-6
        copy_solution!(bestSol, candidateSol)
    end
    if Δ < -1e-6
        # melhorou -> aceita sempre
        copy_solution!(currSol, candidateSol)
    else
        # Metropolis
        if rand(solver.seed) < exp(-Δ / criteria.temperature)
            # aceita pior solução (diversificação)
            copy_solution!(currSol, candidateSol)
        else
            # rejeita -> volta pro best
            copy_solution!(candidateSol, currSol)
        end
    end
    totalTime = time() - solver.startTime
    criteria.temperature = criteria.initialTemperature*(1-totalTime/criteria.maxTime)^criteria.p
    if totalTime >= criteria.maxTime
        criteria.temperature = 0.0
    end
end

function accept!(criteria::MetropolisTimedIter, solver::Solver, currSol::Solution, bestSol::Solution)
    Δ = currSol.cost - bestSol.cost
    
    if Δ < -1e-6
        # melhorou -> aceita sempre
        copy_solution!(bestSol, currSol)
    else
        # Metropolis
        if rand(solver.seed) < exp(-Δ / criteria.temperature)
            # aceita pior solução (diversificação)
            copy_solution!(bestSol, currSol)
        else
            # rejeita -> volta pro best
            copy_solution!(currSol, bestSol)
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