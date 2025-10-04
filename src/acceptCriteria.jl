function acceptSol(solver::Solver)
    currInfeas = 0
    for r = 1:length(solver.currSol.routes)
        currInfeas += length(solver.currSol.routes[r]) -2 - max(solver.currSol.feasiblesF[r], solver.currSol.feasiblesB[r])
    end
    bestInfeas = 0
    for r = 1:length(solver.bestSol.routes)
        bestInfeas += length(solver.bestSol.routes[r]) -2 - max(solver.bestSol.feasiblesF[r], solver.bestSol.feasiblesB[r])
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