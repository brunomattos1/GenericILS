function acceptSol(solver::Solver)
    if sum(solver.currSol.feasiblesF) + sum(solver.currSol.feasiblesB) > sum(solver.bestSol.feasiblesF) + sum(solver.bestSol.feasiblesB)
        return true
    end
    if sum(solver.currSol.feasiblesF) + sum(solver.currSol.feasiblesB) == sum(solver.bestSol.feasiblesF) + sum(solver.bestSol.feasiblesB) && solver.currSol.cost < solver.bestSol.cost - 1e-5
        return true
    end
    return false
end