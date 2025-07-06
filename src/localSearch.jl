
function RVND!(solver::Solver)
    neighborhoods = copy(solver.neighborhoods)
    computeLabels(solver)
    while length(neighborhoods) > 0
        neigh = rand(solver.seed, neighborhoods)
        # @show neighborhoods
        if neigh == 1
            improv = intraShift10!(solver)
            if improv
                neighborhoods = copy(solver.neighborhoods)
                computeLabels(solver)
            else
                setdiff!(neighborhoods, neigh)
            end
            # checkCVRP(solver, solver.currSol)
        end
        if neigh == 2
            improv = intraShift20!(solver)
            if improv
                neighborhoods = copy(solver.neighborhoods)
                computeLabels(solver)
            else
                setdiff!(neighborhoods, neigh)
            end
            # checkCVRP(solver, solver.currSol)
        end
        if neigh == 3
            # println("InterShift10")
            improv = interShift10!(solver)
            if improv
                neighborhoods = copy(solver.neighborhoods)
                computeLabels(solver)
            else
                setdiff!(neighborhoods, neigh)
            end
            # checkCVRP(solver, solver.currSol)
        end
        if neigh == 4
            improv = interShift20!(solver)
            if improv
                neighborhoods = copy(solver.neighborhoods)
                computeLabels(solver)
            else
                setdiff!(neighborhoods, neigh)
            end
            # checkCVRP(solver, solver.currSol)
        end
        if neigh == 5
            improv = interSwap11!(solver)
            if improv
                neighborhoods = copy(solver.neighborhoods)
                computeLabels(solver)
            else
                setdiff!(neighborhoods, neigh)
            end
            # checkCVRP(solver, solver.currSol)
        end
        if neigh == 6
            improv = interSwap22!(solver)
            if improv
                neighborhoods = copy(solver.neighborhoods)
                computeLabels(solver)
            else
                setdiff!(neighborhoods, neigh)
            end
            # checkCVRP(solver, solver.currSol)
        end
    end
end