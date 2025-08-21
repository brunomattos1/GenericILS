
function RVND!(solver::Solver)
    neighborhoods = copy(solver.neighborhoods)
    while length(neighborhoods) > 0
        neigh = rand(solver.seed, neighborhoods)
        if neigh == 1
            improv = intraShift10!(solver)
            if improv
                neighborhoods = copy(solver.neighborhoods)
            else
                setdiff!(neighborhoods, neigh)
            end
        end
        if neigh == 2
            improv = intraShift20!(solver)
            if improv
                neighborhoods = copy(solver.neighborhoods)
            else
                setdiff!(neighborhoods, neigh)
            end
        end
        if neigh == 3
            improv = interShift10!(solver)
            if improv
                neighborhoods = copy(solver.neighborhoods)
            else
                setdiff!(neighborhoods, neigh)
            end
        end
        if neigh == 4
            improv = interShift20!(solver)
            if improv
                neighborhoods = copy(solver.neighborhoods)
            else
                setdiff!(neighborhoods, neigh)
            end
        end
        if neigh == 5
            improv = interSwap11!(solver)
            if improv
                neighborhoods = copy(solver.neighborhoods)
            else
                setdiff!(neighborhoods, neigh)
            end
        end
        if neigh == 6
            improv = interSwap22!(solver)
            if improv
                neighborhoods = copy(solver.neighborhoods)
            else
                setdiff!(neighborhoods, neigh)
            end
        end
        if neigh == 7
            improv = twoOptStar!(solver)
            if improv
                neighborhoods = copy(solver.neighborhoods)
            else
                setdiff!(neighborhoods, neigh)
            end
        end
        if neigh == 8
            improv = intraSwap11!(solver)
            if improv
                neighborhoods = copy(solver.neighborhoods)
            else
                setdiff!(neighborhoods, neigh)
            end
        end
        if neigh == 9
            improv = twoOpt!(solver)
            if improv
                neighborhoods = copy(solver.neighborhoods)
            else
                setdiff!(neighborhoods, neigh)
            end
        end
    end
end