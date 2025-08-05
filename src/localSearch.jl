
function RVND!(solver::Solver)
    neighborhoods = copy(solver.neighborhoods)
    # computeLabels(solver)
    improvs = 0
    while length(neighborhoods) > 0
        neigh = rand(solver.seed, neighborhoods)
        # println(improvs)
        # println("$neigh, $(solver.currSol.cost)")
        if neigh == 1
            improv = intraShift10!(solver)
            if improv
                neighborhoods = copy(solver.neighborhoods)
                # computeLabels(solver)
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
                # computeLabels(solver)
                improvs += 1
                # if improvs > 300
                #     sleep(1000)
                # end
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
                # computeLabels(solver)
                # improvs += 1
                # if improvs > 300
                #     @show solver.currSol
                #     printCVRP(solver, solver.currSol)
                #     for r = 1:length(solver.currSol.routes)
                #         currComputedInfeas, currAccDemand1F, currAccDemand1B= checkInfeasibles(solver, solver.currSol.routes[r])
                #         infeas = length(solver.currSol.routes[r]) -2 - max(solver.currSol.feasiblesF[r], solver.currSol.feasiblesB[r])
                #         println("infeas: $infeas")
                #         println("computed infeas: $currComputedInfeas")
                #         println("curr r1: $(solver.currSol.routes[r])")
                #         println("acum D r1 F: $(currAccDemand1F)")
                #         println("acum D r1 B: $(currAccDemand1B)")
                #         println()
                #     end
                #     sleep(1000)
                # end
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