
function RVND!(solver::Solver, solution::Solution)
    neighborhoods = copy(solver.neighborhoods)
    while length(neighborhoods) > 0
        neigh = rand(solver.seed, neighborhoods)
        improvement = false
        if neigh == 1
            improvement = intraShift10!(solver, solution)
        end
        if neigh == 2
            improvement = interShift10!(solver, solution)
        end
        if neigh == 3
            improvement = interSwap11!(solver, solution)
        end
        if neigh == 4
            improvement = twoOptStar!(solver, solution)
        end
        if improvement
            neighborhoods = copy(solver.neighborhoods)
        else
            delete!(neighborhoods, neigh)
        end
    end
    updatePenalty(solver.params, solution)
end


# function RVND!(solver::Solver, solution::Solution)
#     neighborhoods = BitVector([true, true, true, true])

#     while any(neighborhoods)
#         improv = false
#         actives = findall(neighborhoods)
#         neigh = rand(solver.seed, actives)
#         if neigh == 1
#             improv = intraShift10!(solver, solution)
#         elseif neigh == 2
#             improv = interShift10!(solver, solution)
#         elseif neigh == 3
#             improv = interSwap11!(solver, solution)
#         elseif neigh == 4
#             improv = twoOptStar!(solver, solution)
#         end

#         if improv
#             neighborhoods = BitVector([true, true, true, true])
#         else
#             neighborhoods[neigh] = false
#         end
#     end
# end

