
function RVND!(solver::Solver, solution::Solution)
    resize!(solver.auxNeighborhoods, length(solver.neighborhoods))
    copyto!(solver.auxNeighborhoods, solver.neighborhoods)
    while !isempty(solver.auxNeighborhoods)
        shuffle!(solver.seed, solver.auxNeighborhoods)
        improvement = false
        
        for neigh in solver.auxNeighborhoods
            if neigh == 1
                improvement = intraShift10!(solver, solution)
            elseif neigh == 2
                improvement = interShift10!(solver, solution)
            elseif neigh == 3
                improvement = interSwap11!(solver, solution)
            elseif neigh == 4
                improvement = twoOptStar!(solver, solution)
            elseif neigh == 5
                improvement = interShift20!(solver, solution)
            end
            updatePenalty(solver.parameters, solution)   
            if improvement
                copyto!(solver.auxNeighborhoods, solver.neighborhoods)
                break
            end
        end
        if !improvement
            empty!(solver.auxNeighborhoods)
        end
    end
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
#         updatePenalty(solver.parameters, solution)

#         if improv
#             neighborhoods = BitVector([true, true, true, true])
#         else
#             neighborhoods[neigh] = false
#         end
#     end
# end

