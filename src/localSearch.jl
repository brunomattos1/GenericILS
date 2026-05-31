
# function RVND!(solver::Solver, solution::Solution)
#     resize!(solver.auxNeighborhoods, length(solver.neighborhoods))
#     copyto!(solver.auxNeighborhoods, solver.neighborhoods)
#     while !isempty(solver.auxNeighborhoods)
#         shuffle!(solver.seed, solver.auxNeighborhoods)
#         improvement = false
        
#         for neigh in solver.auxNeighborhoods
#             improvement = search!(neigh, solver, solution)

#             updatePenalty(solver.parameters, solution)   
#             if improvement
#                 copyto!(solver.auxNeighborhoods, solver.neighborhoods)
#                 break
#             end
#         end
#         if !improvement
#             empty!(solver.auxNeighborhoods)
#         end
#     end
# end

function RVND!(solver::Solver, sol::Solution)

    neighs = solver.active_neighs
    src = solver.neighborhoods

    nsrc = length(src)

    resize!(neighs, nsrc)
    for i in 1:nsrc
        neighs[i] = i
    end

    n = nsrc

    while n > 0

        pos = rand(solver.seed, 1:n)
        idx = neighs[pos]

        neigh = src[idx]
        updatePenalty(solver.parameters, sol)
        improv = search!(neigh, solver, sol)

        if improv

            # reset RVND
            for i in 1:nsrc
                neighs[i] = i
            end
            n = nsrc

        else

            # remove vizinhança (O(1))
            neighs[pos] = neighs[n]
            n -= 1

        end

    end
end