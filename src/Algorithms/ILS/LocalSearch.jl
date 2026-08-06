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
