# Per-customer candidate lists (k nearest other customers by raw arc distance),
# built once per run -- restricts the granular RVND! (LocalSearch.jl) to
# plausible move partners instead of scanning every route pair.
function buildGranularNeighbors!(solver::Solver, k::Int)
    n = length(solver.data.vertices)
    costMatrix = solver.data.costMatrix
    neighbors = [Int[] for _ in 1:n]
    order = Vector{Int}(undef, n - 1)
    for c in 1:n
        m = 0
        for j in 1:n
            j == c && continue
            m += 1
            order[m] = j
        end
        kc = min(k, m)
        partialsort!(view(order, 1:m), 1:kc; by = j -> costMatrix[c + 1, j + 1])
        neighbors[c] = order[1:kc]
    end
    return neighbors
end
