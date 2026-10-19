"""
Read a Team Orienteering Problem instance (Chao, Golden and Wasil): `n` customers, `m` paths, the maximum length
`Tmax` of a path and then `n + 2` lines `x y profit`, the first being the start point and the last the end point of
the paths (they may differ), both with profit 0. Returns `(name, n, vehicles, tmax, coords, profit)`; `coords[1]`
is the start, `coords[2:n+1]` the customers and `coords[n+2]` the end, and `profit[i + 1]` is the profit of customer
`i` (so the start has index 1 and profit 0, like the depot in the other applications).
"""
function readTOP(path::String)
    tokens = split(read(path, String))
    n = parse(Int, tokens[1])
    vehicles = parse(Int, tokens[2])
    tmax = parse(Float64, tokens[3])
    coords = Vector{Tuple{Float64, Float64}}(undef, n + 2)
    profit = zeros(Float64, n + 1)
    for i in 0:n+1
        offset = 3 + 3i
        coords[i + 1] = (parse(Float64, tokens[offset + 1]), parse(Float64, tokens[offset + 2]))
        0 < i <= n && (profit[i + 1] = parse(Float64, tokens[offset + 3]))
    end
    return (; name = splitext(basename(path))[1], n, vehicles, tmax, coords, profit)
end

"""
Travel distances between the points of the instance, as a `(n + 1) x (n + 1)` matrix indexed by vertex id + 1. The
depot is both the start and the end of the paths: its row holds the distances from the start and its column the
distances to the end, which is why the matrix is not symmetric. The distance of the empty path is 0.
"""
function travelMatrix(top)
    n = top.n
    dist(a, b) = hypot(top.coords[a][1] - top.coords[b][1], top.coords[a][2] - top.coords[b][2])
    d = zeros(Float64, n + 1, n + 1)
    for i in 1:n
        d[1, i + 1] = dist(1, i + 1)               # start -> customer i
        d[i + 1, 1] = dist(i + 1, n + 2)           # customer i -> end
        for j in 1:n
            i != j && (d[i + 1, j + 1] = dist(i + 1, j + 1))
        end
    end
    return d
end

"""
Weight of the distance in the objective. The objective is `-profit + weight * distance`: the profit is what
counts, and the small distance term rewards shorter paths, which leave room for more profit (without it, moving
customers between paths would never change the objective). It is small enough for the whole distance of a
feasible solution to weigh less than one unit of profit.
"""
distanceWeight(top) = 0.1 / (top.vehicles * top.tmax)

"""
Cost of each arc, as `distanceWeight * distance - profit` of the customer it enters: the cost of a path is its
(weighted) distance minus the profit of the customers it visits.
"""
function arcCosts(top, d::Matrix{Float64})
    c = distanceWeight(top) .* d
    for j in 1:top.n
        c[:, j + 1] .-= top.profit[j + 1]
    end
    return c
end
