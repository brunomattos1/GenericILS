"""
Read a PCVRP instance: the first line holds the number of vehicles `m`, the capacity `C` and the quota `Q` of demand
that has to be satisfied; then the depot (`id x y`) and one line `id x y demand prize` per customer. Returns
`(name, vehicles, capacity, quota, coords, demand, prize)`; `coords`, `demand` and `prize` are indexed by vertex id
plus one (the depot first, with demand and prize 0).
"""
function readPCVRP(path::String)
    lines = [split(l) for l in eachline(path) if !isempty(strip(l))]
    vehicles, capacity, quota = parse(Int, lines[1][1]), parse(Float64, lines[1][2]), parse(Float64, lines[1][3])
    n = length(lines) - 2
    coords = Vector{Tuple{Float64, Float64}}(undef, n + 1)
    demand = zeros(Float64, n + 1)
    prize = zeros(Float64, n + 1)
    coords[1] = (parse(Float64, lines[2][2]), parse(Float64, lines[2][3]))
    for k in 1:n
        t = lines[k + 2]
        id = parse(Int, t[1])
        id == k || error("$path: customers must be numbered 1:n, found $id at line $(k + 2)")
        coords[id + 1] = (parse(Float64, t[2]), parse(Float64, t[3]))
        demand[id + 1] = parse(Float64, t[4])
        prize[id + 1] = parse(Float64, t[5])
    end
    return (; name = splitext(basename(path))[1], vehicles, capacity, quota, coords, demand, prize)
end

"""Euclidean distances between the points, indexed by vertex id + 1."""
function distanceMatrix(pc)
    n = length(pc.coords)
    return [hypot(pc.coords[i][1] - pc.coords[j][1], pc.coords[i][2] - pc.coords[j][2]) for i in 1:n, j in 1:n]
end

"""
Cost of each arc: its distance minus the prize of the customer it enters. The cost of a route is its distance minus
the prizes of the customers it visits, which is what the solver minimizes (the profit is its opposite).
"""
function arcCosts(pc, dist::Matrix{Float64})
    c = copy(dist)
    for j in 2:length(pc.coords)
        c[:, j] .-= pc.prize[j]
    end
    return c
end
