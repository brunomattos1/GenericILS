"""
Read a GVRP instance in the format of Bektaş et al. (2011): TSPLIB header (`CAPACITY`, `VEHICLES`),
`NODE_COORD_SECTION`, `DEMAND_SECTION` and `MUTUALLY_EXCLUSIVE_GROUP_SECTION` (one line per cluster, exactly one
of its nodes must be visited). Node 1 is the depot. Returns `(name, capacity, vehicles, coords, demand, clusters)`
with `demand` indexed by node (depot first) and `clusters` holding vertex ids (node number - 1).
"""
function readGVRP(path::String)
    name, capacity, vehicles = "", 0.0, 0
    coords = Vector{Tuple{Float64, Float64}}()
    demand = Float64[]
    clusters = Vector{Vector{Int}}()
    section = ""
    for line in eachline(path)
        tokens = split(line)
        isempty(tokens) && continue
        if tokens[1] == "EOF"
            break
        elseif occursin(':', line) && !(tokens[1] in ("NODE_COORD_SECTION", "DEMAND_SECTION"))
            key, value = strip.(split(line, ':'; limit = 2))
            key == "NAME" && (name = value)
            key == "CAPACITY" && (capacity = parse(Float64, value))
            key == "VEHICLES" && (vehicles = parse(Int, value))
        elseif endswith(tokens[1], "SECTION")
            section = tokens[1]
        elseif section == "NODE_COORD_SECTION"
            push!(coords, (parse(Float64, tokens[2]), parse(Float64, tokens[3])))
        elseif section == "DEMAND_SECTION"
            push!(demand, parse(Float64, tokens[2]))
        elseif section == "MUTUALLY_EXCLUSIVE_GROUP_SECTION"
            push!(clusters, [parse(Int, t) - 1 for t in tokens[2:end]])
        end
    end
    return (; name, capacity, vehicles, coords, demand, clusters)
end

"""Distance matrix (depot first) rounded to the nearest integer, as `EUC_2D` of TSPLIB prescribes."""
function euclideanMatrix(coords::Vector{Tuple{Float64, Float64}})
    n = length(coords)
    return [round(hypot(coords[i][1] - coords[j][1], coords[i][2] - coords[j][2])) for i in 1:n, j in 1:n]
end

"""
One `Vertex` per customer, in the group of its cluster: the solution visits `numVisits` vertices of each
cluster (at most the cluster size), each visited vertex once. `numVisits = 1` is the generalized VRP.
"""
function buildVertices(clusters::Vector{Vector{Int}}, numVisits::Int)
    vertices = Vertex[]
    for (g, members) in enumerate(clusters)
        group = Group(g, min(numVisits, length(members)), false)
        append!(vertices, Vertex(id, group) for id in members)
    end
    return sort!(vertices; by = v -> v.id)
end
