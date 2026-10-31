"""
Read a Family Capacitated VRP instance (`.fcvrp`, Bernardino and Paias): TSPLIB header (`CAPACITY`, `VEHICLES`),
`NODE_COORD_SECTION` and `FAMILY_SECTION`, whose lines are `family nbNodes visits demand`. Node 1 is the depot and
the nodes of the families follow it consecutively, in the order of the section. Each visited node of a family
carries the demand of the family, and exactly `visits` nodes of the family must be visited.
Returns `(name, capacity, vehicles, coords, families)`, `families[f]` being `(nodes, visits, demand)` with
vertex ids (node number - 1).
"""
function readFCVRP(path::String)
    name, capacity, vehicles = "", 0.0, 0
    coords = Vector{Tuple{Float64, Float64}}()
    families = NamedTuple{(:nodes, :visits, :demand), Tuple{Vector{Int}, Int, Float64}}[]
    section = ""
    nextNode = 2
    for line in eachline(path)
        tokens = split(line)
        isempty(tokens) && continue
        if endswith(tokens[1], "SECTION")
            section = tokens[1]
        elseif section == "" && occursin(':', line)
            key, value = strip.(split(line, ':'; limit = 2))
            key == "NAME" && (name = value)
            key == "CAPACITY" && (capacity = parse(Float64, value))
            key == "VEHICLES" && (vehicles = parse(Int, value))
        elseif section == "NODE_COORD_SECTION"
            push!(coords, (parse(Float64, tokens[2]), parse(Float64, tokens[3])))
        elseif section == "FAMILY_SECTION"
            nbNodes = parse(Int, tokens[2])
            push!(families, (nodes = collect(nextNode - 1:nextNode + nbNodes - 2),
                             visits = parse(Int, tokens[3]), demand = parse(Float64, tokens[4])))
            nextNode += nbNodes
        end
    end
    nextNode - 1 == length(coords) || error("$path: the families hold $(nextNode - 2) nodes but there are $(length(coords) - 1) customers")
    return (; name, capacity, vehicles, coords, families)
end

"""Distance matrix (depot first) rounded to the nearest integer (halves up), as in the instances of the literature."""
function euclideanMatrix(coords::Vector{Tuple{Float64, Float64}})
    n = length(coords)
    return [floor(hypot(coords[i][1] - coords[j][1], coords[i][2] - coords[j][2]) + 0.5) for i in 1:n, j in 1:n]
end

"""One `Vertex` per customer, in the group of its family: `visits` of its vertices must be visited, in any routes."""
function buildVertices(families)
    vertices = Vertex[]
    for (f, family) in enumerate(families)
        group = Group(f, family.visits, false)
        append!(vertices, Vertex(id, group) for id in family.nodes)
    end
    return sort!(vertices; by = v -> v.id)
end

"""Demand of each vertex (depot first, demand 0): whichever vertex of a family is visited, it carries the family's demand."""
function vertexDemands(families, nbCustomers::Int)
    q = zeros(Float64, nbCustomers + 1)
    for family in families, id in family.nodes
        q[id + 1] = family.demand
    end
    return q
end
