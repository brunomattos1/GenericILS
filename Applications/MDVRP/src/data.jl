import Unicode

mutable struct Vertex_
    id_vertex::Int
    pos_x::Float64
    pos_y::Float64
    demand::Float64
    st::Float64
    a::Float64
    b::Float64
end

mutable struct InputGraph
    V′::Array{Vertex_}
    A::Array{Tuple{Int64,Int64}}
    cost::Dict{Tuple{Int64,Int64},Float64}
end

mutable struct DataMDVRP
    G′::InputGraph
    Q::Float64
    n::Int
    nb_depots::Int
    depot_ids::Array{Int,1}
    round::Bool
    tw::Bool
    name::String
end

function c(data::DataMDVRP, a::Tuple{Int64,Int64})
    if !(haskey(data.G′.cost, a))
        return 100000.0
    end
    return data.G′.cost[a]
end

function arcDistance(data::DataMDVRP, arc::Tuple{Int64,Int64})
    u, v = arc
    x_sq = (data.G′.V′[v+1].pos_x - data.G′.V′[u+1].pos_x)^2
    y_sq = (data.G′.V′[v+1].pos_y - data.G′.V′[u+1].pos_y)^2
    return data.round ? floor(sqrt(x_sq + y_sq) + 0.5) : sqrt(x_sq + y_sq)
end

contains(p, s) = findnext(s, p, 1) !== nothing

function readData(instance::String; round::Bool = false)
    str = Unicode.normalize(read(instance, String); stripcc=true)
    aux = split(str, [' ', ':', '\n']; limit=0, keepempty=false)

    G′   = InputGraph([], [], Dict())
    data = DataMDVRP(G′, 0.0, 0, 0, [], round, false, splitext(basename(instance))[1])

    dim = 0
    for i in eachindex(aux)
        if contains(aux[i], "TYPE")
            if aux[i+1] == "MDOVRPTW" || aux[i+1] == "OVRPTW"
                data.tw = true
            end
        elseif contains(aux[i], "DIMENSION")
            dim = parse(Int, aux[i+1])
        elseif contains(aux[i], "CAPACITY")
            data.Q = parse(Float64, aux[i+1])
        elseif contains(aux[i], "NODE_COORD_SECTION")
            j = i + 1
            while aux[j] != "DEMAND_SECTION"
                v = Vertex_(parse(Int, aux[j])-1, parse(Float64, aux[j+1]), parse(Float64, aux[j+2]), 0, 0, 0, 0)
                push!(G′.V′, v)
                j += 3
            end
        elseif contains(aux[i], "DEMAND_SECTION")
            j = i + 1
            for _ = 1:dim
                pos = parse(Int, aux[j])
                G′.V′[pos].demand = parse(Float64, aux[j+1])
                j += 2
            end
        elseif contains(aux[i], "SERVICE_TIME_SECTION")
            j = i + 1
            for _ = 1:dim
                pos = parse(Int, aux[j])
                G′.V′[pos].st = parse(Float64, aux[j+1])
                j += 2
            end
        elseif contains(aux[i], "TIME_WINDOWS_SECTION")
            j = i + 1
            for _ = 1:dim
                pos = parse(Int, aux[j])
                G′.V′[pos].a = parse(Float64, aux[j+1])
                G′.V′[pos].b = parse(Float64, aux[j+2])
                j += 3
            end
        elseif contains(aux[i], "DEPOT_SECTION")
            j = i + 1
            while aux[j] != "EOF" && aux[j] != "-1"
                depot_id = parse(Int, aux[j]) - 1
                push!(data.depot_ids, depot_id)
                data.nb_depots += 1
                j += 1
            end
        end
    end

    for v1 in G′.V′, v2 in G′.V′
        i, j = v1.id_vertex, v2.id_vertex
        (i in data.depot_ids && j in data.depot_ids) && continue
        if i != j
            push!(G′.A, (i, j))
            G′.cost[(i, j)] = arcDistance(data, (i, j))
        end
    end

    data.n = length(G′.V′) - data.nb_depots
    for i = 1:data.n
        G′.cost[(i + data.nb_depots - 1, i + data.nb_depots - 1)] = 0.0
    end

    return data
end

# Build (n+1)×(n+1) demand matrix: index 1 = virtual depot, 2..n+1 = customers.
# d[i,j] = demand of customer j-1 for all arcs arriving at customer j.
function buildDemandMatrix(data::DataMDVRP)
    n  = data.n
    m  = data.nb_depots
    d_ = zeros(Float64, n+1, n+1)
    for i = 1:n+1, j = 2:n+1
        i == j && continue
        d_[i, j] = data.G′.V′[j + m - 1].demand
    end
    return d_
end

# Build (n+1)×(n+1) cost matrix with virtual depot (index 1).
# Depot↔customer costs use the minimum over all physical depots.
function buildDistMatrix(data::DataMDVRP)
    n   = data.n
    m   = data.nb_depots
    D   = zeros(Float64, n+1, n+1)
    for j = 2:n+1
        vjId   = j + m - 2   # 0-indexed vertex id of customer j-1
        D[1,j] = minimum(data.G′.cost[(dk, vjId)] for dk in data.depot_ids)
        D[j,1] = minimum(data.G′.cost[(vjId, dk)] for dk in data.depot_ids)
    end
    for i = 2:n+1, j = 2:n+1
        i == j && continue
        D[i,j] = data.G′.cost[(i + m - 2, j + m - 2)]
    end
    return D
end
