mutable struct Vertex_
    id_vertex::Int
    p::Float64
    d::Float64
end

mutable struct InputGraph
    V′::Array{Vertex_}
    E::Array{Tuple{Int64,Int64}}
    cost::Dict{Tuple{Int64,Int64},Float64}
end

mutable struct DataVRPSPD
    G′::InputGraph
    Q::Float64
    n::Int64
    name::String
end

function readData(instance::String)
    str  = read(instance, String)
    aux  = split(str, [' ', '\n', '\t']; limit=0, keepempty=false)

    G′   = InputGraph([], [], Dict())
    data = DataVRPSPD(G′, 0.0, 0, splitext(basename(instance))[1])

    data.Q = parse(Float64, aux[1])
    data.n = parse(Int,     aux[2])
    n      = data.n

    pos = 3
    for i = 0:n, j = 0:n
        if i < j
            push!(G′.E, (i, j))
            G′.cost[(i,j)] = parse(Float64, aux[pos])
        end
        pos += 1
    end

    push!(G′.V′, Vertex_(0, 0.0, 0.0))
    for i = 1:n
        d = parse(Float64, aux[pos])
        p = parse(Float64, aux[pos+1])
        push!(G′.V′, Vertex_(i, d, p))
        pos += 2
    end

    return data
end

function buildDistMatrix(data::DataVRPSPD)
    n = length(data.G′.V′)
    D = zeros(Float64, n, n)
    for i = 1:n, j = 1:n
        i == j && continue
        e      = i < j ? (i-1, j-1) : (j-1, i-1)
        D[i,j] = data.G′.cost[e]
    end
    return D
end

function createPdDemands(data::DataVRPSPD)
    n = length(data.G′.V′)
    p = zeros(Float64, n, n)
    d = zeros(Float64, n, n)
    for i in 1:n, j in 1:n
        if i != j && j != 1
            p[i,j] = data.G′.V′[j].p
            d[i,j] = data.G′.V′[j].d
        end
    end
    return p, d
end
