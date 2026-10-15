import Unicode

"""A vertex (depot or customer) read from the instance file."""
mutable struct InputVertex
    idVertex::Int
    posX::Float64
    posY::Float64
    demand::Float64
end

"""Vertices of the instance."""
mutable struct InputGraph
    V′::Array{InputVertex}
    E::Array{Tuple{Int64,Int64}}
    D::Array{Int64}
    cost::Dict{Tuple{Int64,Int64},Float64}
end

"""Instance data of the risk-constrained VRP."""
mutable struct DataRiskVRP
    G′::InputGraph
    T::Int64       # risk threshold
    n::Int64       # number of customers
    depotId::Int
    coord::Bool
    name::String
end

getCustomers(data::DataRiskVRP) = [i.idVertex for i in data.G′.V′[2:end]]

contains(p, s) = findnext(s, p, 1) != nothing

"""Distance of `arc`."""
function arcDistance(data::DataRiskVRP, arc::Tuple{Int64,Int64})
    e = (arc[1] < arc[2]) ? arc : (arc[2], arc[1])
    if haskey(data.G′.cost, e)
        return data.G′.cost[e]
    elseif data.coord
        u, v = arc
        verts = data.G′.V′
        xSq = (verts[v+1].posX - verts[u+1].posX)^2
        ySq = (verts[v+1].posY - verts[u+1].posY)^2
        return sqrt(xSq + ySq)
    else
        return 0.0
    end
end

"""Read an instance file into a `DataRiskVRP`."""
function readData(instance::String)
    demandsSum = 0
    str  = Unicode.normalize(read(instance, String); stripcc=true)
    aux  = split(str, [' ', ':', '\n']; limit=0, keepempty=false)

    G′   = InputGraph([], [], [], Dict())
    data = DataRiskVRP(G′, 0, 0, 0, false, splitext(basename(instance))[1])

    dim = 0
    for i in 1:length(aux)
        if contains(aux[i], "DIMENSION")
            dim = parse(Int, aux[i+1])
        elseif contains(aux[i], "EDGE_WEIGHT_SECTION")
            if !data.coord
                for j = 1:dim
                    v = InputVertex(j-1, 0, 0, 0)
                    push!(G′.V′, v)
                end
            end
        elseif contains(aux[i], "RISK_THRESHOLD")
            data.T = parse(Int64, aux[i+1])
        elseif contains(aux[i], "NODE_COORD_SECTION")
            data.coord = true
            j = i + 1
            while aux[j] != "DEMAND_SECTION"
                v = InputVertex(parse(Int, aux[j])-1, parse(Float64, aux[j+1]), parse(Float64, aux[j+2]), 0)
                push!(G′.V′, v)
                j += 3
            end
        elseif contains(aux[i], "DEMAND_SECTION")
            j = i + 1
            while aux[j] != "DEPOT_SECTION"
                pos = parse(Int, aux[j])
                G′.V′[pos].demand = parse(Float64, aux[j+1])
                j += 2
            end
            data.depotId = 0
            break
        end
    end

    Lista    = Array{Int,1}(undef, dim+1)
    Lista[1] = data.T
    for k = 1:dim
        Lista[k+1] = Int(G′.V′[k].demand)
    end
    # g        = gcd(Lista)
    # data.T   = data.T / g
    # for i = 1:dim
    #     G′.V′[i].demand = G′.V′[i].demand / g
    #     demandsSum    += G′.V′[i].demand
    # end

    if data.coord
        for i in getCustomers(data)
            for j in getCustomers(data)
                i == j && continue
                e = i < j ? (i, j) : (j, i)
                haskey(G′.cost, e) || (G′.cost[e] = arcDistance(data, e))
                push!(G′.E, (i, j))
                G′.cost[(i,j)] = arcDistance(data, (i,j))
            end
            for ep in [(data.depotId, i), (i, data.depotId)]
                push!(G′.E, ep)
                G′.cost[ep] = arcDistance(data, ep)
            end
        end
    else
        for i in 1:length(aux)
            if contains(aux[i], "EDGE_WEIGHT_SECTION")
                j = i + 1
                for k = 0:dim-1, m = 0:dim-1
                    if k != m
                        e = (k, m)
                        push!(G′.E, e)
                        G′.cost[e] = parse(Float64, aux[j])
                    end
                    j += 1
                end
            end
        end
    end

    for i = 0:demandsSum
        push!(G′.D, i)
    end
    data.n = length(G′.V′) - 1
    return data
end

"""Distance matrix (index = vertex id + 1)."""
function buildDistMatrix(data::DataRiskVRP)
    n = length(data.G′.V′)
    D = zeros(Float64, n, n)
    for i = 1:n, j = 1:n
        i == j && continue
        e    = i < j ? (i-1, j-1) : (j-1, i-1)
        D[i,j] = data.G′.cost[e]
    end
    return D
end

"""Matrix with the demand of the tail vertex of each arc."""
function createDtDemands(data::DataRiskVRP)
    n = length(data.G′.V′)
    d = zeros(Float64, n, n)
    for i in 1:n, j in 1:n
        if i != j && i != 1
            d[i, j] = data.G′.V′[i].demand
        end
    end
    return d
end
