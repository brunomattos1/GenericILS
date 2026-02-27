import Unicode

mutable struct vertex
    id_vertex::Int
    pos_x::Float64
    pos_y::Float64
    demand::Float64
    st::Float64 #service time duration 
    a::Float64 #Begining of the time windows
    b::Float64 #End of the time windows
end

# Undirected graph
mutable struct InputGraph
    V′::Array{vertex} # set of vertices (access with id_vertex + 1). The first vertices are the depot
    A::Array{Tuple{Int64,Int64}} # set of arcs involving any pair of vertices
    cost::Dict{Tuple{Int64,Int64},Float64} # cost for each arc
end

mutable struct DataMDVRP
    G′::InputGraph
    Q::Float64 #Vehicle capacity
    n::Int #Number of customers
    nb_depots::Int
    depot_ids::Array{Int,1}
    round::Bool # Is the distance matrix rounded?
    tw::Bool # Time windows?
    E::Array{Tuple{Int64,Int64}} # set of edges e = (i,j), i < j. 0 represents a generic depot and the remaining nodes represent customers
    CostEdge::Dict{Tuple{Int64,Int64},Float64} # cost for each ege e = (i,j) ∈ E. When i = 0, this cost is zero
    name::String
end

# Euclidian distance
function distance(data::DataMDVRP, arc::Tuple{Int64, Int64})
    u, v = arc
    x_sq = (data.G′.V′[v+1].pos_x - data.G′.V′[u+1].pos_x)^2
    y_sq = (data.G′.V′[v+1].pos_y - data.G′.V′[u+1].pos_y)^2
    if data.round
        return floor(sqrt(x_sq + y_sq) + 0.5)
    end
    return sqrt(x_sq + y_sq)
end

contains(p, s) = findnext(s, p, 1) != nothing



function readMDVRPData(app::Dict{String,Any})
    
    str = Unicode.normalize(read(app["instance"], String); stripcc=true)
    breaks_in = [' '; ':'; '\n']
    aux = split(str, breaks_in; limit=0, keepempty=false)
    
    G′ = InputGraph([],[],Dict())
    data = DataMDVRP(G′, 0,0,0, [], app["round"], false,[],Dict(), "teste")
    
    dim = 0
    for i in 1:length(aux)
        if contains(aux[i], "TYPE")
            if aux[i+1] == "MDOVRPTW" || aux[i+1] == "OVRPTW" 
                data.tw = true
            end
        end
        if contains(aux[i], "DIMENSION")
            dim = parse(Int, aux[i+1])
        elseif contains(aux[i], "CAPACITY")
            data.Q = parse(Float64, aux[i+1])
        elseif contains(aux[i], "NODE_COORD_SECTION")
            j = i+1
            while aux[j] != "DEMAND_SECTION" 
                v = vertex(0, 0, 0, 0,0,0,0)
                v.id_vertex = parse(Int, aux[j])-1 #id starts by 0
                v.pos_x = parse(Float64, aux[j+1])
                v.pos_y = parse(Float64, aux[j+2])
                push!(G′.V′, v) # add v in the vertex array
                j+=3
            end
        elseif contains(aux[i], "DEMAND_SECTION")
            j = i+1
            for k=1:dim
                pos = parse(Int, aux[j])
                G′.V′[pos].demand = parse(Float64, aux[j+1])
                j += 2
            end
        elseif contains(aux[i], "SERVICE_TIME_SECTION")
            j = i+1
            for k=1:dim
                pos = parse(Int, aux[j])
                G′.V′[pos].st = parse(Float64, aux[j+1])
                j += 2
            end
        elseif contains(aux[i], "TIME_WINDOWS_SECTION")
            j = i+1
            for k=1:dim
                pos = parse(Int, aux[j])
                G′.V′[pos].a = parse(Float64, aux[j+1])
                G′.V′[pos].b = parse(Float64, aux[j+2])
                j += 3
            end
        elseif contains(aux[i], "DEPOT_SECTION")
            j = i+1
            while aux[j] != "EOF" &&  aux[j] != "-1"
                depot_id = parse(Int, aux[j]) - 1
                push!(data.depot_ids, depot_id)
                data.nb_depots += 1
                j += 1
            end
        end
    end

    for v1 in G′.V′
        for v2 in G′.V′
            i, j = v1.id_vertex, v2.id_vertex
            if i in data.depot_ids && j in data.depot_ids
                    continue # Skip in order to prohibit edges between depots and from customers to depots
            end
            if i != j
                a = (i,j)
                push!(G′.A, a)
                data.G′.cost[a] = distance(data, a)  
            end
        end
    end
    nd = data.nb_depots
    data.n = length(data.G′.V′) - data.nb_depots
    nc = data.n
    for i=1:nc
        data.CostEdge[(i,i)] = 0.0
        data.G′.cost[(i+data.nb_depots-1,i+data.nb_depots-1)] = 0.0
    end
    if !app["VRPSolver"]
        for a ∈ data.G′.A
            if a[1] ∈ data.depot_ids
                e = (0,a[2] - nd + 1)
                if !(e ∈ data.E)
                    push!(data.E, e)
                    data.CostEdge[e] = 0.0
                    data.CostEdge[(e[2],e[1])] = 0.0
                end
            elseif a[2] ∈ data.depot_ids
                e = (0,a[1] - nd + 1)
                if !(e ∈ data.E)
                    push!(data.E, e)
                    data.CostEdge[e] = 0.0
                    data.CostEdge[(e[2],e[1])] = 0.0
                end
            else
                i = a[1] - nd + 1
                j = a[2] - nd + 1
                e = (i,j)
                if j < i
                    e = (j,i)
                end
                if !(e ∈ data.E)
                    push!(data.E, e)
                    data.CostEdge[e] = data.G′.cost[a]
                    data.CostEdge[(e[2],e[1])] = data.G′.cost[a]
                end
            end
        end
    else
        for a ∈ data.G′.A
            if a[1] < a[2]
                if !(a[1] ∈ data.depot_ids) || !(a[2] ∈ data.depot_ids)
                    push!(data.E, a)
                    data.CostEdge[a] = data.G′.cost[a]
                end
            end
        end
    end

    
    return data
end

arcs(data::DataMDVRP) = data.G′.A
c(data,a) = data.G′.cost[a] # cost of the arc a
d(data::DataMDVRP, i) = data.G′.V′[i+1].demand # return the demand of i
st(data::DataMDVRP, i) = data.G′.V′[i+1].st # return the service time duration of i
a(data::DataMDVRP, i) = data.G′.V′[i+1].a # return the Begining of the time windows of i
b(data::DataMDVRP, i) = data.G′.V′[i+1].b # return the End of the time windows of i
veh_capacity(data::DataMDVRP) = data.Q
nb_customers(data::DataMDVRP) = length(data.G′.V′) - data.nb_depots
customers(data::DataMDVRP) = [i.id_vertex for i in data.G′.V′[(data.nb_depots+1):end]]


# return incoming arcs of i
function δ⁻(data::DataMDVRP, i::Integer)
    incoming_arcs = Vector{Tuple}()
    for j in 0:(length(data.G′.V′)-1) 
        if j != i
            push!(incoming_arcs, (j, i)) 
        end
    end
    return incoming_arcs
end

# return outcoming arcs of i
function δ⁺(data::DataMDVRP, i::Integer)
    outcoming_arcs = Vector{Tuple}()
    for j in 0:(length(data.G′.V′)-1) 
        if j != i && (!(i in data.depot_ids) || !(j in data.depot_ids))
            push!(outcoming_arcs, (i, j)) 
        end
    end
    return outcoming_arcs
end

function δ(data,i)
    Adj = []
    for e ∈ data.E
        if e[1] == i || e[2] == i
            push!(Adj,e)
        end
    end
    return Adj
end
