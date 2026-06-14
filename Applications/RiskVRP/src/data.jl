include("../../../src/include.jl")
import Unicode

# Random.seed!(0)  # inicializa o GLOBAL_RNG (se precisar)
# ENV["JULIA_HASH_SEED"] = "0"
mutable struct VertexRisk
   id_vertex::Int
   pos_x::Float64
   pos_y::Float64
   demand::Float64
end

# Undirected graph
mutable struct InputGraph
   V′::Array{VertexRisk} # set of vertices (access with id_vertex + 1)
   E::Array{Tuple{Int64,Int64}} # set of edges
   D::Array{Int64} #conjunto discreto dos possíveis valores monetários
   cost::Dict{Tuple{Int64,Int64},Float64} # cost for each edge
end

mutable struct DataCVRP
   G′::InputGraph
   T::Int64 # RISK THRESHOLD
   n::Int64 #number of customers
   depot_id::Int
   coord::Bool # instance with NODE_COORD_SECTION
   round::Bool # Is the distance matrix rounded?
   name::String
end





customers(data::DataCVRP) = [i.id_vertex for i in data.G′.V′[2:end]] # return set of customers

function distance(data::DataCVRP, arc::Tuple{Int64, Int64})
   e = (arc[1] < arc[2]) ? arc : (arc[2],arc[1])
   if haskey(data.G′.cost, e) # use already calculated value
      return data.G′.cost[e]
   elseif data.coord
      u, v = arc
      vertices = data.G′.V′
      # array <vertices> is indexed from 1 (depot is vertices[1], customer 1 is vertices[2], and so on)
      x_sq = (vertices[v+1].pos_x - vertices[u+1].pos_x)^2
      y_sq = (vertices[v+1].pos_y - vertices[u+1].pos_y)^2
      return sqrt(x_sq + y_sq)
      if data.round
         return floor(sqrt(x_sq + y_sq) + 0.5)
      end
      return sqrt(x_sq + y_sq)
   else
      return 0.0
   end
end

contains(p, s) = findnext(s, p, 1) != nothing

function readCVRPData(instance)

   demands_sum = 0

   str = Unicode.normalize(read(instance, String); stripcc=true)
   breaks_in = [' '; ':'; '\n']
   aux = split(str, breaks_in; limit=0, keepempty=false)


   G′ = InputGraph([],[],[], Dict())
   data = DataCVRP(G′, 0, 0,0, false, false,splitext(split(basename(app["instance"]))[end] )[1])

   dim = 0
    for i in 1:length(aux)
        if contains(aux[i], "DIMENSION")
         dim = parse(Int, aux[i+1])
      elseif contains(aux[i], "EDGE_WEIGHT_SECTION")
         if data.coord == false
            for j=1:dim
               v = VertexRisk(0, 0, 0, 0)
               v.id_vertex = j-1 # depot is forced to be 0, fisrt customer to be 1, and so on
               push!(G′.V′, v) # add v in the vertex array
            end
         end
      elseif contains(aux[i], "RISK_THRESHOLD")
         data.T = parse(Int64, aux[i+1])  # the method parse() convert the string to Int64
      elseif contains(aux[i], "NODE_COORD_SECTION")
         data.coord = true
         j = i+1
         while aux[j] != "DEMAND_SECTION"
            v = VertexRisk(0, 0, 0, 0)
            v.id_vertex = parse(Int, aux[j])-1 # depot is forced to be 0, fisrt customer to be 1, and so on
            v.pos_x = parse(Float64, aux[j+1])
            v.pos_y = parse(Float64, aux[j+2])
            push!(G′.V′, v) # add v in the vertex array
            j+=3
         end
      elseif contains(aux[i], "DEMAND_SECTION")
         j = i+1
         while aux[j] != "DEPOT_SECTION"
            pos = parse(Int, aux[j])
            G′.V′[pos].demand = parse(Float64, aux[j+1])
            # demands_sum += G′.V′[pos].demand
            j += 2
         end
         data.depot_id = 0
         break
      end
   end

   Lista = Array{Int, 1}(undef, dim+1)
   Lista[1] = data.T
   for k =1:dim
      Lista[k+1] = Int(G′.V′[k].demand )
   end
   # @show Lista
   # @show gcd(Lista)
   data.T = data.T/gcd(Lista)
   for i=1:dim
      G′.V′[i].demand = G′.V′[i].demand/gcd(Lista)
      demands_sum += G′.V′[i].demand
   end

   if data.coord
      # E = {{i,j} : i,j ∈ V′, i < j}
      for i in customers(data)
         e = (data.depot_id, i)
         push!(G′.E, e) # add edge between depot and customer
         data.G′.cost[e] = distance(data, e)
         customers
         e2 = (i, data.depot_id)
         push!(G′.E, e2) # add edge between depot and customer
         data.G′.cost[e2] = distance(data, e2)


         for j in customers(data) # add edges between customers
            if i < j
               e = (i,j)
               push!(G′.E, e) # add edge e
               data.G′.cost[e] = distance(data, e)

               e2 = (j,i)
               push!(G′.E, e2) # add edge e2
               data.G′.cost[e2] = distance(data, e2)
            end
         end
      end
   else
      for i in 1:length(aux)
         if contains(aux[i], "EDGE_WEIGHT_SECTION")
            j=i+1
            for k=0:dim-1, m=0:dim-1
               if k!=m
                  e = (k,m)
                  push!(G′.E, e) # add edge e
                  data.G′.cost[e] = parse(Float64, aux[j])
               end
               j=j+1
            end
         end
      end
   end

   for i=0:demands_sum
      push!(G′.D, i)
   end

   data.n = length(data.G′.V′) - 1 

   return data
end

function CreateDtDemands(data::DataCVRP)
    n = length(data.G′.V′)
    d = zeros(Float64, n, n)

    for i in 1:n
        for j in 1:n
            if i != j
                if i == 1
                    d[i, j] = 0#demands[i]
                else
                    d[i, j] = data.G′.V′[i].demand
                end
            end
        end
    end
    return d
end

function distanceMatrix(cvrp)
    dist = zeros(Float64, cvrp.dimension, cvrp.dimension)
    @show cvrp.coordinates[1, :]
    for i = 1:size(cvrp.coordinates)[1]
        for j = 1:size(cvrp.coordinates)[1]
            if i != j
                dist[i,j] = floor(10*sqrt((cvrp.coordinates[i,:][1] - cvrp.coordinates[j,:][1])^2 + (cvrp.coordinates[i,:][2] - cvrp.coordinates[j,:][2])^2))/10
            end
        end
    end
    return dist
end

function BuildDistMatrix(data::DataCVRP)
    n = length(data.G′.V′)
    D = zeros(Float64, n, n)
    for i=1:n
        for j=1:n
            if i != j
                if i < j
                    D[i,j] = data.G′.cost[(i-1,j-1)]
                else
                    D[i,j] = data.G′.cost[(j-1,i-1)]
                end
            end
        end
    end
    return D
end


