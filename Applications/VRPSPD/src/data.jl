import Unicode

mutable struct vertex
   id_vertex::Int
   p::Float64 #Pickup demand
   d::Float64 #Delivery demand
end

# Undirected graph
mutable struct InputGraph
   V′::Array{vertex} # set of vertices (access with id_vertex + 1)
   E::Array{Tuple{Int64,Int64}} # set of edges
   cost::Dict{Tuple{Int64,Int64},Float64} # cost for each edge
end

mutable struct DataVRPSPDP
   G′::InputGraph
   Q::Float64 # Capacity
   n::Int64
   name::String
end

contains(p, s) = findnext(s, p, 1) != nothing

function Create_PD_Demands(data::DataVRPSPDP)
   n = length(data.G′.V′)
   p = zeros(Float64, n, n)
   d = zeros(Float64, n, n)

   for i in 1:n
      for j in 1:n
         if i != j
            if j == 1
               p[i,j] = 0
               d[i, j] = 0#demands[i]
            else
               p[i,j] = data.G′.V′[j].p
               d[i, j] = data.G′.V′[j].d
            end
         end
      end
   end
   return p,d
end


function BuildDistMatrix(data::DataVRPSPDP)
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


function readVRPSPDP(instance)

   G′ = InputGraph([],[],Dict())
   data = DataVRPSPDP(G′,0.0,0,"")
   
   str = Unicode.normalize(read(instance, String); stripcc=true)
   breaks_in = [' '; ':'; '\n']
   aux = split(str, breaks_in; limit=0, keepempty=false)

   data.Q = parse(Float64, aux[1])
   data.n = parse(Int, aux[2])
   n = data.n

   pos = 3
   for i = 0:n
      for j=0:n
         if i < j
            push!(data.G′.E,(i,j))
            data.G′.cost[(i,j)] = parse(Float64, aux[pos])
         end
         pos += 1
      end
   end

   push!(data.G′.V′,vertex(0,0.0,0.0)) #depot 

   for i=1:n
      d = parse(Float64, aux[pos])
      p = parse(Float64, aux[pos+1])
      push!(data.G′.V′,vertex(i,d,p))
      pos += 2
   end

   return data

   # G′ = InputGraph([],[],[], Dict())
   # data = DataVRPSPDP(G′, 0, 0, false, !app["noround"])

   # dim = 0
   # for i in 1:length(aux)
   #    if contains(aux[i], "DIMENSION")
   #       dim = parse(Int, aux[i+1])
   #    elseif contains(aux[i], "EDGE_WEIGHT_SECTION")
   #       if data.coord == false
   #          for j=1:dim
   #             v = vertex(0, 0, 0, 0)
   #             v.id_vertex = j-1 # depot is forced to be 0, fisrt customer to be 1, and so on
   #             push!(G′.V′, v) # add v in the vertex array
   #          end
   #       end
   #    elseif contains(aux[i], "RISK_THRESHOLD")
   #       data.T = parse(Int64, aux[i+1])  # the method parse() convert the string to Int64
   #    elseif contains(aux[i], "NODE_COORD_SECTION")
   #       data.coord = true
   #       j = i+1
   #       while aux[j] != "DEMAND_SECTION"
   #          v = vertex(0, 0, 0, 0)
   #          v.id_vertex = parse(Int, aux[j])-1 # depot is forced to be 0, fisrt customer to be 1, and so on
   #          v.pos_x = parse(Float64, aux[j+1])
   #          v.pos_y = parse(Float64, aux[j+2])
   #          push!(G′.V′, v) # add v in the vertex array
   #          j+=3
   #       end
   #    elseif contains(aux[i], "DEMAND_SECTION")
   #       j = i+1
   #       while aux[j] != "DEPOT_SECTION"
   #          pos = parse(Int, aux[j])
   #          G′.V′[pos].demand = parse(Float64, aux[j+1])
   #          # demands_sum += G′.V′[pos].demand
   #          j += 2
   #       end
   #       data.depot_id = 0
   #       break
   #    end
   # end

   # Lista = Array{Int, 1}(undef, dim+1)
   # Lista[1] = data.T
   # for k =1:dim
   #    Lista[k+1] = Int(G′.V′[k].demand )
   # end
   # # @show Lista
   # # @show gcd(Lista)
   # data.T = data.T/gcd(Lista)
   # for i=1:dim
   #    G′.V′[i].demand = G′.V′[i].demand/gcd(Lista)
   #    demands_sum += G′.V′[i].demand
   # end

   # if data.coord
   #    # E = {{i,j} : i,j ∈ V′, i < j}
   #    for i in customers(data)
   #       e = (data.depot_id, i)
   #       push!(G′.E, e) # add edge between depot and customer
   #       data.G′.cost[e] = distance(data, e)
   #       customers
   #       e2 = (i, data.depot_id)
   #       push!(G′.E, e2) # add edge between depot and customer
   #       data.G′.cost[e2] = distance(data, e2)


   #       for j in customers(data) # add edges between customers
   #          if i < j
   #             e = (i,j)
   #             push!(G′.E, e) # add edge e
   #             data.G′.cost[e] = distance(data, e)

   #             e2 = (j,i)
   #             push!(G′.E, e2) # add edge e2
   #             data.G′.cost[e2] = distance(data, e2)
   #          end
   #       end
   #    end
   # else
   #    for i in 1:length(aux)
   #       if contains(aux[i], "EDGE_WEIGHT_SECTION")
   #          j=i+1
   #          for k=0:dim-1, m=0:dim-1
   #             if k!=m
   #                e = (k,m)
   #                push!(G′.E, e) # add edge e
   #                data.G′.cost[e] = parse(Float64, aux[j])
   #             end
   #             j=j+1
   #          end
   #       end
   #    end
   # end

   # for i=0:demands_sum
   #    push!(G′.D, i)
   # end

   
end

function c(data::DataVRPSPDP,e::Tuple{Int64, Int64})
   if e[1] == e[2]
      return 0.0
   elseif e[1] < e[2]
      return data.G′.cost[e]
   else
      return data.G′.cost[(e[2],e[1])]  
   end 
end
p(data::DataVRPSPDP,i::Int64) = data.G′.V′[i+1].p
d(data::DataVRPSPDP,i::Int64) = data.G′.V′[i+1].d

function δ(data::DataVRPSPDP,i::Int64)
   E = data.G′.E
   adj = []
   for e ∈ E
      if e[1] == i || e[2] == i
         push!(adj, e)
      end
   end
   return adj
end
