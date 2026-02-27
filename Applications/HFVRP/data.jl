import Base.show, Base.print

mutable struct VehicleType
   Q::Int
   fixed::Float64
   factor::Float64
   l::Int
   u::Int
end

mutable struct vertex
   id_vertex::Int
   pos_x::Int
   pos_y::Int
   demand::Int
end

# Undirected graph
mutable struct InputGraph
   V′::Array{vertex} # set of vertices
   E::Array{Tuple{Int64,Int64}} # set of edges
end

mutable struct DataHVRP
   G′::InputGraph
   cost::Dict{Tuple{Int64,Int64,Int64},Float64} # cost for each edge and vehicle type
   veh_types::Array{VehicleType} 
   name::String
end

# Euclidian distance
function distance(v1::vertex, v2::vertex)
   x_sq = (v1.pos_x - v2.pos_x)^2
   y_sq = (v1.pos_y - v2.pos_y)^2
   return sqrt(x_sq + y_sq)
end
distance(data::DataHVRP, i, j) = distance(data.G′.V′[i+1], data.G′.V′[j+1])

function readHVRPBrandaoData(path_file::String)
   # STEP 1 : pushing data in a vector.
   data = Vector{Any}()
   open(path_file) do file
      for line in eachline(file)
         for peaceofdata in split(line)
            push!(data, parse(Float64, peaceofdata))
         end
      end
   end

   n = Int(data[1]) 

   vertices = vertex[] 
   #reading coordinates
   offset = 2
   for i in 0:n
      x = Int(data[offset + 1])     
      y = Int(data[offset + 2])     
      push!(vertices, vertex(i, x, y, 0))
      offset += 3
   end
   #reading demands
   for i in 0:n
      vertices[i+1].demand = Int(data[offset + 1])
      offset += 2
   end

   nb_veh_type = data[offset]
   veh_types = VehicleType[]
   for k in 1:nb_veh_type
      Q = Int(data[offset + 1])
      fixed = data[offset + 2]
      factor = data[offset + 3]
      l = Int(data[offset + 4])
      u = Int(data[offset + 5])
      push!(veh_types, VehicleType(Q, fixed, factor, l, u))
      offset += 5
   end
  
   E = Tuple{Int64,Int64}[]
   cost = Dict{Tuple{Int64,Int64,Int64},Float64}()

   function add_edge(i, j, veh_types)
      push!(E, (i,j)) 
      for k in 1:length(veh_types)
         c = veh_types[k].factor*distance(vertices[i + 1], vertices[j + 1]) 
         if i == 0 || j == 0
            c += 0.5*veh_types[k].fixed
         end
         cost[(i,j,k)] = c
      end
   end
   for i in 0:n
      for j in (i+1):n
         add_edge(i, j, veh_types)
      end
   end

   DataHVRP(InputGraph(vertices, E), cost, veh_types,"oi")
end

function readHVRPClassicData(path_file::String)
   # STEP 1 : pushing data in a vector.
   data = Vector{Any}()
   open(path_file) do file
      for line in eachline(file)
         for peaceofdata in split(line)
            push!(data, parse(Float64, peaceofdata))
         end
      end
   end

   n = Int(data[1])

   vertices = vertex[] 
   #reading coordinates
   offset = 2
   for i in 0:n
      x = Int(data[offset + 1])     
      y = Int(data[offset + 2])  
      d = Int(data[offset + 3])     
      push!(vertices, vertex(i, x, y, d))
      offset += 4
   end

   nb_veh_type = data[offset]
   veh_types = VehicleType[]
   for k in 1:nb_veh_type
      Q = Int(data[offset + 1])
      fixed = data[offset + 2]
      factor = data[offset + 3]
      l = Int(data[offset + 4])
      u = Int(data[offset + 5])
      push!(veh_types, VehicleType(Q, fixed, factor, l, u))
      offset += 5
   end
  
   E = Tuple{Int64,Int64}[]
   cost = Dict{Tuple{Int64,Int64,Int64},Float64}()

   function add_edge(i, j, veh_types)
      push!(E, (i,j)) 
      for k in 1:length(veh_types)
         c = veh_types[k].factor*distance(vertices[i + 1], vertices[j + 1]) 
         if i == 0 || j == 0
	         c += 0.5*veh_types[k].fixed
	      end
	      cost[(i,j,k)] = c
      end
   end
   for i in 0:n
      for j in (i+1):n
         add_edge(i, j, veh_types)
      end
   end

   DataHVRP(InputGraph(vertices, E), cost, veh_types,"oi")
end


customers(data::DataHVRP) = [i.id_vertex for i in data.G′.V′[2:end]] # return set of customers
veh_types(data::DataHVRP) = [i for i in 1:length(data.veh_types)] # return set of customers
edges(data::DataHVRP) = data.G′.E # return set of edges
c(data,e,k) = data.cost[(e[1],e[2],k)] # cost of the edge e in veh_type k
dimension(data::DataHVRP) = length(data.G′.V′) # return number of vertices
d(data::DataHVRP, i) = Float64(data.G′.V′[i+1].demand) # return demand of i
veh_capacity(data::DataHVRP, k) = Float64(data.veh_types[k].Q)
veh_factor(data::DataHVRP, k) = Float64(data.veh_types[k].factor)
veh_fixed(data::DataHVRP, k) = Float64(data.veh_types[k].fixed)
nb_customers(data::DataHVRP) = length(customers(data))

function lowerBoundNbVehicles(data::DataHVRP, k::Int) 
   return data.veh_types[k].l
end

function upperBoundNbVehicles(data::DataHVRP, k::Int) 
   return data.veh_types[k].u
end

# return incident edges of i
function δ(data::DataHVRP, i::Integer)
   incident_edges = Vector{Tuple}()
   for j in 0:i-1 push!(incident_edges, (j, i)) end
   for j in i+1:(length(data.G′.V′)-1) push!(incident_edges, (i, j)) end
   return incident_edges
end

function show(data::DataHVRP)
   n = nb_customers(data)
   for i in 0:n
      v = data.G′.V′[i+1]
      println("vertex $(i+1) x=$(v.pos_x) y=$(v.pos_y) d=$(v.demand)")
   end
   for vt in data.veh_types
      println("veh type: $(vt.Q) $(vt.fixed) $(vt.factor) $(vt.l) $(vt.u)")
   end
end
