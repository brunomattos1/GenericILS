mutable struct VehicleType
    Q::Int
    fixed::Float64
    factor::Float64
    l::Int
    u::Int
end

mutable struct Vertex_
    idVertex::Int
    posX::Float64
    posY::Float64
    demand::Int
end

mutable struct InputGraph
    V′::Array{Vertex_}
    E::Array{Tuple{Int64,Int64}}
end

mutable struct DataHVRP
    G′::InputGraph
    cost::Dict{Tuple{Int64,Int64,Int64},Float64}
    vehTypes::Array{VehicleType}
    name::String
end

function euclideanDistance(v1::Vertex_, v2::Vertex_)
    return sqrt((v1.posX - v2.posX)^2 + (v1.posY - v2.posY)^2)
end
euclideanDistance(data::DataHVRP, i, j) = euclideanDistance(data.G′.V′[i+1], data.G′.V′[j+1])

function readBrandaoData(path::String)
    raw = Vector{Any}()
    open(path) do file
        for line in eachline(file)
            for token in split(line)
                push!(raw, parse(Float64, token))
            end
        end
    end

    n        = Int(raw[1])
    vertices = Vertex_[]
    offset   = 2
    for i in 0:n
        push!(vertices, Vertex_(i, raw[offset+1], raw[offset+2], 0))
        offset += 3
    end
    for i in 0:n
        vertices[i+1].demand = Int(raw[offset+1])
        offset += 2
    end

    nv        = Int(raw[offset])
    vehTypes = VehicleType[]
    for k in 1:nv
        push!(vehTypes, VehicleType(Int(raw[offset+1]), raw[offset+2], raw[offset+3], Int(raw[offset+4]), Int(raw[offset+5])))
        offset += 5
    end

    E    = Tuple{Int64,Int64}[]
    cost = Dict{Tuple{Int64,Int64,Int64},Float64}()
    for i in 0:n, j in (i+1):n
        push!(E, (i, j))
        for k in 1:nv
            c = vehTypes[k].factor * euclideanDistance(vertices[i+1], vertices[j+1])
            (i == 0 || j == 0) && (c += 0.5 * vehTypes[k].fixed)
            cost[(i, j, k)] = c
        end
    end

    return DataHVRP(InputGraph(vertices, E), cost, vehTypes, splitext(basename(path))[1])
end

function readClassicData(path::String)
    raw = Vector{Any}()
    open(path) do file
        for line in eachline(file)
            for token in split(line)
                push!(raw, parse(Float64, token))
            end
        end
    end

    n        = Int(raw[1])
    vertices = Vertex_[]
    offset   = 2
    for i in 0:n
        push!(vertices, Vertex_(i, raw[offset+1], raw[offset+2], Int(raw[offset+3])))
        offset += 4
    end

    nv        = Int(raw[offset])
    vehTypes = VehicleType[]
    for k in 1:nv
        push!(vehTypes, VehicleType(Int(raw[offset+1]), raw[offset+2], raw[offset+3], Int(raw[offset+4]), Int(raw[offset+5])))
        offset += 5
    end

    E    = Tuple{Int64,Int64}[]
    cost = Dict{Tuple{Int64,Int64,Int64},Float64}()
    for i in 0:n, j in (i+1):n
        push!(E, (i, j))
        for k in 1:nv
            c = vehTypes[k].factor * euclideanDistance(vertices[i+1], vertices[j+1])
            (i == 0 || j == 0) && (c += 0.5 * vehTypes[k].fixed)
            cost[(i, j, k)] = c
        end
    end

    return DataHVRP(InputGraph(vertices, E), cost, vehTypes, splitext(basename(path))[1])
end

nbCustomers(data::DataHVRP)       = length(data.G′.V′) - 1
demand(data::DataHVRP, i)         = Float64(data.G′.V′[i+1].demand)
vehCapacity(data::DataHVRP, k)    = Float64(data.vehTypes[k].Q)
vehFactor(data::DataHVRP, k)      = Float64(data.vehTypes[k].factor)
vehFixed(data::DataHVRP, k)       = Float64(data.vehTypes[k].fixed)
