struct DataCCVRP
    name::String
    weights::Matrix{Float64}
    capacity::Float64
    demand::Vector{Float64}
end

function euclideanDistance(xi, yi, xj, yj)
    return sqrt((xi - xj)^2 + (yi - yj)^2)
end

function nearestIntDistance(xi, yi, xj, yj)
    return round(euclideanDistance(xi, yi, xj, yj))
end

const DISTANCE_FUNCTIONS = Dict(
    "EUC_2D" => euclideanDistance,
    "EXACT_2D" => euclideanDistance,
    "FLOOR_2D" => (xi, yi, xj, yj) -> floor(euclideanDistance(xi, yi, xj, yj)),
    "CEIL_2D" => (xi, yi, xj, yj) -> ceil(euclideanDistance(xi, yi, xj, yj)),
    "ATT" => (xi, yi, xj, yj) -> ceil(sqrt(((xi - xj)^2 + (yi - yj)^2) / 10.0)),
)

function readData(instance::String)
    path = if isfile(instance)
        instance
    else
        folder = if contains(instance, "Golden")
            "Golden"
        elseif contains(instance, "CMT")
            "CMT"
        elseif contains(instance, "Li")
            "Li"
        else
            string(instance[1])
        end
        joinpath(@__DIR__, "..", "data", folder, instance)
    end

    dimension  = 0
    capacity   = 0.0
    weightType = "EUC_2D"
    coords     = Dict{Int, Tuple{Float64, Float64}}()
    demand     = Dict{Int, Float64}()
    section    = :none

    for line in eachline(path)
        line = strip(line)
        isempty(line) && continue

        if startswith(line, "DIMENSION")
            dimension = parse(Int, strip(split(line, ":")[2]))
        elseif startswith(line, "EDGE_WEIGHT_TYPE")
            weightType = strip(split(line, ":")[2])
        elseif startswith(line, "CAPACITY")
            capacity = parse(Float64, strip(split(line, ":")[2]))
        elseif startswith(line, "NODE_COORD_SECTION")
            section = :coord
        elseif startswith(line, "DEMAND_SECTION")
            section = :demand
        elseif startswith(line, "DEPOT_SECTION") || startswith(line, "EOF")
            section = :none
        elseif section == :coord
            tokens = split(line)
            id = parse(Int, tokens[1])
            coords[id] = (parse(Float64, tokens[2]), parse(Float64, tokens[3]))
        elseif section == :demand
            tokens = split(line)
            id = parse(Int, tokens[1])
            demand[id] = parse(Float64, tokens[2])
        end
    end

    distFunc = get(DISTANCE_FUNCTIONS, weightType) do
        error("Unsupported EDGE_WEIGHT_TYPE: $weightType")
    end

    weights = zeros(Float64, dimension, dimension)
    for i in 1:dimension, j in 1:dimension
        i == j && continue
        xi, yi = coords[i]
        xj, yj = coords[j]
        weights[i, j] = distFunc(xi, yi, xj, yj)
    end

    demandVec = [demand[i] for i in 1:dimension]

    return DataCCVRP(
        splitext(basename(instance))[1],
        weights,
        capacity,
        demandVec
    )
end

function buildArcDemands(demands::Vector{Float64})
    n = length(demands)
    d = zeros(Float64, n, n)
    for i in 1:n, j in 1:n
        if i != j && j != 1
            d[i, j] = demands[j]
        end
    end
    return d
end
