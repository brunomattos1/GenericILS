using CVRPLIB

struct DataCCVRP
    name::String
    weights::Matrix{Float64}
    capacity::Float64
    demand::Vector{Float64}
end

function readData(instance::String)
    path = joinpath(@__DIR__, "..", "data", string(instance[1]), instance)
    cvrp = CVRPLIB.readCVRP(path)
    return DataCCVRP(
        splitext(basename(instance))[1],
        Float64.(cvrp.weights),
        Float64(cvrp.capacity),
        Float64.(cvrp.demand)
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
