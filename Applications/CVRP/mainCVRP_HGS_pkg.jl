using GenericILS
using Random
include("resourcesCVRP_pkg.jl")
ENV["JULIA_HASH_SEED"] = "0"
Random.seed!(0)

using CVRPLIB
using CPLEX

function createArcDemands(demands)
    n = length(demands)
    d = zeros(Float64, n, n)
    for i in 1:n
        for j in 1:n
            if i != j
                if j == 1
                    d[i, j] = 0
                else
                    d[i, j] = demands[j]
                end
            end
        end
    end
    return d
end

function main(instance::String, seed::Int)
    instance = joinpath(normpath(joinpath(@__DIR__, "data")), string(instance[1]), instance)

    cvrp = CVRPLIB.readCVRP(instance)
    dist = Float64.(cvrp.weights)
    customers = Vector{Vertex}()
    nbCustomer = size(dist)[1] - 1
    for i = 1:nbCustomer
        push!(customers, Vertex(i))
    end
    maxNbRoute = 3+ceil(Int, sum(cvrp.demand) / cvrp.capacity) + 3
    demands = push!(Float64.(cvrp.demand), 0.0)
    d = createArcDemands(demands)
    customRes = CustomResource(d, cvrp.capacity)
    n = length(customers) + 1
    stdRes1 = StandardResource{1}(d, zeros(Float64, n), Float64[cvrp.capacity for _ in 1:n])
    stdRes2 = StandardResource{2}(zeros(Float64, n, n), zeros(Float64, n), fill(Inf, n))

    res = Resources(customRes, stdRes1, stdRes2)

    algorithm = HGSAlgorithm(res;
        stopCriteria = HGS.ByTime(5.0),
        muMax = 15, lambda = 10, nClosest = 5, nElite = 4, nbIterNonProd = 20000
    )

    solver = Solver(
        res = res,
        data = ProblemData(customers, dist, maxNbRoute),
        neighborhoods = (
            TwoOptStar(),
            IntraShift(),
            InterShift{1}(),
            # InterShift{2}(),
            InterSwap{1, 1}(),
            # InterSwap{2, 1}(),
            # InterSwap{2, 2}()
        ),
        algorithm = algorithm,
        penaltyManager = TargetRatePenaltyManager(penaltyCustom = 1.0,
    penaltyCustomIncrease = 0.3,
    penaltyCustomDecrease = 0.1,
    penaltyStandard1 = 1.0,
    penaltyStandard1Increase = 0.3,
    penaltyStandard1Decrease = 0.1,
    penaltyStandard2 = 1.0,
    penaltyStandard2Increase = 0.4,
    penaltyStandard2Decrease = 0.2,
    targetFeasRate = 0.2,
    feasRateTolerance = 0.05,
    updatePeriod = 200)
    )

    setSeed!(solver, Random.MersenneTwister(seed))
    setMIPSolver!(solver, CPLEX.Optimizer)

    @time solve!(solver)
    sol = getBestSol(solver)
    return sol.cost
end

set = "M"
n = 151
k = 12
instance = "$set-n$n-k$k.vrp"
seed = 1

main(instance, seed)
