using GenericILS
using Random
using Printf
include("resourcesPCVRP.jl")
include("dataPCVRP.jl")
Random.seed!(0)

"""Print each route with its load and profit, the demand satisfied and the total profit."""
function printPCVRP(pc, dist::Matrix{Float64}, routes::Vector{Vector{Int}})
    served = prizes = travel = 0.0
    for (r, route) in enumerate(routes)
        visits = route[2:end-1]
        length_ = sum(dist[route[k]+1, route[k+1]+1] for k in 1:length(route)-1)
        load = sum(pc.demand[v + 1] for v in visits)
        prize = sum(pc.prize[v + 1] for v in visits)
        served += load; prizes += prize; travel += length_
        @printf("#%d: %s   load = %.0f / %.0f   distance = %.2f   prize = %.0f\n", r, join(visits, " -> "), load, pc.capacity, length_, prize)
    end
    @printf("\nDemand satisfied: %.0f (the quota of %.0f of the instance is not enforced)\nProfit: %.2f (prizes %.0f - distance %.2f)\n",
        served, pc.quota, prizes - travel, prizes, travel)
end

"""
Solve a PCVRP instance (a path or a file of `data/`): visit customers with at most `vehicles` routes of capacity
`C` to maximize the prizes collected minus the distance. The solver minimizes the opposite. The minimum amount of
demand to satisfy that the instances state (`Q`) is NOT enforced: this is the problem without that constraint.
`seconds` is the time the Metropolis acceptance takes to cool down. Returns the solver.
"""
function main(instance::String, outerIterMax::Int, innerIterMax::Int, seed::Int; seconds::Float64 = 10.0)
    path = isfile(instance) ? instance : joinpath(@__DIR__, "data", instance)
    pc = readPCVRP(path)
    dist = distanceMatrix(pc)
    n = length(pc.coords) - 1
    customers = [optionalVertex(i) for i in 1:n]                     # no customer has to be visited
    demand = zeros(Float64, n + 1, n + 1)                           # arc (i, j) brings the demand of j
    for i in 1:n+1, j in 2:n+1
        i != j && (demand[i, j] = pc.demand[j])
    end
    res = Resources(
        NoResource(),
        StandardResource{1}(demand, zeros(Float64, n + 1), fill(pc.capacity, n + 1)),         # capacity
        StandardResource{2}(zeros(Float64, n + 1, n + 1), zeros(Float64, n + 1), fill(Inf, n + 1))
    )

    algorithm = NILSAlgorithm(res;
        acceptCriteria = NILS.MetropolisTimed(100.0, 100.0, seconds, 2),
        stopCriteria = NILS.ByTemperature(0.001),
        diversification = NILS.Diversification(outerShift = 2, outerSwap = 2, innerShift = 1, innerSwap = 1),
        innerIterMax = innerIterMax
    )

    solver = Solver(
        res = res,
        data = ProblemData(customers, arcCosts(pc, dist), pc.vehicles),
        neighborhoods = (
            TwoOptStar(),
            IntraShift(),
            InterShift{1}(),
            InterShift{2}(),
            InterSwap{1, 1}(),
            InterSwap{2, 1}(),
            InterSwap{2, 2}(),
            AddOptional(),
            RemoveOptional()
        ),
        algorithm = algorithm
    )

    setSeed!(solver, Random.MersenneTwister(seed))

    setPenaltyManager!(solver, StandardPenaltyManager(
        penaltyCustom = 0.0, penaltyCustomIncrease = 0.0, penaltyCustomDecrease = 0.0,
        penaltyStandard1 = 5.0, penaltyStandard1Increase = 0.02, penaltyStandard1Decrease = 0.02,
        penaltyStandard2 = 0.0, penaltyStandard2Increase = 0.0, penaltyStandard2Decrease = 0.0
    ))

    @time solve!(solver)
    sol = getBestSol(solver)
    printPCVRP(pc, dist, [rt.visits for rt in sol.routes if length(rt.visits) > 2])
    return solver
end

if abspath(PROGRAM_FILE) == @__FILE__
    instance = length(ARGS) >= 1 ? ARGS[1] : "A-n32-k5_1.txt"
    seed = 1
    outerIterMax = 1000
    innerIterMax = 5

    main(instance, outerIterMax, innerIterMax, seed)
end
