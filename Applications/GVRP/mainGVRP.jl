using GenericILS
using Random
using Printf
include("resourcesGVRP.jl")
include("dataGVRP.jl")
Random.seed!(0)

"""Arc-indexed demand matrix: entry `(i, j)` is the demand of `j` (0 for the depot)."""
function createArcDemands(demands)
    n = length(demands)
    d = zeros(Float64, n, n)
    for i in 1:n, j in 2:n
        i != j && (d[i, j] = demands[j])
    end
    return d
end

"""Print the routes with their load, the vertices visited per cluster and the cost of `sol`."""
function printGVRP(solver::Solver, sol::Solution, demands::Vector{Float64})
    data = solver.data
    for (r, rt) in enumerate(filter(rt -> length(rt.visits) > 2, sol.routes))
        route = rt.visits
        println("#$r: ", join(["$v[$(data.groupOf[v])]" for v in route[2:end-1]], " -> "),
            "   load = ", sum(demands[v + 1] for v in route[2:end-1]))
    end
    println("(vertex[cluster])")
    println("\nCost: $(sol.dist)")
    violations = groupViolations(data, sol)
    isempty(violations) || println("Group violations: ", violations)
end

"""
Solve a GVRP instance of `data/` (Bektaş et al. format): the solution visits `numVisits` vertices of each
cluster (one by default), with at most `vehicles` routes. Every vertex carries the demand of its cluster. Returns the solver.
"""
function main(instance::String, outerIterMax::Int, innerIterMax::Int, seed::Int; numVisits::Int = 1)
    gvrp = readGVRP(joinpath(@__DIR__, "data", instance))
    dist = euclideanMatrix(gvrp.coords)
    customers = buildVertices(gvrp.clusters, numVisits)
    demands = gvrp.demand
    maxNbRoute = gvrp.vehicles
    d = createArcDemands(demands)
    customRes = CustomResource(d, gvrp.capacity)
    n = length(customers) + 1
    stdRes1 = StandardResource{1}(d, zeros(Float64, n), Float64[gvrp.capacity for _ in 1:n])
    stdRes2 = StandardResource{2}(zeros(Float64, n, n), zeros(Float64, n), fill(Inf, n))

    res = Resources(customRes, stdRes1, stdRes2)

    algorithm = NILSAlgorithm(res;
        acceptCriteria = NILS.MetropolisTimed(100.0, 100.0, 30.0, 2.0),
        stopCriteria = NILS.ByTemperature(0.0001),
        diversification = NILS.Diversification(outerShift = 3, outerSwap = 3, innerShift = 1, innerSwap = 1),
        innerIterMax = innerIterMax
    )

    solver = Solver(
        res = res,
        data = ProblemData(customers, dist, maxNbRoute),
        neighborhoods = (
            TwoOptStar(),
            IntraShift(),
            InterShift{1}(),
            InterShift{2}(),
            InterSwap{1, 1}(),
            InterSwap{2, 1}(),
            InterSwap{2, 2}(),
            IntraGroupExchange()
        ),
        algorithm = algorithm
    )

    setSeed!(solver, Random.MersenneTwister(seed))

    setPenaltyManager!(solver, StandardPenaltyManager(
        penaltyCustom = 1.0, penaltyCustomIncrease = 0.01, penaltyCustomDecrease = 0.01,
        penaltyStandard1 = 1.0, penaltyStandard1Increase = 0.01, penaltyStandard1Decrease = 0.01,
        penaltyStandard2 = 0.0, penaltyStandard2Increase = 0.0, penaltyStandard2Decrease = 0.0
    ))

    @time solve!(solver)
    sol = getBestSol(solver)
    printGVRP(solver, sol, demands)
    return solver
end

if abspath(PROGRAM_FILE) == @__FILE__
    instance = "M-n121-k7-C61-V4.vrp"
    seed = 2
    outerIterMax = 200
    innerIterMax = 5

    main(instance, outerIterMax, innerIterMax, seed)
end
