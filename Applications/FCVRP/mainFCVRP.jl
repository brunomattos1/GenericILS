using GenericILS
using Random
using Printf
include("resourcesFCVRP.jl")
include("dataFCVRP.jl")
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

"""Print the routes with their load and the family of each vertex, and the cost of `sol`."""
function printFCVRP(solver::Solver, sol::Solution, demands::Vector{Float64})
    data = solver.data
    for (r, rt) in enumerate(filter(rt -> length(rt.visits) > 2, sol.routes))
        route = rt.visits
        println("#$r: ", join(["$v[$(data.groupOf[v])]" for v in route[2:end-1]], " -> "),
            "   load = ", sum(demands[v + 1] for v in route[2:end-1]))
    end
    println("(vertex[family])")
    println("\nCost: $(sol.dist)")
    violations = groupViolations(data, sol)
    isempty(violations) || println("Group violations: ", violations)
end

"""
Solve an F-CVRP instance (`.fcvrp`, a path or a file of `data/`): every family must have `visits` of its
vertices visited, each by one of at most `vehicles` routes. Returns the solver.
"""
function main(instance::String, outerIterMax::Int, innerIterMax::Int, seed::Int)
    path = isfile(instance) ? instance : joinpath(@__DIR__, "data", instance)
    fcvrp = readFCVRP(path)
    dist = euclideanMatrix(fcvrp.coords)
    customers = buildVertices(fcvrp.families)
    demands = vertexDemands(fcvrp.families, length(customers))
    d = createArcDemands(demands)
    customRes = CustomResource(d, fcvrp.capacity)
    n = length(customers) + 1
    stdRes1 = StandardResource{1}(d, zeros(Float64, n), Float64[fcvrp.capacity for _ in 1:n])
    stdRes2 = StandardResource{2}(zeros(Float64, n, n), zeros(Float64, n), fill(Inf, n))

    res = Resources(customRes, stdRes1, stdRes2)

    algorithm = NILSAlgorithm(res;
        acceptCriteria = NILS.MetropolisTimed(100.0, 100.0, 30.0, 2.0),
        stopCriteria = NILS.ByTemperature(0.0001),
        diversification = NILS.Diversification(outerShift = 2, outerSwap = 2, innerShift = 1, innerSwap = 1),
        innerIterMax = innerIterMax
    )

    solver = Solver(
        res = res,
        data = ProblemData(customers, dist, fcvrp.vehicles),
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
    printFCVRP(solver, sol, demands)
    return solver
end

if abspath(PROGRAM_FILE) == @__FILE__
    instance = length(ARGS) >= 1 ? ARGS[1] : "P16_2_2_2.fcvrp"
    seed = 2
    outerIterMax = 1000
    innerIterMax = 10

    main(instance, outerIterMax, innerIterMax, seed)
end
