using GenericILS
using Random
include("resources_pkg.jl")
include("data.jl")

using CPLEX

function printSol(sol::Solution)
    for r = 1:length(sol.routes)
        print("route $r: ")
        for v in sol.routes[r].visits
            print("$v ")
        end
        println()
    end
    println("Cost: $(round(sol.cost, digits=2))")
end

function main(data::DataCCVRP, outerIterMax::Int, innerIterMax::Int, seed::Int)
    dist = data.weights
    n    = size(dist, 1) - 1

    customers = Vector{Vertex}()
    for i = 1:n
        push!(customers, Vertex(i))
    end
    maxNbRoute    = ceil(Int, sum(data.demand) / data.capacity)
    dataHeuristic = ProblemData(customers, zeros(Float64, n+1, n+1), maxNbRoute)
    demands   = push!(copy(data.demand), 0.0)
    weights   = vcat(0.0, ones(Float64, n), 0.0)
    d         = buildArcDemands(demands)
    customRes = CustomResource(weights, dist, Inf, Inf)

    nv     = n + 1
    stdRes1 = StandardResource{1}(d, zeros(Float64, nv), Float64[data.capacity for _ in 1:nv])
    stdRes2 = StandardResource{2}(zeros(Float64, nv, nv), zeros(Float64, nv), fill(Inf, nv))
    res     = Resources(customRes, stdRes1, stdRes2)

    algorithm = NILSAlgorithm(res;
        acceptCriteria = NILS.Metropolis(100.0, 0.9),
        stopCriteria = NILS.ByTemperature(0.1),
        diversification = NILS.Diversification(outerShift = 3, outerSwap = 0, innerShift = 1, innerSwap = 0),
        innerIterMax = innerIterMax,
        timeLimitSP = 30.0
    )

    solver = Solver(
        res = res,
        data = dataHeuristic,
        neighborhoods = (
            TwoOptStar(),
            IntraShift(),
            InterShift{1}(),
            InterShift{2}(),
            InterSwap{1, 1}(),
            InterSwap{2, 1}(),
            InterSwap{2, 2}()
        ),
        algorithm = algorithm,
        penaltyManager = TargetRatePenaltyManager(),
        MIPSolver = CPLEX.Optimizer
    )

    setSeed!(solver, Random.MersenneTwister(seed))

    t   = @elapsed solve!(solver)
    sol = getBestSol(solver)

    return sol.cost, t
end

instance = joinpath(@__DIR__, "..", "data", "Golden", "Golden_1.vrp")
data = readData(instance)
main(data, 1, 3, 1)
