using GenericILS
using Random
include("data.jl")

using CPLEX

# The custom resource is the C++ one of the VRPSolver model (MetaSolver format), the same files that the BCP model uses.
const CPP_DIR = joinpath(@__DIR__, "meta_solver")

"""Print the routes and the cost of `sol`."""
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

"""
Load the C++ resource of `meta_solver/` for an instance with distances `dist`: the arc `(i, j)` has weight 1 when it
enters a customer (0 when it returns to the depot) and travel time `dist[i, j]`, so the cost of a route is the sum of
the arrival times at its customers.
"""
function loadCumulativeResource(dist::Matrix{Float64})
    nv = size(dist, 1)
    return loadCppResource(
        joinpath(CPP_DIR, "rcsp_custom_res_impl.cpp"),
        joinpath(CPP_DIR, "rcsp_custom_res_impl.hpp");
        nv = nv,
        arcParams = (i, j) -> (w = j == 1 ? 0.0 : 1.0, t = dist[i, j]),
        constParams = (Wmax = Inf, Tmax = Inf)    # only read by dominationCost, which the heuristic does not use
    )
end

"""Solve one instance with NILS (build resources, solver and algorithm, run, report the result)."""
function main(data::DataCCVRP, outerIterMax::Int, innerIterMax::Int, seed::Int)
    dist = data.weights
    n    = size(dist, 1) - 1

    customers = Vector{Vertex}()
    for i = 1:n
        push!(customers, Vertex(i, Group(i, 1, false)))   # alone in its group, which needs one visit: a classic VRP
    end
    maxNbRoute    = ceil(Int, sum(data.demand) / data.capacity)
    dataHeuristic = ProblemData(customers, zeros(Float64, n+1, n+1), maxNbRoute)
    demands   = push!(copy(data.demand), 0.0)
    d         = buildArcDemands(demands)
    customRes = loadCumulativeResource(dist)

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

    freeCppResource!(customRes)

    return sol.cost, t
end

instance = joinpath(@__DIR__, "..", "data", "Golden", "Golden_1.vrp")
data = readData(instance)
main(data, 1, 3, 1)
