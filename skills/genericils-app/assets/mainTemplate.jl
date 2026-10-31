# Template main for a new GenericILS application. Runs out of the box on a random
# instance (`demoInstance`), so you can smoke-test the setup before adding real data.
# Search for "CHANGE" to find the parts you must adapt.
using GenericILS
using Random
include("resourcesTemplate.jl")      # CHANGE: rename to resources<Problem>.jl
ENV["JULIA_HASH_SEED"] = "0"

using HiGHS                          # CHANGE: `using CPLEX` if you have a license (faster Set Partitioning)

# CHANGE: replace by a reader for your instance format; return whatever main needs.
function demoInstance(n::Int = 20; seed::Int = 1)
    rng = MersenneTwister(seed)
    xy = rand(rng, n + 1, 2) .* 100
    dist = [sqrt(sum((xy[i, :] .- xy[j, :]) .^ 2)) for i in 1:n+1, j in 1:n+1]
    demand = vcat(0.0, Float64.(rand(rng, 1:10, n)))   # index 1 = depot
    return dist, demand, 40.0
end

# CHANGE: arc-indexed data derived from the instance (see references/api.md on indexing).
function arcDemands(demand)
    n = length(demand)
    d = zeros(Float64, n, n)
    for i in 1:n, j in 1:n
        i != j && j != 1 && (d[i, j] = demand[j])
    end
    return d
end

# CHANGE: an independent checker of YOUR constraints, written without the labels.
# This is how you know the resource implementation is right.
function checkSolution(sol::Solution, demand, Q, n)
    seen = Int[]
    for rt in sol.routes
        length(rt.visits) <= 2 && continue
        inner = rt.visits[2:end-1]
        append!(seen, inner)
        sum(demand[inner .+ 1]) <= Q + 1e-6 || error("capacity violated in $(rt.visits)")
    end
    sort(seen) == collect(1:n) || error("customers not visited exactly once")
    return true
end

function main(outerIterMax::Int, innerIterMax::Int, seed::Int)
    dist, demand, Q = demoInstance()                 # CHANGE: read your instance
    n = length(demand) - 1
    customers = [Vertex(i, Group(i, 1, false)) for i in 1:n]   # one group per customer, one visit
    d = arcDemands(demand)

    customRes = CustomResource(d, Q)                 # CHANGE
    # Standard resources: 2 generic "bounded accumulators" (time windows, capacity, ...).
    # Leave a slot loose (zeros / Inf) when unused. See references/api.md.
    stdRes1 = StandardResource{1}(d, zeros(n + 1), fill(Q, n + 1))                       # capacity mirrored
    stdRes2 = StandardResource{2}(zeros(n + 1, n + 1), zeros(n + 1), fill(Inf, n + 1))   # unused
    res = Resources(customRes, stdRes1, stdRes2)

    algorithm = NILSAlgorithm(res;
        acceptCriteria = NILS.AcceptBest(),
        stopCriteria = NILS.ByIterMax(outerIterMax),
        diversification = NILS.Diversification(outerShift = 1, outerSwap = 0, innerShift = 2, innerSwap = 0),
        innerIterMax = innerIterMax
    )

    solver = Solver(
        res = res,
        data = ProblemData(customers, dist, ceil(Int, sum(demand) / Q) + 3),   # CHANGE: max number of routes
        neighborhoods = (
            TwoOptStar(), IntraShift(), InterShift{1}(), InterShift{2}(),
            InterSwap{1, 1}(), InterSwap{2, 1}(), InterSwap{2, 2}()
        ),
        algorithm = algorithm
    )
    setSeed!(solver, Random.MersenneTwister(seed))
    setPenaltyManager!(solver, StandardPenaltyManager(
        penaltyCustom = 1.0, penaltyCustomIncrease = 0.01, penaltyCustomDecrease = 0.01,
        penaltyStandard1 = 1.0, penaltyStandard1Increase = 0.01, penaltyStandard1Decrease = 0.01,
        penaltyStandard2 = 0.0, penaltyStandard2Increase = 0.0, penaltyStandard2Decrease = 0.0
    ))
    setMIPSolver!(solver, HiGHS.Optimizer)           # CHANGE: CPLEX.Optimizer if available

    @time solve!(solver)
    sol = getBestSol(solver)
    checkSolution(sol, demand, Q, n)
    println("Checked OK. Cost: ", sol.cost)
    return sol.cost
end

main(100, 5, 1)    # CHANGE: outerIterMax, innerIterMax, seed
