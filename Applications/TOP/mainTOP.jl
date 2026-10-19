using GenericILS
using Random
using Printf
include("resourcesTOP.jl")
include("dataTOP.jl")
Random.seed!(0)

"""Print each path with its length and profit, and the total profit of `routes` (lists of customer ids)."""
function printTOP(top, d::Matrix{Float64}, routes::Vector{Vector{Int}})
    total = 0.0
    for (r, route) in enumerate(routes)
        visits = route[2:end-1]
        length_ = sum(d[route[k]+1, route[k+1]+1] for k in 1:length(route)-1)
        prize = sum(top.profit[v + 1] for v in visits)
        total += prize
        @printf("#%d: %s   length = %.2f / %.2f   profit = %.0f\n", r, join(visits, " -> "), length_, top.tmax, prize)
    end
    @printf("\nProfit: %.0f\n", total)
end

"""
Solve a TOP instance (a path or a file of `data/`): visit customers with at most `vehicles` paths of length at most
`Tmax` to collect the largest profit. The solver minimizes `-profit` plus a small distance term. Returns the solver.
"""
function main(instance::String, outerIterMax::Int, innerIterMax::Int, seed::Int)
    path = isfile(instance) ? instance : joinpath(@__DIR__, "data", instance)
    top = readTOP(path)
    d = travelMatrix(top)
    n = top.n
    customers = [optionalVertex(i) for i in 1:n]                     # no customer has to be visited
    res = Resources(
        NoResource(),
        StandardResource{1}(d, zeros(Float64, n + 1), fill(top.tmax, n + 1)),       # length of the path
        StandardResource{2}(zeros(Float64, n + 1, n + 1), zeros(Float64, n + 1), fill(Inf, n + 1))
    )

    algorithm = NILSAlgorithm(res;
        acceptCriteria = NILS.MetropolisTimed(100.0, 100.0, 120.0, 2),
        stopCriteria = NILS.ByTemperature(0.001),
        diversification = NILS.Diversification(outerShift = 2, outerSwap = 2, innerShift = 1, innerSwap = 1),
        innerIterMax = innerIterMax
    )

    solver = Solver(
        res = res,
        data = ProblemData(customers, arcCosts(top, d), top.vehicles),
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
    printTOP(top, d, [rt.visits for rt in sol.routes if length(rt.visits) > 2])
    return solver
end

if abspath(PROGRAM_FILE) == @__FILE__
    instance = length(ARGS) >= 1 ? ARGS[1] : "p4.2.a.txt"
    seed = 1
    outerIterMax = 1000
    innerIterMax = 5

    main(instance, outerIterMax, innerIterMax, seed)
end
