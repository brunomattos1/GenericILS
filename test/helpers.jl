# Shared test fixtures: a small CVRP (custom resource = capacity) and a variant with time
# windows in StandardResource{1} and capacity in StandardResource{2}.
using Test
using Random
using GenericILS
using HiGHS
import GenericILS: isSymmetric, isCostResource, initStateForward, initStateBackward,
    extendAlongArc, concatenationCost

struct CustomResource <: AbstractResource
    d::Matrix{Float64}
    Q::Float64
end
struct ForwardState
    q::Float64
end
struct BackwardState
    q::Float64
end

GenericILS.isSymmetric() = false
GenericILS.isCostResource() = false
initStateForward(::CustomResource) = (ForwardState(0.0), 0.0)
initStateBackward(::CustomResource) = (BackwardState(0.0), 0.0)

function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int,Int})
    q = label.state.q + res.d[(1, a[2])...]
    q > res.Q + 1e-5 ? (ForwardState(q), Inf) : (ForwardState(q), 0.0)
end
function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int,Int})
    q = label.state.q + res.d[(1, a[1])...]
    q > res.Q + 1e-5 ? (BackwardState(q), Inf) : (BackwardState(q), 0.0)
end
function concatenationCost(res::CustomResource, v::Int, f::ForwardLabel, b::BackwardLabel)
    q = f.state.q + b.state.q
    q > res.Q + 1e-5 ? (ForwardState(q), Inf) : (ForwardState(q), 0.0)
end

function arcDemands(demands)
    n = length(demands)
    d = zeros(Float64, n, n)
    for i in 1:n, j in 1:n
        i != j && j != 1 && (d[i, j] = demands[j])
    end
    return d
end

struct DummyAlgorithm <: Algorithm end

const NEIGHBORHOODS = (TwoOptStar(), IntraShift(), InterShift{1}(), InterShift{2}(),
    InterSwap{1,1}(), InterSwap{2,1}(), InterSwap{2,2}())

function randomInstance(nCust; seed = 1, Q = 30.0)
    rng = MersenneTwister(seed)
    xy = rand(rng, nCust + 1, 2) .* 100
    dist = [sqrt(sum((xy[i, :] .- xy[j, :]) .^ 2)) for i in 1:nCust+1, j in 1:nCust+1]
    demands = vcat(0.0, Float64.(rand(rng, 1:10, nCust)))
    return dist, demands, Q
end

"""
    buildCVRP(algorithmFor; nCust, Q, instanceSeed, neighborhoods, penaltyManager)

Returns `(solver, demands, Q)`. `algorithmFor(res)` builds the algorithm.
"""
function buildCVRP(algorithmFor; nCust = 12, Q = 30.0, instanceSeed = 1,
                   neighborhoods = NEIGHBORHOODS, penaltyManager = nothing)
    dist, demands, Q = randomInstance(nCust; seed = instanceSeed, Q = Q)
    d = arcDemands(demands)
    n = nCust + 1
    res = Resources(
        CustomResource(d, Q),
        StandardResource{1}(d, zeros(n), fill(Q, n)),
        StandardResource{2}(zeros(n, n), zeros(n), fill(Inf, n)),
    )
    extra = penaltyManager === nothing ? (;) : (; penaltyManager = penaltyManager)
    solver = Solver(;
        res = res,
        data = ProblemData([Vertex(i) for i in 1:nCust], dist, ceil(Int, sum(demands) / Q) + 3),
        neighborhoods = neighborhoods,
        algorithm = algorithmFor(res),
        extra...
    )
    setSeed!(solver, MersenneTwister(2))
    return solver, demands, Q
end

"""
    buildTW(algorithmFor; nCust, Q, horizon)

Variant with time windows in `StandardResource{1}` (travel time = distance / 10) and capacity in
`StandardResource{2}`. Returns `(solver, demands, Q, t, lb, ub)`.
"""
function buildTW(algorithmFor; nCust = 10, Q = 30.0, horizon = 60.0, instanceSeed = 3)
    dist, demands, Q = randomInstance(nCust; seed = instanceSeed, Q = Q)
    t = dist ./ 10
    n = nCust + 1
    rng = MersenneTwister(instanceSeed)
    lb = vcat(0.0, rand(rng, n - 1) .* horizon / 2)
    ub = vcat(10 * horizon, lb[2:end] .+ horizon)
    d = arcDemands(demands)
    res = Resources(
        CustomResource(d, Q),
        StandardResource{1}(t, lb, ub),
        StandardResource{2}(d, zeros(n), fill(Q, n)),
    )
    solver = Solver(
        res = res,
        data = ProblemData([Vertex(i) for i in 1:nCust], dist, ceil(Int, sum(demands) / Q) + 3),
        neighborhoods = NEIGHBORHOODS,
        algorithm = algorithmFor(res),
    )
    setSeed!(solver, MersenneTwister(2))
    setPenaltyManager!(solver, StandardPenaltyManager(
        penaltyCustom = 1.0, penaltyCustomIncrease = 0.05, penaltyCustomDecrease = 0.05,
        penaltyStandard1 = 1.0, penaltyStandard1Increase = 0.05, penaltyStandard1Decrease = 0.05,
        penaltyStandard2 = 1.0, penaltyStandard2Increase = 0.05, penaltyStandard2Decrease = 0.05))
    return solver, demands, Q, t, lb, ub
end

"Independent check of a CVRP solution: every customer once, capacity respected."
function checkCVRP(sol, demands, Q, nCust)
    visited = Int[]
    for r in sol.routes
        v = r.visits
        length(v) <= 2 && continue
        @test v[1] == 0 && v[end] == 0
        inner = v[2:end-1]
        append!(visited, inner)
        @test sum(demands[inner .+ 1]) <= Q + 1e-5
    end
    @test sort(visited) == collect(1:nCust)
end

"Independent time-window check: waits until `lb`, must arrive by `ub`."
function checkTW(sol, t, lb, ub)
    for r in sol.routes
        v = r.visits
        length(v) <= 2 && continue
        time = 0.0
        for k in 2:length(v)
            time = max(time + t[v[k-1]+1, v[k]+1], lb[v[k]+1])
            @test time <= ub[v[k]+1] + 1e-6
        end
    end
end

nilsAlgorithm(; accept = NILS.AcceptBest(), stop = NILS.ByIterMax(4), kwargs...) =
    res -> NILSAlgorithm(res; acceptCriteria = accept, stopCriteria = stop,
        diversification = NILS.Diversification(outerShift = 1, outerSwap = 1, innerShift = 2, innerSwap = 1),
        innerIterMax = 3, kwargs...)

ilsAlgorithm(; accept = ILS.AcceptBest(), stop = ILS.ByIterMax(4), kwargs...) =
    res -> ILSAlgorithm(res; acceptCriteria = accept, stopCriteria = stop,
        diversification = ILS.Diversification(shift = 2, swap = 1), kwargs...)

"Run `f` with stdout silenced (the package prints routes and progress tables)."
quiet(f) = redirect_stdout(f, devnull)

"Run `f` and return everything it printed to stdout."
function capture(f)
    mktemp() do path, io
        redirect_stdout(io) do
            f()
        end
        flush(io)
        return read(path, String)
    end
end

"`getBestSol` prints the routes; silence it."
bestSol(solver) = quiet(() -> getBestSol(solver))

# ---------------------------------------------------------------------------------------------
# Helpers for unit tests of labels, infeasibility evaluation and neighborhoods
# ---------------------------------------------------------------------------------------------

"Solver over an explicit distance matrix (index 1 = depot), unit demands and loose capacity."
function solverFromDist(dist; demands = nothing, Q = 1000.0, algorithmFor = nilsAlgorithm())
    n = size(dist, 1) - 1
    demands === nothing && (demands = vcat(0.0, ones(n)))
    d = arcDemands(demands)
    res = Resources(
        CustomResource(d, Q),
        StandardResource{1}(d, zeros(n + 1), fill(Q, n + 1)),
        StandardResource{2}(zeros(n + 1, n + 1), zeros(n + 1), fill(Inf, n + 1)),
    )
    solver = Solver(res = res, data = ProblemData([Vertex(i) for i in 1:n], dist, n),
        neighborhoods = NEIGHBORHOODS, algorithm = algorithmFor(res))
    setSeed!(solver, MersenneTwister(2))
    return solver
end

"Points on a line: customer i sits at `xs[i]`, the depot at 0, distance = |x - y|."
function lineSolver(xs; kwargs...)
    pos = vcat(0.0, Float64.(xs))
    return solverFromDist([abs(a - b) for a in pos, b in pos]; kwargs...)
end

"Recompute distance, per-route bookkeeping, totals and cost of `sol` from its visits."
function refreshBookkeeping!(solver, sol)
    C = solver.data.costMatrix
    sol.totalInfeas = 0
    sol.totalWarpStd1 = 0.0
    sol.totalWarpStd2 = 0.0
    sol.totalLabelCost = 0.0
    sol.dist = 0.0
    for rt in sol.routes
        v = rt.visits
        rt.infeas = length(v) - max(rt.feasibleF, rt.feasibleB) - 1
        rt.warpStd1 = rt.forwardLabels[end].std1State.stdWarp
        rt.warpStd2 = rt.forwardLabels[end].std2State.stdWarp
        rt.labelCost = min(rt.forwardLabels[end].cost, rt.backwardLabels[end].cost)
        sol.totalInfeas += rt.infeas
        sol.totalWarpStd1 += rt.warpStd1
        sol.totalWarpStd2 += rt.warpStd2
        sol.totalLabelCost += rt.labelCost
        sol.dist += sum(C[v[k]+1, v[k+1]+1] for k in 1:length(v)-1)
    end
    sol.cost = objectiveValue(solver, sol)
    return sol
end

"Build a fully consistent `Solution` (labels, bookkeeping, lastEval) from raw routes."
function makeSolution(solver, routes::Vector{Vector{Int}})
    sol = new_solution(solver)
    sol.routes = [new_route(solver, copy(v)) for v in routes]
    sol.lastEval = zeros(Int, length(solver.neighborhoods), length(routes), length(routes))
    computeLabels(solver, sol)
    return refreshBookkeeping!(solver, sol)
end

"Random partition of customers 1..n into `R` routes (some may be empty)."
function randomRoutes(rng, n, R)
    perm = shuffle(rng, collect(1:n))
    cuts = sort!(rand(rng, 0:n, R - 1))
    bounds = vcat(0, cuts, n)
    return [vcat(0, perm[bounds[r]+1:bounds[r+1]], 0) for r in 1:R]
end

"`(infeas, labelCost, warpStd1, warpStd2)` of a route computed from scratch."
function routeStats(solver, visits)
    rt = makeSolution(solver, [visits]).routes[1]
    return (rt.infeas, rt.labelCost, rt.warpStd1, rt.warpStd2)
end

"Check that all incremental bookkeeping of `sol` equals a from-scratch recomputation."
function checkConsistency(solver, sol)
    ref = makeSolution(solver, [r.visits for r in sol.routes])
    @test sol.dist ≈ ref.dist
    @test sol.cost ≈ ref.cost
    @test sol.totalInfeas == ref.totalInfeas
    @test sol.totalWarpStd1 ≈ ref.totalWarpStd1 atol = 1e-9
    @test sol.totalWarpStd2 ≈ ref.totalWarpStd2 atol = 1e-9
    for (rt, rr) in zip(sol.routes, ref.routes)
        @test rt.infeas == rr.infeas
        @test rt.warpStd1 ≈ rr.warpStd1 atol = 1e-9
        @test rt.warpStd2 ≈ rr.warpStd2 atol = 1e-9
        @test rt.labelCost == rr.labelCost
        @test (rt.feasibleF, rt.feasibleB) == (rr.feasibleF, rr.feasibleB)
        @test rt.forwardLabels == rr.forwardLabels
        @test rt.backwardLabels == rr.backwardLabels
    end
end

customersOf(sol) = sort!(reduce(vcat, [r.visits[2:end-1] for r in sol.routes]; init = Int[]))
