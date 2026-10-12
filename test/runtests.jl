using Test
using Random
using GenericILS
import GenericILS: isSymmetric, isCostResource, initStateForward, initStateBackward,
    extendAlongArc, concatenationCost

# Minimal CVRP defined through the public resource API (see docs/tutorial.md)
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

function buildSolver(algorithmFor; nCust = 12, Q = 30.0)
    rng = MersenneTwister(1)
    xy = rand(rng, nCust + 1, 2) .* 100
    dist = [sqrt(sum((xy[i, :] .- xy[j, :]) .^ 2)) for i in 1:nCust+1, j in 1:nCust+1]
    demands = vcat(0.0, Float64.(rand(rng, 1:10, nCust)))   # depot first
    d = arcDemands(demands)
    n = nCust + 1
    res = Resources(
        CustomResource(d, Q),
        StandardResource{1}(d, zeros(n), fill(Q, n)),
        StandardResource{2}(zeros(n, n), zeros(n), fill(Inf, n)),
    )
    solver = Solver(
        res = res,
        data = ProblemData([Vertex(i) for i in 1:nCust], dist, ceil(Int, sum(demands) / Q) + 3),
        neighborhoods = (TwoOptStar(), IntraShift(), InterShift{1}(), InterSwap{1,1}()),
        algorithm = algorithmFor(res),
    )
    setSeed!(solver, MersenneTwister(2))
    return solver, demands, Q
end

function checkSolution(solver, demands, Q, nCust)
    sol = getBestSol(solver)
    visited = Int[]
    for r in sol.routes
        v = r.visits
        @test v[1] == 0 && v[end] == 0
        inner = v[2:end-1]
        append!(visited, inner)
        @test sum(demands[inner .+ 1]) <= Q + 1e-5
    end
    @test sort(visited) == collect(1:nCust)
    @test isfinite(sol.cost) && sol.cost > 0
end

@testset "GenericILS" begin
    @testset "NILS solves a small CVRP" begin
        solver, demands, Q = buildSolver(res -> NILSAlgorithm(res;
            acceptCriteria = NILS.AcceptBest(),
            stopCriteria = NILS.ByIterMax(5),
            diversification = NILS.Diversification(outerShift = 1, outerSwap = 0, innerShift = 2, innerSwap = 0),
            innerIterMax = 3))
        solve!(solver)
        checkSolution(solver, demands, Q, 12)
    end

    @testset "ILS solves a small CVRP" begin
        solver, demands, Q = buildSolver(res -> ILSAlgorithm(res;
            acceptCriteria = ILS.AcceptBest(),
            stopCriteria = ILS.ByIterMax(5)))
        solve!(solver)
        checkSolution(solver, demands, Q, 12)
    end
end
