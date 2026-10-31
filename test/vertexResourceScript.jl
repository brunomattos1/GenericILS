# Run in its own Julia process by testExtendToVertex.jl: isSymmetric()/isCostResource() are global to a process, and
# the other tests need isCostResource() == false.
#
# Problem: the CVRP with optional customers of optionalResourceScript.jl, whose profit is paid by the resource. It is
# written twice, in the arc step (a cost for entering the vertex through the arc) and in the vertex step, and the two
# must lead to the same search, the same solution and the cost distance - profit.
using Random
using GenericILS
using HiGHS
import GenericILS: isSymmetric, isCostResource, initStateForward, initStateBackward,
    extendAlongArc, extendToVertex, concatenationCost

struct ProfitInArc <: AbstractResource
    profit::Vector{Float64}          # by matrix index (id + 1); the depot has none
end
struct ProfitInVertex <: AbstractResource
    profit::Vector{Float64}
end
struct FProfit
    c::Float64
end
struct BProfit
    c::Float64
end

GenericILS.isSymmetric() = false
GenericILS.isCostResource() = true
for R in (ProfitInArc, ProfitInVertex)
    @eval initStateForward(::$R) = (FProfit(0.0), 0.0)
    @eval initStateBackward(::$R) = (BProfit(0.0), 0.0)
    @eval concatenationCost(res::$R, v::Int, f::ForwardLabel, b::BackwardLabel) =
        (FProfit(f.state.c + b.state.c), f.state.c + b.state.c)
end

# arc form: the arc (i, j) costs -profit[j]; backward, the arc (next, cur) enters `next`
extendAlongArc(res::ProfitInArc, l::ForwardLabel, a::Tuple{Int,Int}) =
    (FProfit(l.state.c - res.profit[a[2]]), l.state.c - res.profit[a[2]])
extendAlongArc(res::ProfitInArc, l::BackwardLabel, a::Tuple{Int,Int}) =
    (BProfit(l.state.c - res.profit[a[1]]), l.state.c - res.profit[a[1]])

# vertex form: the arc is free and the vertex that enters the label pays its profit
extendAlongArc(::ProfitInVertex, l::ForwardLabel, ::Tuple{Int,Int}) = (l.state, l.cost)
extendAlongArc(::ProfitInVertex, l::BackwardLabel, ::Tuple{Int,Int}) = (l.state, l.cost)
extendToVertex(res::ProfitInVertex, l::ForwardLabel, v::Int) =
    (FProfit(l.state.c - res.profit[v]), l.cost - res.profit[v])
extendToVertex(res::ProfitInVertex, l::BackwardLabel, v::Int) =
    (BProfit(l.state.c - res.profit[v]), l.cost - res.profit[v])

n = 14
nRegular = 7
Q = 1000.0                     # capacity never binds: every solution is feasible, the objective is distance - profit
rng = MersenneTwister(21)
xy = rand(rng, n + 1, 2) .* 100
dist = [sqrt(sum((xy[i, :] .- xy[j, :]) .^ 2)) for i in 1:n+1, j in 1:n+1]
demands = vcat(0.0, Float64.(rand(rng, 1:10, n)))
profit = vcat(0.0, zeros(nRegular), rand(rng, 20.0:5.0:90.0, n - nRegular))     # only the optional ones have profit
d = zeros(n + 1, n + 1)
for i in 1:n+1, j in 2:n+1
    i != j && (d[i, j] = demands[j])
end
vertices = vcat([Vertex(i, Group(i, 1, false)) for i in 1:nRegular], [optionalVertex(i) for i in nRegular+1:n])

function solveWith(custom)
    res = Resources(
        custom,
        StandardResource{1}(d, zeros(n + 1), fill(Q, n + 1)),
        StandardResource{2}(zeros(n + 1, n + 1), zeros(n + 1), fill(Inf, n + 1)),
    )
    algorithm = NILSAlgorithm(res;
        acceptCriteria = NILS.AcceptBest(),
        stopCriteria = NILS.ByIterMax(8),
        diversification = NILS.Diversification(outerShift = 1, outerSwap = 1, innerShift = 2, innerSwap = 1),
        innerIterMax = 3)
    neighborhoods = (TwoOptStar(), IntraShift(), InterShift{1}(), InterShift{2}(), InterSwap{1,1}(), InterSwap{2,1}(),
                     InterSwap{2,2}(), AddOptional(), RemoveOptional())
    solver = Solver(res = res, data = ProblemData(vertices, dist, 3), neighborhoods = neighborhoods, algorithm = algorithm)
    setSeed!(solver, MersenneTwister(5))
    @assert canPruneByDist(solver) == false      # pruning by distance is unsafe with a cost resource
    redirect_stdout(devnull) do
        solve!(solver)
    end
    return solver
end

routeDist(v) = sum(dist[v[k]+1, v[k+1]+1] for k in 1:length(v)-1)
objective(routes) = sum(routeDist(v) - sum(profit[x+1] for x in v[2:end-1]; init = 0.0) for v in routes if length(v) > 2; init = 0.0)

inArc = solveWith(ProfitInArc(profit))
inVertex = solveWith(ProfitInVertex(profit))

routes = [r.visits for r in inVertex.bestFeasSol.routes if length(r.visits) > 2]
visited = vcat([v[2:end-1] for v in routes]...)
@assert groupViolations(inVertex.data, inVertex.bestFeasSol) == String[]
@assert all(i -> i in visited, 1:nRegular) "a regular vertex is not visited"
@assert count(i -> i > nRegular, visited) > 0 "no optional vertex was visited although they pay"
@assert abs(inVertex.bestFeasSol.cost - objective(routes)) < 1e-6 "cost $(inVertex.bestFeasSol.cost) differs from distance - profit $(objective(routes))"

@assert inVertex.bestFeasSol.cost == inArc.bestFeasSol.cost "the vertex form found $(inVertex.bestFeasSol.cost), the arc form $(inArc.bestFeasSol.cost)"
@assert [r.visits for r in inVertex.bestFeasSol.routes] == [r.visits for r in inArc.bestFeasSol.routes]

println("VERTEX_RESOURCE_OK")
