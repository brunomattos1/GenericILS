# Run in its own Julia process by testOptionalResource.jl: isSymmetric()/isCostResource() are global to a
# process, and the other tests need isCostResource() == false.
#
# Problem: a CVRP with optional customers whose profit is paid by the resource (a negative cost for entering
# the vertex), while the distance stays in the cost matrix. The searches of optional visits must then work through
# the label cost, which disables the pruning by distance.
using Random
using GenericILS
using HiGHS
import GenericILS: isSymmetric, isCostResource, initStateForward, initStateBackward,
    extendAlongArc, concatenationCost

struct ProfitCost <: AbstractResource
    profit::Vector{Float64}          # indexed by matrix index (id + 1); the depot has none
end
struct FProfit
    c::Float64
end
struct BProfit
    c::Float64
end

GenericILS.isSymmetric() = false
GenericILS.isCostResource() = true
initStateForward(::ProfitCost) = (FProfit(0.0), 0.0)
initStateBackward(::ProfitCost) = (BProfit(0.0), 0.0)

# arc (i, j) costs -profit[j]: forward appends j, backward arc (next, cur) prepends cur and enters `next`
extendAlongArc(res::ProfitCost, l::ForwardLabel, a::Tuple{Int,Int}) =
    (FProfit(l.state.c - res.profit[a[2]]), l.state.c - res.profit[a[2]])
extendAlongArc(res::ProfitCost, l::BackwardLabel, a::Tuple{Int,Int}) =
    (BProfit(l.state.c - res.profit[a[1]]), l.state.c - res.profit[a[1]])
concatenationCost(res::ProfitCost, v::Int, f::ForwardLabel, b::BackwardLabel) =
    (FProfit(f.state.c + b.state.c), f.state.c + b.state.c)

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
res = Resources(
    ProfitCost(profit),
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

routeDist(v) = sum(dist[v[k]+1, v[k+1]+1] for k in 1:length(v)-1)
objective(routes) = sum(routeDist(v) - sum(profit[x+1] for x in v[2:end-1]; init = 0.0) for v in routes if length(v) > 2; init = 0.0)

# ---- 1. the whole algorithm: valid solution whose cost is distance minus the profit it collects
redirect_stdout(devnull) do
    solve!(solver)
end
best = solver.bestFeasSol
routes = [r.visits for r in best.routes if length(r.visits) > 2]
visited = vcat([v[2:end-1] for v in routes]...)
@assert groupViolations(solver.data, best) == String[] "group constraints broken: $(groupViolations(solver.data, best))"
@assert all(i -> i in visited, 1:nRegular) "a regular vertex is not visited"
@assert length(unique(visited)) == length(visited)
@assert abs(best.cost - objective(routes)) < 1e-6 "cost $(best.cost) differs from distance - profit $(objective(routes))"
@assert length(routes) <= 3
collected = count(i -> i > nRegular, visited)
@assert collected > 0 "no optional vertex was visited although they pay"

# ---- 2. the searches of optional visits stop at a point where no single insertion or removal helps
sol = constructSol!(solver)
for r in eachindex(sol.routes), i in reverse(2:length(sol.routes[r].visits)-1)      # leave the optional vertices out
    v = sol.routes[r].visits[i]
    v > nRegular || continue
    cut = GenericILS.removeOptionalCost(sol.dist, solver.data.costMatrix, sol.routes[r].visits, i)
    GenericILS.apply!(RemoveOptional(), solver, sol, GenericILS.BestOptional(0.0, cut, r, i, v))
end
@assert all(v -> v <= nRegular, vcat([r.visits[2:end-1] for r in sol.routes]...))
for _ in 1:50
    a = search!(AddOptional(), solver, sol)
    b = search!(RemoveOptional(), solver, sol)
    (a || b) || break
end
@assert !search!(AddOptional(), solver, sol) && !search!(RemoveOptional(), solver, sol)
@assert any(r -> any(v -> v > nRegular, r.visits), sol.routes) "AddOptional brought nothing back"
current = [r.visits for r in sol.routes]
@assert abs(sol.cost - objective(current)) < 1e-6 "label cost $(sol.cost) differs from distance - profit $(objective(current))"
base = objective(current)
isVisited(x) = any(v -> x in v, current)
for u in nRegular+1:n
    if isVisited(u)
        for (r, v) in enumerate(current), i in 2:length(v)-1
            v[i] == u || continue
            trial = [copy(x) for x in current]
            deleteat!(trial[r], i)
            @assert objective(trial) >= base - 1e-6 "removing $u would improve the objective"
        end
    else
        for (r, v) in enumerate(current), j in 2:length(v)
            trial = [copy(x) for x in current]
            insert!(trial[r], j, u)
            @assert objective(trial) >= base - 1e-6 "inserting $u at $j of route $r would improve the objective"
        end
    end
end

println("OPTIONAL_RESOURCE_OK")
