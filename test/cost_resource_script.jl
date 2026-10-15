# Run in its own Julia process by test_cost_resource.jl: isSymmetric()/isCostResource() are global
# to a process, and the other tests need isCostResource() == false.
#
# Problem: CVRP where the whole route cost lives in the custom resource (like HFVRP/MDVRP), so
# ProblemData receives a zero cost matrix and the solver cost must equal the true distance.
using Random
using GenericILS
using HiGHS
import GenericILS: isSymmetric, isCostResource, initStateForward, initStateBackward,
    extendAlongArc, concatenationCost

struct DistanceCost <: AbstractResource
    dist::Matrix{Float64}
end
struct FState
    D::Float64   # accumulated distance
end
struct BState
    D::Float64
end

GenericILS.isSymmetric() = false
GenericILS.isCostResource() = true
initStateForward(::DistanceCost) = (FState(0.0), 0.0)
initStateBackward(::DistanceCost) = (BState(0.0), 0.0)

# forward arc (i, j): j is appended, the label ends at j
extendAlongArc(res::DistanceCost, l::ForwardLabel, a::Tuple{Int,Int}) =
    (FState(l.state.D + res.dist[a[1], a[2]]), l.state.D + res.dist[a[1], a[2]])
# backward arc (next, cur): cur is prepended, the travel direction is cur -> next
extendAlongArc(res::DistanceCost, l::BackwardLabel, a::Tuple{Int,Int}) =
    (BState(l.state.D + res.dist[a[2], a[1]]), l.state.D + res.dist[a[2], a[1]])
# forward label at x (route up to x) + backward label at x (route after x)
concatenationCost(res::DistanceCost, v::Int, f::ForwardLabel, b::BackwardLabel) =
    (FState(f.state.D + b.state.D), f.state.D + b.state.D)

n = 14
Q = 30.0
rng = MersenneTwister(11)
xy = rand(rng, n + 1, 2) .* 100
dist = [sqrt(sum((xy[i, :] .- xy[j, :]) .^ 2)) for i in 1:n+1, j in 1:n+1]
demands = vcat(0.0, Float64.(rand(rng, 1:10, n)))
d = zeros(n + 1, n + 1)
for i in 1:n+1, j in 2:n+1
    i != j && (d[i, j] = demands[j])
end

res = Resources(
    DistanceCost(dist),
    StandardResource{1}(d, zeros(n + 1), fill(Q, n + 1)),
    StandardResource{2}(zeros(n + 1, n + 1), zeros(n + 1), fill(Inf, n + 1)),
)
algorithm = NILSAlgorithm(res;
    acceptCriteria = NILS.AcceptBest(),
    stopCriteria = NILS.ByIterMax(6),
    diversification = NILS.Diversification(outerShift = 1, outerSwap = 1, innerShift = 2, innerSwap = 1),
    innerIterMax = 3)
solver = Solver(
    res = res,
    data = ProblemData([Vertex(i) for i in 1:n], zeros(n + 1, n + 1), ceil(Int, sum(demands) / Q) + 3),
    neighborhoods = (TwoOptStar(), IntraShift(), InterShift{1}(), InterShift{2}(),
        InterSwap{1,1}(), InterSwap{2,1}(), InterSwap{2,2}()),
    algorithm = algorithm)
setSeed!(solver, MersenneTwister(5))

@assert canPruneByDist(solver) == false      # pruning by distance is unsafe with a cost resource

redirect_stdout(devnull) do
    solve!(solver)
end
sol = solver.bestFeasSol

visited = Int[]
total = 0.0
for r in sol.routes
    v = r.visits
    length(v) <= 2 && continue
    append!(visited, v[2:end-1])
    @assert sum(demands[v[2:end-1] .+ 1]) <= Q + 1e-6 "capacity violated"
    global total += sum(dist[v[k]+1, v[k+1]+1] for k in 1:length(v)-1)
end
@assert sort(visited) == collect(1:n) "customers not visited exactly once"
@assert isapprox(sol.cost, total; atol = 1e-6) "solver cost $(sol.cost) != true distance $total"

# the helpers that depend on the cost resource
user = createSolution(solver, [[0, 1, 2, 0], [0, 3, 0]])
expected = dist[1, 2] + dist[2, 3] + dist[3, 1] + dist[1, 4] + dist[4, 1]
@assert isapprox(user.cost, expected; atol = 1e-6) "createSolution cost mismatch"
@assert objectiveValue(solver, solver.bestFeasSol) >= 0

# Move evaluation with a cost resource: the label cost predicted for a move must equal the true
# distance of the resulting route (this resource never returns Inf, so it is exact for every move).
function labelled(solver, routes)
    sol = newSolution(solver)
    sol.routes = [newRoute(solver, copy(v)) for v in routes]
    computeLabels(solver, sol)
    return sol
end
trueCost(visits) = sum(dist[visits[k]+1, visits[k+1]+1] for k in 1:length(visits)-1)

checks = 0
for trial in 1:5
    perm = shuffle(rng, collect(1:n))
    cuts = sort!(rand(rng, 0:n, 2))
    routes = [vcat(0, perm[1:cuts[1]], 0), vcat(0, perm[cuts[1]+1:cuts[2]], 0), vcat(0, perm[cuts[2]+1:end], 0)]
    lsol = labelled(solver, routes)
    for r in 1:3
        v = routes[r]
        for k in 1:2, pos in 2:length(v)-k
            got = GenericILS.infeasArcsRemovalK(solver, lsol, r, pos, k)
            @assert isapprox(got[2], trueCost(vcat(v[1:pos-1], v[pos+k:end])); atol = 1e-6) "removal label cost"
            global checks += 1
        end
        others = [c for c in 1:n if c ∉ v]
        length(others) >= 2 || continue
        for pos in 2:length(v)
            got = GenericILS.infeasArcsInsertionK(solver, lsol, r, others[1:2], pos)
            @assert isapprox(got[2], trueCost(vcat(v[1:pos-1], others[1:2], v[pos:end])); atol = 1e-6) "insertion label cost"
            global checks += 1
        end
        for pos in 2:length(v)-1
            got = GenericILS.infeasArcsReplaceBlockK(solver, lsol, r, pos, 1, others[1:2])
            @assert isapprox(got[2], trueCost(vcat(v[1:pos-1], others[1:2], v[pos+1:end])); atol = 1e-6) "replace label cost"
            global checks += 1
        end
    end
    for rH in 1:3, rT in 1:3
        rH == rT && continue
        head, tail = routes[rH], routes[rT]
        for posH in 1:length(head)-1, posT in 1:length(tail)-1
            got = GenericILS.infeasArcs2optStar(solver, lsol, rH, rT, posH, posT)
            @assert isapprox(got[2], trueCost(vcat(head[1:posH], tail[posT+1:end])); atol = 1e-6) "2-opt* label cost"
            global checks += 1
        end
    end
end
@assert checks > 100

println("COST_RESOURCE_OK")
