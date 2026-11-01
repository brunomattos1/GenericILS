# Tutorial: solving a CVRP with GenericILS

This tutorial walks through using `GenericILS` to solve a Capacitated
Vehicle Routing Problem (CVRP). The goal isn't the CVRP itself — it's to
show every piece a new problem needs to provide, and how those pieces fit
together.

For a deeper reference on each piece, see:

- [Accept Criteria](accept-criteria.md)
- [Stop Criteria](stop-criteria.md)
- [Penalty Manager](penalty-manager.md)
- [Algorithms](algorithms.md)

The complete, working example used here lives in
[`Applications/CVRP/mainCVRP.jl`](../Applications/CVRP/mainCVRP.jl) and
[`Applications/CVRP/resourcesCVRP.jl`](../Applications/CVRP/resourcesCVRP.jl).

## Overview

`GenericILS` splits the problem into two responsibilities that you, as the
package's user, need to fill in:

1. **What counts as a feasible solution for your problem** — capacity, time
   windows, fleet cost, etc. This is the `CustomResource` (see
   [Defining the problem's resource](#step-1--defining-the-problems-resource)).
2. **How to search for that solution** — which metaheuristic to run, with
   which accept criterion, stop criterion, and infeasibility penalty
   scheme. This is the `Solver` plus an `Algorithm` (`NILSAlgorithm` or
   `ILSAlgorithm`).

The package already provides, ready to reuse for any problem: forward/backward
labels, local search (RVND) over a set of neighborhoods (`TwoOptStar`,
`IntraShift`, `InterShift`, `InterSwap`), greedy initial construction
(`constructSol!`), and exact resolution of the partitioning subproblem via
Set Partitioning (JuMP/MIP) at the end of the search.

## Step 1 — Defining the problem's resource

Every problem needs a `CustomResource`: a `struct` representing your
problem's "custom" resource (for CVRP, vehicle capacity) that knows how that
resource evolves as it traverses an arc `(i, j)`.

```julia
using GenericILS

struct CustomResource <: AbstractResource
    d::Matrix{Float64}   # demand "picked up" when entering arc (i, j)
    Q::Float64            # vehicle capacity
end
```

Next you need to tell the package:

- whether the problem is **symmetric** (`d[i,j] == d[j,i]` for all relevant
  distances/resources) — this enables some local search optimizations;
- whether your `CustomResource` represents **cost** (i.e. whether the
  label's value should enter the objective function) or is purely a
  feasibility constraint (e.g. capacity, with no associated cost).

```julia
GenericILS.isSymmetric() = false
GenericILS.isCostResource() = false
```

> This form takes no arguments, so it is global to the process: it works for
> one problem type per Julia process. To run problems with different answers in
> the same process, define the flags for your resource type instead —
> `GenericILS.isCostResource(::CustomResource) = false` and
> `GenericILS.isSymmetric(::CustomResource) = false`. A resource without its
> own methods uses the global form. A resource that learns the answer when it
> is created (one read from C++, for instance) must use the per-type form.

### The label's state

The labeling algorithm needs to know, for your resource, what "state"
accumulates along a partial route — for CVRP, that's just the accumulated
demand:

```julia
struct ForwardState
    q::Float64
end

struct BackwardState
    q::Float64
end
```

### Initializing and extending the label

Finally, you implement 3 functions the package calls internally during the
search (construction, labeling, local search — everything reuses these 3
functions):

```julia
import GenericILS: isSymmetric, isCostResource, initStateForward, initStateBackward,
    extendAlongArc, concatenationCost, AbstractResource, ForwardLabel, BackwardLabel

# initial state (at the depot) and its cost
function initStateForward(res::CustomResource)
    return (ForwardState(0.0), 0.0)
end

function initStateBackward(res::CustomResource)
    return (BackwardState(0.0), 0.0)
end

# how the state (and cost) evolves when traversing arc a = (i, j)
function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
    q_custom = label.state.q + res.d[(1, a[2])...]
    if q_custom > res.Q + 1e-5
        return (ForwardState(q_custom), Inf)   # Inf = violates capacity
    else
        return (ForwardState(q_custom), 0.0)   # no cost (isCostResource() == false)
    end
end

function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
    q_custom = label.state.q + res.d[(1, a[1])...]
    if q_custom > res.Q + 1e-5
        return (BackwardState(q_custom), Inf)
    else
        return (BackwardState(q_custom), 0.0)
    end
end

# how to concatenate a forward label with a backward one (used by local
# search to evaluate moves without recomputing the whole route)
function concatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    if forwardLabel.state.q + backwardLabel.state.q > res.Q + 1e-5
        return (ForwardState(forwardLabel.state.q + backwardLabel.state.q), Inf)
    else
        return (ForwardState(forwardLabel.state.q + backwardLabel.state.q), 0.0)
    end
end
```

That's everything the package needs to know about your problem. From here
on, it's all configuration — no other domain code is needed.

#### Optional: a step for the vertex

Data that belongs to a vertex rather than to an arc (a demand, a service time, a profit) can go in a fourth
function, `extendToVertex(res, label, v)`. The package calls it right after `extendAlongArc`, on the label that
`extendAlongArc` produced, so `label.state` and `label.cost` already include the arc. It is the same logic as the
vertex step of the MetaSolver resources of VRPSolver. `v` is the vertex that enters the label, as a matrix index
like the arc: `a[2]` after a forward extension and `a[1]` after a backward one. It returns `(state, cost)` with the
accumulated cost, as `extendAlongArc` does, and `Inf` when the vertex violates a rule. The default does nothing,
so a resource that does not define it is not affected.

The capacity above can be written this way, with the arc doing nothing:

```julia
import GenericILS: extendToVertex

extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int}) = (label.state, label.cost)
extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int}) = (label.state, label.cost)

function extendToVertex(res::CustomResource, label::ForwardLabel, v::Int)
    q = label.state.q + res.demand[v]              # a vector of demands, by matrix index
    q > res.Q + 1e-5 ? (ForwardState(q), Inf) : (ForwardState(q), 0.0)
end

function extendToVertex(res::CustomResource, label::BackwardLabel, v::Int)
    q = label.state.q + res.demand[v]
    q > res.Q + 1e-5 ? (BackwardState(q), Inf) : (BackwardState(q), 0.0)
end
```

Every vertex of a route but its first is entered exactly once, by the forward labels up to the vertex where the
two halves are joined and by the backward ones after it, so `concatenationCost` has nothing to add for the vertices.

## Step 2 — Building the `Resources`

`GenericILS` always works with **three** resources per arc simultaneously:
your `CustomResource`, and up to two generic `StandardResource` (ready to
use, useful for common constraints like time windows or a second capacity,
without writing custom labels for them). If you don't need a
`StandardResource`, just leave it "loose" (with no effective constraint).

```julia
using CVRPLIB

cvrp = CVRPLIB.readCVRP(instance_path)
dist = Float64.(cvrp.weights)
demands = push!(Float64.(cvrp.demand), 0.0)

# "demand picked up entering arc (i,j)" matrix: depends only on destination j
d = createArcDemands(demands)   # your own utility function

customRes = CustomResource(d, cvrp.capacity)

n = length(customers) + 1
stdRes1 = StandardResource{1}(d, zeros(Float64, n), Float64[cvrp.capacity for _ in 1:n])   # mirrors capacity
stdRes2 = StandardResource{2}(zeros(Float64, n, n), zeros(Float64, n), fill(Inf, n))         # loose (unused)

res = Resources(customRes, stdRes1, stdRes2)
```

## Step 3 — Choosing the algorithm

Every `Solver` needs an `Algorithm`. The package ships two ready to use —
`NILSAlgorithm` (Nested ILS: two layers of perturbation, with Set
Partitioning at the end) and `ILSAlgorithm` (single-layer ILS) — see
[Algorithms](algorithms.md) for details.

```julia
algorithm = NILSAlgorithm(res;
    acceptCriteria = NILS.AcceptBest(),
    stopCriteria = NILS.ByIterMax(200),
    diversification = NILS.Diversification(outerShift = 1, outerSwap = 0, innerShift = 2, innerSwap = 0),
    innerIterMax = 5
)
```

Note that `res` is passed to the algorithm's constructor — not because the
algorithm "uses" the resources directly, but because the package needs to
infer, from it, the concrete forward/backward label types for your problem
in order to preallocate the algorithm's internal structures with the right
type (a Julia performance requirement, not a domain requirement).

`NILS.AcceptBest()`, `NILS.ByIterMax(...)` and `NILS.Diversification(...)`
are qualified with the `NILS.` prefix because each algorithm (`NILS`,
`ILS`) defines its **own** vocabulary of criteria — even though the names
coincide (`Metropolis`, `Diversification`, etc.), they are different,
independent types. See why in
[Algorithms](algorithms.md#why-are-nilsmetropolis-and-ilsmetropolis-different-types).

## Step 4 — Building the `Solver`

```julia
solver = Solver(
    res = res,
    data = ProblemData(customers, dist, maxNbRoute),
    neighborhoods = (
        TwoOptStar(),
        IntraShift(),
        InterShift{1}(),
        InterShift{2}(),
        InterSwap{1, 1}(),
        InterSwap{2, 1}(),
        InterSwap{2, 2}()
    ),
    algorithm = algorithm
)
```

- `data` is a `ProblemData(vertices, costMatrix, maxNbRoutes)` — the
  customers (excluding the depot), the distance matrix, and a route count
  cap. Every customer is a `Vertex(id, group)` (see below).
- `neighborhoods` is the tuple of local search neighborhoods that the
  algorithm's `RVND!` will consider, in whatever order/composition you
  want. Neighborhoods available today: `TwoOptStar`, `IntraShift`,
  `InterShift{k}` (moves a block of `k` customers between routes),
  `InterSwap{k1,k2}` (swaps blocks of size `k1`/`k2` between routes),
  `IntraGroupExchange` (replaces a visited vertex by an unvisited member of its group; only useful when
  some group has `numVisits` below its size), `AddOptional` and `RemoveOptional` (visit or drop optional
  vertices; only useful when there are any, see below).

### Visit groups

Each `Vertex` belongs to a `Group(groupId, numVisits, sameRoute)`: the solution must visit exactly
`numVisits` of the vertices of the group (each at most once), all in the same route when `sameRoute`
is `true`. A classic VRP puts every customer alone in its own group with `numVisits = 1`:

```julia
customers = [Vertex(i, Group(i, 1, false)) for i in 1:n]
```

Other shapes: one vertex per cluster (generalized VRP) is `numVisits = 1` over the cluster's vertices;
a whole cluster served by one vehicle is `numVisits = length(cluster)` with `sameRoute = true`.
Group ids must be `1:G`, and every vertex of a group must carry the same `Group`.
`groupViolations(solver.data, sol)` lists the group constraints a solution breaks.

The construction, the neighborhoods and the perturbation never split a group that has `sameRoute` and
needs several visits: a visit of such a group cannot move to another route alone, only together with the
rest of the group (as a block of `InterShift`/`InterSwap` that holds all its visits, or in a tail of
`TwoOptStar`). With `IntraGroupExchange` it can still be replaced, in place, by an unvisited member.

### Optional vertices

A vertex can be optional: `Vertex(id, group; isOptional = true)`. An optional vertex does not count for the
`numVisits` of its group, which is about the regular vertices only, and the solution may visit it or not.
`optionalVertex(id)` makes an optional customer alone in its own group, which needs no visit
(`numVisits = 0`); `numVisits` is `0` for any group with no regular vertex. If the group has to stay in one
route (`sameRoute`), its optional vertices go along with the rest.

Whether visiting an optional vertex is worth it is up to the problem: make the resource reward it, for instance
a profit as a negative cost for entering the vertex (`isCostResource() = true`, the returned cost being the
accumulated `-profit`). The construction visits an optional vertex only if that lowers the objective, and
`AddOptional` and `RemoveOptional` do the same during the search. This models prize-collecting and
orienteering problems; limits such as a maximum route length are ordinary resource constraints.

```julia
customers = vcat([Vertex(i, Group(i, 1, false)) for i in 1:m],      # mandatory
                 [optionalVertex(i) for i in m+1:n])                # visited only if they pay
neighborhoods = (TwoOptStar(), IntraShift(), InterShift{1}(), InterSwap{1,1}(), AddOptional(), RemoveOptional())
```

`res`, `algorithm` and `neighborhoods` fix type parameters of the `Solver`
— that's why they're passed at construction time. Everything else (seed,
`PenaltyManager`, MIP solver) is adjusted afterward via setters, without
needing to rebuild the `Solver`:

```julia
setSeed!(solver, Random.MersenneTwister(2))

setPenaltyManager!(solver, StandardPenaltyManager(
    penaltyCustom = 1.0, penaltyCustomIncrease = 0.01, penaltyCustomDecrease = 0.01,
    penaltyStandard1 = 1.0, penaltyStandard1Increase = 0.01, penaltyStandard1Decrease = 0.01,
    penaltyStandard2 = 0.0, penaltyStandard2Increase = 0.0, penaltyStandard2Decrease = 0.0
))

setMIPSolver!(solver, CPLEX.Optimizer)   # or HiGHS.Optimizer (default), any JuMP solver
```

See [Penalty Manager](penalty-manager.md) for what each
`penaltyCustom`/`penaltyStandard1`/`penaltyStandard2` parameter controls.

## Step 5 — Running it

```julia
solve!(solver)

sol = getBestSol(solver)   # prints the routes and cost, and returns the Solution
println(sol.cost)
```

`solve!(solver)` dispatches to `run!(solver.algorithm, solver)` — the same
`solve!` works for any `Algorithm`, present or future.

## Full example

```julia
using GenericILS
using Random
using CVRPLIB
using CPLEX

# --- Step 1: problem definition (normally in a separate file) ---
struct CustomResource <: AbstractResource
    d::Matrix{Float64}
    Q::Float64
end

GenericILS.isSymmetric() = false
GenericILS.isCostResource() = false

struct ForwardState
    q::Float64
end
struct BackwardState
    q::Float64
end

import GenericILS: isSymmetric, isCostResource, initStateForward, initStateBackward,
    extendAlongArc, concatenationCost, AbstractResource, ForwardLabel, BackwardLabel

initStateForward(res::CustomResource) = (ForwardState(0.0), 0.0)
initStateBackward(res::CustomResource) = (BackwardState(0.0), 0.0)

function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
    q = label.state.q + res.d[(1, a[2])...]
    q > res.Q + 1e-5 ? (ForwardState(q), Inf) : (ForwardState(q), 0.0)
end

function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
    q = label.state.q + res.d[(1, a[1])...]
    q > res.Q + 1e-5 ? (BackwardState(q), Inf) : (BackwardState(q), 0.0)
end

function concatenationCost(res::CustomResource, v::Int, fwd::ForwardLabel, bwd::BackwardLabel)
    q = fwd.state.q + bwd.state.q
    q > res.Q + 1e-5 ? (ForwardState(q), Inf) : (ForwardState(q), 0.0)
end

# --- Steps 2-5: setup and execution ---
Random.seed!(0)
cvrp = CVRPLIB.readCVRP("A-n37-k5.vrp")
dist = Float64.(cvrp.weights)
# one group per customer that needs one visit: a classic VRP (see "Visit groups" in Step 4)
customers = [Vertex(i, Group(i, 1, false)) for i in 1:size(dist, 1) - 1]
maxNbRoute = ceil(Int, sum(cvrp.demand) / cvrp.capacity) + 3

demands = push!(Float64.(cvrp.demand), 0.0)
n = length(customers) + 1
d = zeros(Float64, n, n)
for i in 1:n, j in 1:n
    i != j && (d[i, j] = j == 1 ? 0.0 : demands[j])
end

customRes = CustomResource(d, cvrp.capacity)
stdRes1 = StandardResource{1}(d, zeros(Float64, n), Float64[cvrp.capacity for _ in 1:n])
stdRes2 = StandardResource{2}(zeros(Float64, n, n), zeros(Float64, n), fill(Inf, n))
res = Resources(customRes, stdRes1, stdRes2)

algorithm = NILSAlgorithm(res;
    acceptCriteria = NILS.AcceptBest(),
    stopCriteria = NILS.ByIterMax(200),
    diversification = NILS.Diversification(outerShift = 1, outerSwap = 0, innerShift = 2, innerSwap = 0),
    innerIterMax = 5
)

solver = Solver(
    res = res,
    data = ProblemData(customers, dist, maxNbRoute),
    neighborhoods = (TwoOptStar(), IntraShift(), InterShift{1}(), InterShift{2}(), InterSwap{1,1}(), InterSwap{2,1}(), InterSwap{2,2}()),
    algorithm = algorithm
)

setSeed!(solver, Random.MersenneTwister(2))
setPenaltyManager!(solver, StandardPenaltyManager(
    penaltyCustom = 1.0, penaltyCustomIncrease = 0.01, penaltyCustomDecrease = 0.01,
    penaltyStandard1 = 1.0, penaltyStandard1Increase = 0.01, penaltyStandard1Decrease = 0.01,
    penaltyStandard2 = 0.0, penaltyStandard2Increase = 0.0, penaltyStandard2Decrease = 0.0
))
setMIPSolver!(solver, CPLEX.Optimizer)

solve!(solver)
sol = getBestSol(solver)
```

### Expected output

Running the example above against `A-n37-k5.vrp` (seed `2`) prints the
routes found and their total cost — `getBestSol` prints as a side effect,
then returns the `Solution`:

```
Route #1: 0 -> 34 -> 26 -> 18 -> 35 -> 25 -> 8 -> 27 -> 11 -> 9 -> 24 -> 3 -> 0
Route #2: 0 -> 16 -> 21 -> 0
Route #3: 0 -> 17 -> 14 -> 23 -> 20 -> 19 -> 2 -> 12 -> 1 -> 0
Route #4: 0 -> 7 -> 4 -> 33 -> 5 -> 6 -> 10 -> 13 -> 22 -> 0
Route #5: 0 -> 15 -> 30 -> 31 -> 28 -> 32 -> 29 -> 36 -> 0
Cost: 669.0
```

(Empty routes — depot-to-depot with no customers — are skipped in the
printout; the numbering only counts non-empty routes.)

## Next steps

- To change how candidate solutions are accepted, see
  [Accept Criteria](accept-criteria.md).
- To control when the search stops, see [Stop Criteria](stop-criteria.md).
- To understand how the package handles infeasibility (penalization instead
  of rejection), see [Penalty Manager](penalty-manager.md).
- To choose between `NILSAlgorithm`/`ILSAlgorithm`, or to learn how to
  implement your own metaheuristic, see [Algorithms](algorithms.md).
