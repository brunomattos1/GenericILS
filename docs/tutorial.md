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
[`Applications/CVRP/mainCVRP_pkg.jl`](../Applications/CVRP/mainCVRP_pkg.jl) and
[`Applications/CVRP/resourcesCVRP_pkg.jl`](../Applications/CVRP/resourcesCVRP_pkg.jl).

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

> These two functions take no arguments — they're global to the process. If
> you plan to run more than one problem type in the same Julia process, each
> one needs its own process/environment.

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
  cap.
- `neighborhoods` is the tuple of local search neighborhoods that the
  algorithm's `RVND!` will consider, in whatever order/composition you
  want. Neighborhoods available today: `TwoOptStar`, `IntraShift`,
  `InterShift{k}` (moves a block of `k` customers between routes),
  `InterSwap{k1,k2}` (swaps blocks of size `k1`/`k2` between routes).

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
customers = [Vertex(i) for i in 1:size(dist, 1) - 1]
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
