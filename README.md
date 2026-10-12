# GenericILS

`GenericILS` is a Julia package for building Iterated Local Search
metaheuristics for Vehicle Routing Problems (VRP) and related resource-
constrained routing problems, using a label-correcting (forward/backward)
approach for local search evaluation.

The package separates two concerns:

- **What counts as a feasible solution for your problem** — capacity, time
  windows, or any custom resource — which you define once, as a small set of
  functions.
- **How to search for that solution** — local search over a configurable
  set of neighborhoods, plus a pluggable metaheuristic (`NILSAlgorithm`,
  `ILSAlgorithm`, or your own) with configurable accept/stop criteria and
  infeasibility penalization.

Everything reusable — label computation, local search neighborhoods
(`TwoOptStar`, `IntraShift`, `InterShift`, `InterSwap`), greedy construction,
and an exact Set Partitioning step (via JuMP/MIP) — is provided by the
package. You only implement the parts specific to your problem.

## Installation

`GenericILS` isn't registered in the General registry yet, so install it
straight from GitHub (Julia 1.9 or newer):

```julia
using Pkg
Pkg.add(url="https://github.com/brunomattos1/GenericILS")
```

To work on the package itself, clone the repository and use:

```julia
using Pkg
Pkg.develop(path="/path/to/GenericILS")
```

To run the test suite: `Pkg.test("GenericILS")`.

The applications under `Applications/` depend on the local checkout through a
`[sources]` entry in their `Project.toml` (this needs Julia 1.11 or newer):

```toml
[deps]
GenericILS = "264616b9-4ba5-42b2-a769-b189edc0be0a"

[sources]
GenericILS = {path = "../.."}
```

## Quickstart

```julia
using GenericILS

# 1. Define your problem's custom resource (see docs/tutorial.md)
struct CustomResource <: AbstractResource
    d::Matrix{Float64}
    Q::Float64
end
GenericILS.isSymmetric() = false
GenericILS.isCostResource() = false
# ... initStateForward, initStateBackward, extendAlongArc, concatenationCost

# 2. Build the resources and pick an algorithm
res = Resources(CustomResource(d, Q), stdRes1, stdRes2)
algorithm = NILSAlgorithm(res;
    acceptCriteria = NILS.AcceptBest(),
    stopCriteria = NILS.ByIterMax(200)
)

# 3. Build the solver and run it
solver = Solver(
    res = res,
    data = ProblemData(customers, costMatrix, maxNbRoutes),
    neighborhoods = (TwoOptStar(), IntraShift(), InterShift{1}(), InterSwap{1,1}()),
    algorithm = algorithm
)
solve!(solver)
sol = getBestSol(solver)
```

A complete, runnable walkthrough of this (a CVRP solved end to end) is in
[docs/tutorial.md](docs/tutorial.md).

## Documentation

- **[Tutorial](docs/tutorial.md)** — start here. Solves a CVRP from scratch,
  step by step: defining the problem's resource, building the `Solver`, and
  running it.
- **[Accept Criteria](docs/accept-criteria.md)** — how candidate solutions
  are accepted or rejected during the search (`AcceptBest`, `Metropolis`,
  `MetropolisTimed`, `RandomWalk`).
- **[Stop Criteria](docs/stop-criteria.md)** — when the search stops
  (`ByIterMax`, `ByTime`, `ByTemperature`).
- **[Penalty Manager](docs/penalty-manager.md)** — how infeasible solutions
  are penalized rather than rejected (`StandardPenaltyManager`,
  `TargetRatePenaltyManager`).
- **[Algorithms](docs/algorithms.md)** — the metaheuristics available
  out of the box (`NILSAlgorithm`, `ILSAlgorithm`) and how to choose
  between them.

## Example applications

[`Applications/`](Applications/) contains problem-specific applications
built on top of the package. [`Applications/CVRP`](Applications/CVRP) is the
one used throughout the tutorial and is kept up to date with the package's
current API (`mainCVRP.jl` / `resourcesCVRP.jl`).
