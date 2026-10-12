# API cheat-sheet for writing a resource

See also `docs/tutorial.md` (CVRP, step by step) and the other files in `docs/`. Everything is
exported by `using GenericILS`; to *add methods* you must `import GenericILS: extendAlongArc, ...`.

## Indexing — the main source of bugs

- Vertex ids: depot = 0, customers 1..n. Matrices/vectors passed to the resources have size
  (n+1) and are indexed by **id + 1** (depot = index 1).
- An arc is `a = (i, j)` in **matrix indices**. `label.last` and `Vertex(id)` use ids (`a[2]-1`).
- Forward extension along `(i, j)`: the prefix gets vertex `j` appended → use data of the head
  `a[2]`. Backward extension along `(i, j)`: the suffix gets vertex `i` prepended → use data of
  the tail `a[1]`. (CVRP: forward adds `d[1, a[2]]`, backward adds `d[1, a[1]]`.)
- `concatenationCost(res, v, fwd, bwd)` joins a forward label (prefix) with a backward label
  (suffix). What it returns must equal what extending the entire route forward would give.
  Start from an existing app and keep its convention.

## Functions to define for the custom resource

```julia
struct MyResource <: AbstractResource ... end           # data of your rules
GenericILS.isSymmetric() = false                        # global to the process
GenericILS.isCostResource() = false                     # true if it contributes route cost
struct ForwardState ... end ; struct BackwardState ... end   # immutable
initStateForward(res::MyResource)  = (ForwardState(...),  0.0)   # (state, cost)
initStateBackward(res::MyResource) = (BackwardState(...), 0.0)
extendAlongArc(res::MyResource, label::ForwardLabel,  a::Tuple{Int,Int}) -> (state, cost)
extendAlongArc(res::MyResource, label::BackwardLabel, a::Tuple{Int,Int}) -> (state, cost)
concatenationCost(res::MyResource, v::Int, fwd::ForwardLabel, bwd::BackwardLabel) -> (state, cost)
```

- `cost == Inf` means "violates"; for a non-cost resource otherwise return `0.0`.
- For a cost resource, `cost` is the **accumulated** custom cost of the partial route (MDVRP and
  CCVRP return the running total, not the increment), and `concatenationCost` returns the total
  for the full route.
- Labels: `ForwardLabel{S}(state, cost, std1State, std2State, last)`; read your data from
  `label.state`.

## Standard resources (two built in)

```julia
StandardResource{1}(d::Matrix, lb::Vector, ub::Vector)   # d: (n+1)x(n+1), lb/ub: length n+1
```

Forward: `q' = max(min(q + d[i,j], ub[j]), lb[j])`; the excess over `ub[j]` accumulates as *warp*
(violation, penalized by the penalty manager). Typical uses: time windows (`d` = travel+service,
`lb`/`ub` = window) and capacity (`d[i,j]` = demand of j, `lb = 0`, `ub = Q`). An unused standard
resource: `StandardResource{2}(zeros(n+1,n+1), zeros(n+1), fill(Inf, n+1))`.

## Building and running

```julia
res = Resources(customRes, stdRes1, stdRes2)
algorithm = NILSAlgorithm(res;
    acceptCriteria = NILS.AcceptBest(),            # or NILS.Metropolis(T0, α), NILS.MetropolisTimed(...)
    stopCriteria = NILS.ByIterMax(200),            # or NILS.ByTemperature(Tmin), NILS.ByTime(...)
    diversification = NILS.Diversification(outerShift = 1, outerSwap = 0, innerShift = 2, innerSwap = 0),
    innerIterMax = 5, timeLimitILS = 3600.0, timeLimitSP = 3600.0)
solver = Solver(res = res,
    data = ProblemData(customers::Vector{Vertex}, costMatrix, maxNbRoutes),
    neighborhoods = (TwoOptStar(), IntraShift(), InterShift{1}(), InterShift{2}(),
                     InterSwap{1,1}(), InterSwap{2,1}(), InterSwap{2,2}()),
    algorithm = algorithm)
setSeed!(solver, Random.MersenneTwister(seed))
setPenaltyManager!(solver, StandardPenaltyManager(penaltyCustom = 1.0, ...))   # docs/penalty-manager.md
setMIPSolver!(solver, HiGHS.Optimizer)                                         # default; CPLEX optional
solve!(solver)
sol = getBestSol(solver)          # prints routes; sol.cost; sol.routes[r].visits = [0, ..., 0]
```

`ILSAlgorithm` / `ILS.*` is the single-layer alternative (`docs/algorithms.md`). Criteria are per
algorithm and must be qualified with `NILS.` or `ILS.`. There are no restarts: for several seeds,
run the program several times. Check `docs/accept-criteria.md` and `docs/stop-criteria.md` for the
exact constructors before using one you haven't seen.

## Notes

- `ProblemData` costMatrix: distance cost between ids (index id+1). Zero matrix if all cost is in
  the resource.
- `maxNbRoutes`: upper bound on routes (e.g. `ceil(sum(demand)/Q) + 3`, or `n`).
- After `solve!`: `solver.algorithm.outerBestSol` holds the final solution too.
- Set Partitioning time limit: `timeLimitSP`; the cutoff hint is CPLEX-only and silently skipped
  with other solvers.
