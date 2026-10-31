# Visit groups: GVRP, F-CVRP and clusters in one route as special cases of one model

**Branch:** `feature/visit-groups` → `main`

## Summary

Until now the package assumed that every customer is visited exactly once. This PR replaces that assumption with
**visit groups**: every vertex belongs to a `Group(groupId, numVisits, sameRoute)`, and a solution is valid when
each group has exactly `numVisits` of its vertices visited, all in the same route when `sameRoute` is set.

A classic VRP is the special case of one group per customer with one visit, so existing problems keep working, and
three new problem families fit the same model without touching the labels or the resource machinery:

| Problem | Model |
|---|---|
| Classic VRP / CVRP / VRPTW … | one group per customer, `numVisits = 1` |
| Generalized VRP (GVRP) | one group per cluster, `numVisits = 1` |
| Family capacitated VRP (F-CVRP) | one group per family, `numVisits = k`, `sameRoute = false` |
| Whole cluster served by one vehicle | `numVisits = length(cluster)`, `sameRoute = true` |

The group constraints are **hard invariants**: the construction establishes them and every move preserves them,
exactly as the "each customer once" constraint was handled before. Nothing is penalized, so the penalty lanes, the
labels and the `lastEval` cache keep their meaning.

## Breaking change

`Vertex(id)` no longer exists. A vertex now carries its group:

```julia
Vertex(i, Group(i, 1, false))     # a plain customer: alone in its group, which needs one visit
```

`ProblemData(vertices, costMatrix, maxNbRoutes)` keeps its signature. All the applications under `Applications/`
were migrated (one line each), and the README, the tutorial and the `skills/` guides were updated.

## What changes

### Data model (`src/Groups.jl`, `src/Solver.jl`)

- `Group`, `Vertex(id, group)` and `ProblemData`, which validates the instance (ids unique and positive, group ids
  `1:G`, every vertex of a group carrying the same `Group`, `1 ≤ numVisits ≤ length(members)`) and derives
  `groups`, `members`, `groupOf` and `hasLockedGroups` (whether some group has to stay in one route, which
  `mustStayInRoute(group)` decides: `sameRoute` with `numVisits > 1`).
- `groupViolations(data, solution_or_routes)` lists the group constraints a solution breaks. It is independent of the
  search, so it serves as the oracle in the tests and as a validator for users.

### Construction (`src/Construct.jl`)

The parallel greedy insertion only inserts vertices of groups that still need visits, and binds a `sameRoute` group to
the route of its first visit. The number of seed routes is limited by what the groups can seed (a `sameRoute` group
seeds one route; any other group seeds up to `numVisits`). For a plain VRP the initial solution is **bit-identical**
to the one on `main` (same seed, 4 instances compared).

### Neighborhoods and perturbation

The moves that only permute visited vertices (`TwoOptStar`, `IntraShift`, `InterShift`, `InterSwap`) cannot break the
quota, but they could split a `sameRoute` group. They now never do:

- `InterShift`/`InterSwap` skip a block that would leave part of a group behind (`canLeaveRoute`). A block that holds
  **all** the visits of a group may move: the group travels together.
- `TwoOptStar` only cuts where no group is split between head and tail (`safeCuts!`); a tail that carries a whole
  group is allowed.
- The random shift/swap perturbations of NILS and ILS draw only vertices that can leave their route (`randomMovable`).

The checks cost nothing when the problem has no locked group (`hasLockedGroups` is a precomputed flag).

### New neighborhood: `IntraGroupExchange`

Replaces a visited member of a group by an unvisited member of the same group. The new member can go to any position
of any route unless the group has to stay in one route, in which case it takes the place of the member it replaces.
This is the move that lets the search choose *which* vertex of a cluster or family to visit (GVRP, F-CVRP).

- Reuses the existing label primitives (`infeasArcsRemovalK`, `infeasArcsInsertionK`, `infeasArcsReplaceBlockK`).
- The effect of removing a member is computed once per position and shared by all the candidates and routes.
- An exchange changes which members are unvisited, which the cached evaluations of every route with a member of the
  group depend on, so `apply!` marks those routes as modified (covered by a test that fails without it).
- It returns immediately when no group has anything to choose from.

### Set Partitioning (`src/SetPartitioning.jl`)

The NILS and ILS copies of the model shared the same constraints; they now call one function,
`addPartitioningConstraints!`:

- per group: `Σ visits == numVisits`;
- per vertex: `Σ ≤ 1` (stated only when needed);
- at most `maxNbRoutes` routes; routes that hold part of a `sameRoute` group are fixed to 0 (they cannot appear in
  pools built by the algorithms, so this only guards hand-made pools).

For a plain VRP the model is exactly the previous one (one equality per customer). It is also built from sparse
incidence lists instead of testing `i in route` for every customer and route: 5000 pooled routes build in 7.8 ms
instead of 12.8 ms.

Small fix included: after the MIP improves the best solution, `bestFeasSol.dist` was left at `0`, so applications
printed `Cost: 0.0`. It is now recomputed from the routes.

### Applications

- **GVRP** (new): reader for the Bektaş et al. instances, application and batch runner with an independent checker.
- **F-CVRP** (new): reader for the `.fcvrp` family instances, application and batch runner with an independent checker.
- Every other application builds its customers with `Group` and was run to completion after the migration.

## Behavior and performance

Measured against `main` on a 150-customer CVRP (M-n151-k12), same seed:

- Initial solution and local-search results are identical; one full RVND takes 2.7 ms against 2.5 ms (within the noise
  of the measurement) and the allocations per search are unchanged.
- The group checks of the moves run after the distance pruning and are skipped when no group is locked, so problems
  that do not use `sameRoute` pay nothing for them.

GVRP, Bektaş et al. instances (6 instances × 5 seeds, 1000 outer iterations, with the MIP): 24 of 30 runs reach the
known optimum, and the average gap is 0.05 %.

## Testing

`Pkg.test()` passes: **59,920 tests**, in new files `testGroups.jl`, `testGroupMoves.jl`,
`testIntraGroupExchange.jl` and `testSetPartitioning.jl`. Besides unit tests for each helper they include:

- property tests: every neighborhood, RVND and the perturbations keep `groupViolations(...) == []`, never worsen the
  solution, and leave the bookkeeping identical to a from-scratch recomputation (`checkConsistency`);
- a cache test: after other exchanges, a search that trusts `lastEval` finds the same move as a fresh one;
- a brute-force oracle for the Set Partitioning model (every subset of a small pool is enumerated and the optimum is
  compared; feasible and infeasible pools, with and without a route limit);
- mutation checks done by hand: removing the main guards (the block check, the safe cuts, the perturbation filter,
  the cache invalidation, the MIP constraints and route filter) makes at least one test fail.

## Known limitations / follow-ups

- `sameRoute` means "same route", not "consecutive": the vertices of a cluster may be interleaved with others.
- `IntraGroupExchange` keeps a locked group in its route by replacing in place; reordering is left to `IntraShift`.
- Constraints that involve several groups at once (for example "cover every customer with some visited facility")
  are not expressible with groups and are out of scope.

🤖 Generated with [Claude Code](https://claude.com/claude-code)
