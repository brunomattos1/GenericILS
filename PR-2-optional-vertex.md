# Optional vertices: prize-collecting and orienteering problems (TOP, PCVRP)

**Branch:** `feature/optional-vertex` → `main`

> **Depends on the visit-groups PR** (`feature/visit-groups`). This branch is built on top of it, so merge that one
> first; until then this diff also shows its commits. After the merge only the two commits of this PR remain
> (`feat: optinal vertices + neighborhoods + TOP`, `feat: PCVRP application`).

## Summary

A vertex can now be **optional**: the solution may visit it or leave it out. Together with a resource that rewards
the visit (a profit), this models prize-collecting problems, and the heuristic still **minimizes** the objective
(a profit is a negative cost). Two applications use it: the **Team Orienteering Problem (TOP)** and the
**Prize-Collecting VRP (PCVRP)**.

```julia
customers = vcat([Vertex(i, Group(i, 1, false)) for i in 1:m],     # mandatory
                 [optionalVertex(i) for i in m+1:n])               # visited only if they pay
neighborhoods = (TwoOptStar(), IntraShift(), InterShift{1}(), InterSwap{1,1}(), AddOptional(), RemoveOptional())
```

## Semantics

`Vertex(id, group; isOptional = true)`: an optional vertex **does not count for the `numVisits` of its group**, which
is about the regular vertices only, and may be visited or not. `optionalVertex(id)` is an optional customer alone in
its own group, which needs no visit (`numVisits = 0`). Groups and optional vertices compose: a group may have regular
and optional members, and a `sameRoute` group keeps all its visited members, optional or not, in one route.

A group that has only optional vertices must have `numVisits = 0`; otherwise `ProblemData` rejects it.

The existing two-argument `Vertex(id, group)` keeps working, so nothing changes for problems without optional
vertices.

## What changes

### Data model

`ProblemData` derives `isOptional`, `optionalIds`, `regularMembers`, `nOptional`, `locked` (per group, now aware of
the optional members) and the flags `hasOptional` and `hasExchangeableGroups`. A group has to stay in one route when
it is `sameRoute` and can have several visits (`numVisits + number of optional members > 1`).
`groupViolations` does not count optional vertices in the quota. `canLeaveRoute` and `safeCuts!` count the visits a
group actually has in the route for groups with optional members; groups without them keep the constant-time check.

### Construction

Seeds are drawn from the regular vertices only (a route without a regular seed starts empty), the mandatory insertion
phase ignores optional vertices, and a last phase inserts optional vertices one by one **only while the best insertion
lowers the objective**. For problems without optional vertices the result is unchanged.

`evalBestInsertion` no longer builds a one-element vector per evaluation (it reuses the solver's buffer). On a
150-customer CVRP the construction allocates 186 KiB instead of 43 MiB and runs in 43 ms instead of 52 ms, with the
same solution.

### New neighborhoods

- **`AddOptional`**: inserts an unvisited optional vertex in its best position of each route if that lowers the
  objective; a vertex of a group that stays in one route can only go to the route of its group. It does **not** use
  the `lastEval` cache: its candidates depend on which optional vertices are visited in the whole solution, so a
  cached pair of routes could hide a move created by a removal elsewhere.
- **`RemoveOptional`**: removes a visited optional vertex if that lowers the objective. It depends on one route only
  and uses the cache like the other neighborhoods.
- `IntraGroupExchange` now exchanges regular vertices only (optional ones are in nobody's quota).

Both new moves reuse the label primitives and the distance pruning, so with the profit as a discount in the cost
matrix they stay cheap (about 0.05 ms per pass on 150 customers with half of them optional).

### Perturbation

`randomToggleOptional!` flips a random optional vertex (removes it if visited, otherwise inserts it at a random
position, in the route of its group when the group must stay together). The NILS and ILS perturbations call it once
per perturbation, only when the problem has optional vertices; otherwise the random-number stream is untouched.

### Set Partitioning

The quota constraint of a group sums the regular visits; optional vertices only appear in the "at most once"
constraints; the empty solution is valid when nothing is required. A `sameRoute` group with optional vertices gets one
extra constraint ("at most one of the selected routes holds the group"), which the quota used to imply and no longer
does.

### Modelling the profit

Two equivalent ways, both documented in the tutorial and in `skills/`:

1. **In the resource**: `isCostResource() = true` and the arc entering a vertex returns `-profit` as its cost. It
   can depend on the state (arrival time, position, …). It turns off the pruning by distance, which is global.
2. **In the cost matrix**: `cost[i, j] - profit[j]`. It keeps the pruning and is what the applications below use.

### Applications

- **TOP** (new): Chao et al. instances (60 files) with the maximum path length as a standard resource. The objective
  is `-profit + w · distance` with a small `w` so that shorter paths count; start and end points that differ are
  handled with an asymmetric depot row/column. Includes a batch runner with a checker that recomputes the lengths from
  the coordinates.
- **PCVRP** (new): instances A and M (65 files). The prize is a discount on the arc entering the customer and the
  capacity is a standard resource. **The minimum demand `Q` that the instances state is not enforced**: it is a global
  constraint over all the routes, which the package does not support (see below). The application prints the demand
  satisfied next to `Q`, and its docstring says so.
- GVRP and F-CVRP now use the `MetropolisTimed` acceptance; two more instances were added (`G-n262-k25-C131-V12`,
  `P101_3_51_1`).

### Behavior change worth a look

`NILS.run!` now calls `printInfo` after every outer iteration, so every application that uses NILS prints one row of
the progress table per iteration (the function already existed and was never called).

## Testing

`Pkg.test()` passes: **75,277 tests**. New: `testOptionalVertices.jl` and `testOptionalResource.jl`, with
`optionalResourceScript.jl`, which runs in its own process because `isCostResource()` is global.

- unit tests of the data model, validation, `groupViolations`, `canLeaveRoute`, `safeCuts!`;
- directed cases on a line (where a profit pays, which route, which position, the group that must stay together);
- `apply!` of each move checked against the same operation on plain vectors, and the predicted objective against the
  applied one on feasible routes;
- RVND with every neighborhood keeps the solution valid and a fresh evaluation (cache cleared) finds nothing more;
- a brute-force oracle for Set Partitioning with optional vertices (every subset of a small pool, feasible and
  infeasible, with and without a route limit);
- the profit paid by the **resource** end to end: the final solution costs `distance − profit` and no single
  insertion or removal improves it, checked by brute force;
- mutation checks done by hand for the main guards (construction phases, group route of an optional vertex, route
  modification marks, perturbation, MIP constraints).

Smoke runs: TOP `p4.2.a` reaches 206 on three seeds; PCVRP `A-n32-k5_1` 673.66, `A-n60-k9_1` 1200.47 and
`M-n101-k10_1` 3088.95 (profit = prizes − distance), identical across seeds.

## Known limitations / follow-ups

- Global constraints (a minimum total demand over all routes, or covering customers with the visited facilities) are
  not supported; a hard invariant would work for a single quota but not for coverage.
- A pair "remove an optional vertex, add another" is not one move; with a tight capacity the search needs both steps
  to improve separately.
- `AddOptional` evaluates every route at every call (no cache), and a resource cost disables the pruning by distance.
- One flip per perturbation call; the intensity is not a `Diversification` parameter yet.

🤖 Generated with [Claude Code](https://claude.com/claude-code)
