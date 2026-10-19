# Is the problem a fit for GenericILS?

## The core idea (explain this to the user in one paragraph)

GenericILS evaluates local-search moves in O(1) by keeping, for every position of a route, a
**forward label** (summary of the prefix from the depot) and a **backward label** (summary of the
suffix up to the depot). A move (shift, swap, 2-opt*) is evaluated by *concatenating* a forward
and a backward label. So a problem fits when:

1. A solution is a **set of routes** that start/end at a depot, each customer on exactly one route.
2. Everything that makes a route feasible or costly can be computed **per route**.
3. That per-route logic can be written as a **resource extension**: a small state that is
   updated arc by arc (forward and backward) and can be **concatenated** (prefix + suffix → full
   route) in constant time.
4. The objective is the sum over routes of (arc costs from the distance matrix + an optional
   custom cost computed by the custom resource).

At the end a Set Partitioning MIP picks the best combination of routes found during the search,
which works for any problem where the route-set structure holds.

## Quick decision table

| Feature | Verdict | Notes |
|---|---|---|
| Capacity (single dim) | Direct | Standard resource or custom (CVRP) |
| Time windows (hard, with waiting) | Direct | Standard resource: `d` = travel+service, `lb`/`ub` = window (VRPTW) |
| Route duration / max length | Direct | Custom state (VRPTWDC) |
| Multiple capacity dims | Direct | Std1 + Std2 + custom |
| Release dates / ready times | Direct | Custom (VRPTWRD) |
| Simultaneous pickup & delivery | Direct | Custom, track picked-up and remaining delivery (VRPSPD) |
| Pickup→delivery precedence | Direct, bigger state | Track a visited-set in the state (`Vector{Bool}` or bitset, see VRPTWRDIC's `FastBitSet32`) |
| Incompatible customer pairs | Direct, bigger state | Bitsets of visited and of forbidden customers (VRPTWRDIC) |
| Max risk / accumulated exposure | Direct | Custom state, return `Inf` when exceeded (RiskVRP) |
| Load- or position-dependent cost | Direct | Cost resource (`isCostResource() = true`), e.g. cumulative cost (CCVRP) |
| Heterogeneous fleet, fixed + variable cost | Trick | Cost resource that accounts for the vehicle type of the route (HFVRP) |
| Multiple depots | Trick | Single dummy depot; real depot costs through a cost resource tracking first/last vertex (MDVRP) |
| Backhauls / ordering rules | Direct | State with a "phase" flag |
| Open routes | Likely direct | Zero the return-to-depot arc costs; verify with the checker |
| Fixed number of vehicles | Direct | `maxNbRoutes` in `ProblemData`; check behavior when it is tight |
| Visit one of several alternatives (generalized VRP) | Direct | `Group` with `numVisits = 1` over the alternatives; add `IntraGroupExchange()` so the search can change the chosen vertex (GVRP) |
| Visit k of the n customers of a family, in any routes (family capacitated VRP) | Direct | `Group(f, k, false)` and `IntraGroupExchange()`; each visited vertex carries the family demand (FCVRP) |
| A cluster served by a single vehicle | Direct | `Group(g, length(cluster), true)`; the moves never split it (no contiguity inside the route) |
| Optional customers / prize collecting, orienteering, team orienteering | Direct | `isOptional` vertices (`optionalVertex(i)`) with the profit in the resource as a negative cost for entering the vertex; a route length limit is an ordinary resource. Add `AddOptional()` and `RemoveOptional()` |
| Objective coupling routes (min-max, balance, makespan) | **No** (or reformulate) | Not decomposable per route |
| Shared resources across routes, synchronization, multi-trip with one vehicle | **No** | Routes are evaluated independently |
| Time-dependent travel times | **Unlikely** | Backward labels can't evaluate arrival-time-dependent travel without extra machinery; ask first |
| Location-routing, two-echelon, inventory routing | **No** | Extra decisions outside the route sequence |
| Stochastic/dynamic with re-optimization | **No** | Offline deterministic solver |

When a row says "Direct" or "Trick" but you have not seen a working app for it, say the
verdict is expected, not proven, and plan the prototype validation (SKILL.md, Step 5).

## Red flags to probe in the interview

- "…across all vehicles…" / "…the longest route…" / "…balanced…" → cross-route objective.
- "…the same driver/vehicle can do several routes…" → multi-trip.
- "…not all customers need to be served…" → optional vertices (or "visit k of these n": a group).
- "…customers of a cluster/family…" → visit groups; ask how many visits and whether they share a route.
- "…travel time depends on the time of day…" → time-dependent.
- "…decide where to open facilities…" → location decisions.
- "…resource consumed that is shared between routes…" → coupling.

## How to say it

State the verdict, the one-sentence reason, and the concrete consequence. Examples:

- "Fits directly: capacity + time windows are the VRPTW setup; only a reader and a checker are new."
- "Fits with a trick: heterogeneous fleet is handled like HFVRP — the custom resource carries the
  vehicle-type choice in its cost."
- "Fits directly: the optional customers are `isOptional` vertices and the profit is a negative cost in the
  resource; only the profit rule and a checker are new."
- "Doesn't fit: minimizing the longest route couples routes, and the labels evaluate one route at a
  time. Minimizing total distance would fit — is that acceptable as a first step?"
