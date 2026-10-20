# Modeling patterns — which existing app to copy

Each app lives in `Applications/<Name>/` (files `main<Name>.jl`, `resources<Name>.jl`; some under `src/`).
Open the closest one *before* writing the resource. If the skill is used outside the repository,
browse them at https://github.com/brunomattos1/GenericILS/tree/main/Applications.

| Constraint / feature | App | What to look at |
|---|---|---|
| Capacity only; the tutorial example | `CVRP` | Simplest resource; `docs/tutorial.md` walks through it |
| Time windows + capacity, release dates | `VRPTW`, `VRPTWRD` | Time windows in `StandardResource{1}`, capacity in `{2}` or custom |
| Time windows + route duration limit | `VRPTWDC` | Custom state `q, ET, RD, TB` |
| Incompatible customers | `VRPTWRDIC` | Bitset states (`FastBitSet32`): `inc` (forbidden) and `inVeh` (visited) |
| Simultaneous pickup and delivery | `VRPSPD` | State: accumulated pickup `P`, remaining delivery `D`; `TargetRatePenaltyManager` |
| Accumulated risk limit | `RiskVRP` | State: risk `R`, demand `D`, cost `C`; `Inf` when `R > Rmax` |
| Cumulative / load-dependent cost | `CCVRP` | `isCostResource() = true`; returned cost is the accumulated `S` |
| Heterogeneous fleet | `HFVRP` | Cost resource; `ProblemData` cost matrix is zero, cost lives in the custom resource |
| Multi-depot | `MDVRP` | Dummy single depot; first/last vertex in the state, depot arc costs in a cost resource |
| Profit collection with a length limit (team orienteering) | `TOP` | Every customer `optionalVertex`; profit is a discount on the arc entering the customer in the cost matrix, plus a small distance term (`-profit + w * distance`) so that shorter paths count; the length limit is `StandardResource{1}`; start and end points that differ use an asymmetric depot row/column; `AddOptional()`, `RemoveOptional()` |
| Prize collection with capacity (prize-collecting VRP) | `PCVRP` | Every customer `optionalVertex`; the prize is a discount on the arc entering the customer in the cost matrix (`distance - prize`), capacity in `StandardResource{2}`; the minimum demand `Q` of the benchmark instances is not enforced; `AddOptional()`, `RemoveOptional()`, `MetropolisTimed` acceptance |
| One vertex per cluster (generalized VRP) | `GVRP` | `Group(g, 1, false)` per cluster, capacity resource as in `CVRP`, `IntraGroupExchange()`; reader for the `.vrp` GVRP format |
| Several visits per family (family capacitated VRP) | `FCVRP` | `Group(f, visits, false)` per family, each visited vertex carries the family demand, `IntraGroupExchange()`; batch runner with independent checker |

## Recurring ideas

- **Cost resource.** If the cost isn't a sum of arc costs, set `isCostResource() = true` and return
  the *accumulated* custom cost from `extendAlongArc` / `concatenationCost`. When the whole cost is in
  the resource (HFVRP, MDVRP), pass a zero matrix as `costMatrix` in `ProblemData`.
- **Hard constraints → `Inf`.** Return `(newState, Inf)` once a rule is violated, still returning a
  well-defined state so later extensions stay consistent.
- **Set-valued state.** For precedence/incompatibility, carry a bitset or `Vector{Bool}` of visited
  vertices; copy before modifying, because states are shared between labels.
- **Pushing a rule into a standard resource.** A rule "an accumulated quantity must stay within
  [lb_j, ub_j] at vertex j, where arc (i,j) adds d[i,j]" is exactly a standard resource. Time
  windows with waiting are the typical case.
- **Penalty manager.** `StandardPenaltyManager` (smooth) is the default; `TargetRatePenaltyManager`
  aims at a target share of feasible solutions (VRPSPD uses it). Unused lanes: penalty 0.
- **Algorithm choice.** NILS with `AcceptBest` + `ByIterMax` is the simple baseline;
  `Metropolis`/`MetropolisTimed` + `ByTemperature` is what tuned benchmark apps (e.g. VRPSPD) use;
  `ILSAlgorithm` if the nested loop doesn't help.
