---
name: genericils-app
description: Guides a user through modeling a new routing problem with the GenericILS Julia package, from "is my problem even a fit?" to a running application. Interviews the user about the problem (routes, vehicles, constraints, objective, data), decides whether GenericILS can model it (resource-extension style route constraints/costs) or whether it needs workarounds or package changes, then scaffolds Applications/<Problem>/ (resources file, main file, Project.toml, independent checker) and validates it. Use this skill whenever the user wants to create, add, port or prototype a VRP variant or any route-based problem with GenericILS, asks "can GenericILS solve X?", asks how to write a CustomResource / extendAlongArc / concatenationCost, wants to use Pkg.add GenericILS for their own problem, or mentions things like CVRP, VRPTW, pickup and delivery, heterogeneous fleet, multi-depot, duration limits, incompatibilities or any new VRP constraint, even if they never say "GenericILS application" explicitly. Reply in the user's language (Portuguese or English).
---

# Creating a GenericILS application

_These instructions are plain Markdown and work with any AI coding assistant. Paths below are relative to this skill folder; use whatever file, shell and Julia tools you have._

GenericILS searches over sets of routes with local search + Set Partitioning. The user supplies
only the problem-specific part: a small **resource** that says, for a partial route, what to
remember and when it becomes infeasible or costly. Your job is to (1) understand the problem,
(2) say honestly whether it fits, (3) translate it into that resource, (4) scaffold and
validate the application.

Talk to the user in the language they write in. Many users know routing well but not Julia
internals, so explain package terms (label, resource, penalty) briefly when you first use them.

## Step 0 — Orient yourself (silently, ~1 minute)

- Are we inside the GenericILS repository (has `src/GenericILS.jl` and `Applications/`)? Then the
  new app goes in `Applications/<Problem>/`. Otherwise the user installed the package with
  `Pkg.add(url="https://github.com/brunomattos1/GenericILS")`; create a standalone folder.
- List `Applications/` (when present) and skim the closest existing app. They are the best
  documentation of conventions; `references/patterns.md` maps constraints to them.
- Check `julia` is available (`julia --version`); you will run the result.

## Step 1 — Interview

Ask in small batches (2–4 questions), not one giant form. Skip what the user already told you.
If the user has a paper, instance files or code, read those first and only ask about gaps.

1. **The story.** In their own words: what is being routed, who/what is visited, what is one
   "route", what must be decided, what is the objective?
2. **Structure.** One depot or several? Homogeneous fleet or vehicle types? Fleet size fixed or free?
   Must every customer be visited exactly once, or are customers grouped (visit k of a group, a
   whole group by the same vehicle)? Any optional customers or profits?
3. **Per-route constraints.** Capacity (one or several dimensions, pickups and deliveries?),
   time windows, max duration/distance, precedence (pickup before delivery), incompatible
   customers, release dates, risk/exposure limits, ordering rules (backhauls last), anything else.
4. **Objective.** Pure distance? Fixed vehicle costs? Costs that depend on load, arrival time or
   position in the route (e.g. cumulative/latency)? Is anything coupled *across* routes
   (balance, makespan, shared drivers/resources, synchronization)?
5. **Data and goal.** Instance format/benchmarks, size (customers), time budget, known best
   solutions to compare against, exactness vs good heuristic solutions.

For choices with a few clear options, use your interactive question tool if you have one; otherwise ask in plain text.

## Step 2 — Feasibility verdict

Read `references/feasibility.md` and classify the problem. Give the user the verdict **and the
reason**, in a few lines, before writing any code:

- **Direct fit** — every constraint/cost is decomposable per route and expressible as resource
  extension along arcs. Go on.
- **Fit with a modeling trick** — e.g. heterogeneous fleet, multi-depot, load-dependent cost,
  incompatibilities. Name the trick and the existing app that uses it.
- **Partial fit** — core works but something needs changes inside the package (cross-route
  objectives, new neighborhoods). Say exactly what, and offer a
  reduced version (relax/fix that piece) that does fit.
- **Does not fit** — say so plainly and suggest what would fit better. A clear "no" saves the
  user days; do not force it.

If unsure, say what you are unsure about and propose a 30-minute prototype to find out.

## Step 3 — Modeling plan (agree before coding)

Write a short plan and confirm it with the user:

| Piece | Decision |
|---|---|
| Vertices & indices | customers 1..n, depot 0; matrices are (n+1)x(n+1), index = id+1 |
| `ProblemData` | vertices with their visit `Group`s (one group per customer for a classic VRP), cost matrix (distance cost), max number of routes |
| `StandardResource{1}` / `{2}` | which constraints use the two built-in bounded accumulators (e.g. time windows, capacity) — see `references/api.md` |
| `CustomResource` | the remaining rules; state fields for forward and backward; `isCostResource`, `isSymmetric` |
| Algorithm | `NILSAlgorithm` (default) or `ILSAlgorithm`; accept/stop criteria |
| Penalty manager | which lanes are active (custom, std1, std2) |
| Data reader | instance format → arrays |
| Checker | the independent feasibility/cost check you will write |

Prefer the built-in standard resources when a constraint fits them (they are already correct
in both directions); put only the rest into the custom resource.

## Step 4 — Scaffold

Copy and adapt the templates in `assets/` (they run out of the box on a random CVRP, so you
always start from something working):

- `assets/resourcesTemplate.jl` → `resources<Problem>.jl`
- `assets/mainTemplate.jl` → `main<Problem>.jl` (repo naming: `main<Problem>.jl`, `resources<Problem>.jl`)
- `assets/ProjectTemplate.toml` → `Project.toml`
- put instance files in a `data/` folder next to them; add a reader in `main<Problem>.jl`

Implement the resource following `references/api.md` (indexing and the forward/backward
conventions are where bugs hide). Copy the structure of the closest existing app rather than
inventing new conventions.

## Step 5 — Validate (do not skip)

A resource that is wrong still *runs* — it just returns bad or infeasible routes. So:

1. **Independent checker.** Write `checkSolution` from the problem definition, without labels
   (walk each route, recompute load/time/etc.). The template shows the shape. Run it on the
   solver's best solution.
2. **Tiny instance first** (5–15 customers), then a real one. Compare cost with a known value or
   a brute-force/hand solution when possible.
3. **Forward/backward consistency.** If results look suspiciously infeasible or worse than
   expected, the usual cause is a forward/backward/concatenation mismatch. Test that
   `concatenationCost` on a split route agrees with extending the full route forward.
4. Run for real with the intended algorithm settings; report cost, time, and checker result.

Run with `julia --project=. main<Problem>.jl` from the app folder. HiGHS is the default MIP
solver (works everywhere); CPLEX is faster for Set Partitioning if the user has a license.

## Step 6 — Hand-off

Summarize: files created, how to run, what is modeled exactly, what was relaxed, validation
results, and suggested next steps (tuning criteria, bigger instances, benchmark comparison).
Do not commit or push unless asked.

## Gotchas worth remembering

- `isSymmetric()`/`isCostResource()` are global per Julia process (one problem type per process); define
  `isSymmetric(::MyResource)`/`isCostResource(::MyResource)` instead to mix problem types in one process.
- Two built-in standard resources + one custom resource is the whole budget; plan for it.
- `Inf` returned by `extendAlongArc` means "violates"; the penalty manager prices violations, so
  the search crosses infeasible space on purpose. Don't return a big finite number instead.
- Unused standard resource: leave it loose (zeros / `Inf` bounds) and zero its penalty.
- Criteria live per algorithm: `NILS.AcceptBest()`, `ILS.Metropolis(...)` — always qualified.
