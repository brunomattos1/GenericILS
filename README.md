# GenericILS

[![CI](https://github.com/brunomattos1/GenericILS/actions/workflows/CI.yml/badge.svg)](https://github.com/brunomattos1/GenericILS/actions/workflows/CI.yml)
[![codecov](https://codecov.io/gh/brunomattos1/GenericILS/graph/badge.svg)](https://codecov.io/gh/brunomattos1/GenericILS)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

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
(`TwoOptStar`, `IntraShift`, `InterShift`, `InterSwap`, `IntraGroupExchange`,
`AddOptional`, `RemoveOptional`), greedy construction, and an exact Set Partitioning step (via JuMP/MIP) — is
provided by the package. You only implement the parts specific to your problem.

Customers belong to **visit groups** (`Group(groupId, numVisits, sameRoute)`):
a group asks for `numVisits` of its vertices to be visited, all in the same
route when `sameRoute` is set. A classic VRP is one group per customer with one
visit; one visit per cluster is the generalized VRP (GVRP); several visits per
family is the family capacitated VRP (F-CVRP); a whole cluster served by one
vehicle is `numVisits = length(cluster)` with `sameRoute = true`. Vertices can also be **optional**
(`isOptional`): the solution visits them only if the resource rewards it, which models prize-collecting and
orienteering problems.

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
#    `customers` are `Vertex(id, Group(groupId, numVisits, sameRoute))`; a plain customer is
#    `Vertex(i, Group(i, 1, false))`
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

## Running the tests

```julia
using Pkg
Pkg.test("GenericILS")                  # or Pkg.test(coverage = true)
```

The suite lives in [`test/`](test/) and is split by area (penalty managers, accept/stop criteria,
solver API, algorithms, cost resources, block moves, visit groups and their moves, optional vertices, Set Partitioning). Line coverage of `src/` is above 90%. To
measure it, run the tests with `coverage = true` and summarize the generated `.cov` files with
[Coverage.jl](https://github.com/JuliaCI/Coverage.jl) (`using Coverage; process_folder("src")`).

## Repository layout and conventions

```
src/                    the package
  GenericILS.jl           module, exports
  Solver.jl, Solution.jl  solver state, routes and solutions
  LabelsFunctions.jl      labels and the built-in StandardResource
  Labels.jl, Infeasibility.jl   label computation and move evaluation
  Groups.jl               visit groups: Group, group-aware move checks, groupViolations
  Construct.jl, SetPartitioning.jl, PenaltyManager.jl, utils.jl
  neighborhoods/          TwoOptStar, IntraShift, InterShift, InterSwap, IntraGroupExchange, AddOptional,
                          RemoveOptional
  Algorithms/NILS, Algorithms/ILS   the two algorithms (each is a self-contained submodule)
test/                   test suite (see "Running the tests")
docs/                   tutorial and reference pages
Applications/<Problem>/ example applications (CVRP, VRPTW and variants, VRPSPD, CCVRP, HFVRP, MDVRP,
                        RiskVRP, GVRP, FCVRP, TOP): main<Problem>.jl, resources<Problem>.jl, data/
skills/                 guide for AI assistants (see below)
```

- **Naming:** `camelCase` for functions, variables and file names of scripts, `PascalCase` for types and
  modules, a trailing `!` for functions that mutate their first arguments.
- **Documentation:** every type and function of `src/` has a short docstring (`?name` in the REPL).
- **Generated files** (logs, result tables, spreadsheets, `Manifest.toml`, editor settings) are not
  versioned; see `.gitignore`.

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

## Creating a new application with an AI assistant

[`skills/genericils-app/`](skills/genericils-app/) is a guide for AI coding assistants (Claude,
Codex, Gemini, Cursor, Copilot, ...) that helps you model a new routing problem with GenericILS.
It is plain Markdown ([`SKILL.md`](skills/genericils-app/SKILL.md) plus reference files and
templates), so it is not tied to any one tool. It:

1. **Interviews you** about the problem: what a route is, depots and fleet, constraints
   (capacity, time windows, precedence, ...), objective and data.
2. **Tells you whether the problem fits** the package (constraints and costs that can be computed
   per route), needs a modeling trick, needs changes to the package, or doesn't fit at all.
3. **Scaffolds the application** (`main<Problem>.jl`, `resources<Problem>.jl`, `Project.toml`)
   from templates that run out of the box.
4. **Validates it** with an independent feasibility checker, on a small instance first.

The assistant needs to be able to read files, write files and run Julia in your project folder
(any agentic coding tool does).

### Using it with any assistant

Open your assistant in the folder where the application should live (for example a clone of this
repository, so it can use `Applications/` as reference) and say:

```
Read skills/genericils-app/SKILL.md and follow it to help me create a GenericILS application.
My problem is: <describe it>
```

If you installed the package with `Pkg.add`, the folder is in the package directory
(`using GenericILS; pkgdir(GenericILS)`), or you can download it from the repository.

### Making the assistant load it automatically

Each tool has its own place for persistent instructions. Pick yours:

| Tool | How |
|---|---|
| Claude Code | Copy the folder: `mkdir -p .claude/skills && cp -r skills/genericils-app .claude/skills/` (or `~/.claude/skills/` for all projects). Then it triggers on its own, or run `/genericils-app`. |
| Codex CLI | Copy the folder into `.agents/skills/` (or your Codex skills directory), or add a line to `AGENTS.md`: *"For new GenericILS applications, follow skills/genericils-app/SKILL.md"*. |
| Gemini CLI | Add the same line to `GEMINI.md` (or use `@skills/genericils-app/SKILL.md` to include the file). |
| Cursor, Copilot, others | Add the same line to the tool's rules/instructions file (`.cursor/rules`, `.github/copilot-instructions.md`, ...). |

Skill-loading locations change between tool versions, so check your tool's documentation if the
path above doesn't work; the "Read ... and follow it" prompt always does.

### Example prompts

- *"I need to route technicians with time windows and a maximum of 8 hours per route. Can GenericILS
  solve it?"*
- *"Create a GenericILS application for a VRP with pickup and delivery and incompatible items."*
- *"How do I write `extendAlongArc` and `concatenationCost` for my own constraint?"*

Answer its questions, confirm the modeling plan it proposes, and it will create and run the
application. It should not commit anything unless you ask.

## Contributing

`main` is protected: all changes go through a branch and a pull request, and the CI checks must pass.

```bash
git config core.hooksPath .githooks     # once per clone: blocks commits, merges and pushes on main
git switch -c my-change
# ...work, commit...
git push -u origin my-change            # then open a pull request
```

The hooks in [`.githooks/`](.githooks/) only protect your local clone (`git commit --no-verify` skips them).
The real enforcement is the GitHub ruleset described in [`.github/rulesets/protect-main.json`](.github/rulesets/protect-main.json):
in the repository go to *Settings → Rules → Rulesets → New ruleset → Import a ruleset* and upload that file. It
forbids direct pushes, force pushes and deletion of `main`, and requires a pull request with the CI checks passing
and the approval of the code owner ([`.github/CODEOWNERS`](.github/CODEOWNERS)); new pushes dismiss earlier approvals.
Repository admins may merge their own pull requests (GitHub does not let anyone approve their own PR), but still only
through a pull request, never by pushing to `main`.

## License

GenericILS is released under the [MIT License](LICENSE).
