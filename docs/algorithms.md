# Algorithms

Every `Solver` needs an `Algorithm` — the object that actually decides *how*
to search for a solution (every other piece — labels, resources, local
search — is reused by any algorithm). `GenericILS` ships two ready to use:
`NILSAlgorithm` and `ILSAlgorithm`. This page describes both, how to choose
between them, and how to implement a third one from scratch.

## `NILSAlgorithm` — Nested ILS

An ILS with **two layers**: an outer loop perturbs and accepts/rejects
solutions according to an `AcceptCriteria`/`StoppingCriteria`; for each
outer-loop candidate, an inner loop (`innerIterMax` iterations) refines it
via local search (`RVND!`) plus light perturbation, before handing the
result back to the outer loop. At the end of the outer loop, a **Set
Partitioning** step runs — exactly solving, via MIP, the best combination of
routes visited and stored in the algorithm's `route_storage`, which
typically improves the final solution beyond what local search alone
reaches.

```julia
algorithm = NILSAlgorithm(res;
    acceptCriteria = NILS.AcceptBest(),
    stopCriteria = NILS.ByIterMax(200),
    diversification = NILS.Diversification(outerShift = 1, outerSwap = 0, innerShift = 2, innerSwap = 0),
    innerIterMax = 5,
    timeLimitILS = 3600.0,
    timeLimitSP = 3600.0,
    aggressivePool = false
)
```

Parameters:

- `acceptCriteria` — see [Accept Criteria](accept-criteria.md).
- `stopCriteria` — see [Stop Criteria](stop-criteria.md).
- `diversification` — controls the intensity of perturbation applied at
  each layer: `outerShift`/`outerSwap` (number of random
  `InterShift`/`InterSwap` moves applied per **outer**-loop iteration) and
  `innerShift`/`innerSwap` (same, for the **inner** loop).
- `innerIterMax` — number of inner-loop iterations without improvement
  before handing the result back to the outer loop.
- `timeLimitILS` — time ceiling (seconds) for the search phase (outer +
  inner), independent of `stopCriteria`. Acts as a safeguard to guarantee
  Set Partitioning always runs, even if the chosen `stopCriteria` never
  fires.
- `timeLimitSP` — time ceiling (seconds) for the MIP solver in the Set
  Partitioning step.
- `aggressivePool` — if `true`, registers routes into the Set Partitioning
  pool on every inner-loop iteration too (not just at the end of each
  outer-loop iteration). Increases the chance of Set Partitioning finding a
  good combination, at the cost of more memory/time in that step.

Use `NILSAlgorithm` when you want the classic two-level structure (as
described in the nested-ILS literature) and the final Set Partitioning
step.

## `ILSAlgorithm` — single-layer ILS

Simpler structure: a single loop, with no separate inner loop — each
iteration perturbs, runs `RVND!` once, and accepts/rejects via
`AcceptCriteria`. Also ends with Set Partitioning.

```julia
algorithm = ILSAlgorithm(res;
    acceptCriteria = ILS.AcceptBest(),
    stopCriteria = ILS.ByIterMax(200),
    diversification = ILS.Diversification(shift = 1, swap = 0),
    timeLimitILS = 3600.0,
    timeLimitSP = 3600.0,
    aggressivePool = false
)
```

Same parameters as `NILSAlgorithm`, except `diversification` only has
`shift`/`swap` (there's no inner loop to distinguish `outer`/`inner`), and
there's no `innerIterMax`.

Use `ILSAlgorithm` when you want something simpler to reason about, or when
the NILS's extra layer doesn't bring a measurable gain for your problem.

## Running an algorithm

Regardless of which algorithm you chose, the call is always:

```julia
solve!(solver)
```

`solve!` dispatches to `run!(solver.algorithm, solver)`. This is the only
coupling point between the `Solver` (generic) and the chosen algorithm —
`Solver` never knows which algorithm is running; it only calls `run!`.

## Why are `NILS.Metropolis` and `ILS.Metropolis` different types?

`NILSAlgorithm` and `ILSAlgorithm` live in **isolated submodules**
(`GenericILS.NILS` and `GenericILS.ILS`), each with its own complete copy of
`AcceptCriteria`, `StoppingCriteria`, `Diversification`, local search
(`RVND!`), and Set Partitioning logic. This is deliberate: each algorithm is
free to evolve its own criteria independently, with no risk of a change in
one affecting the other — in fact, if you deleted the entire
`Algorithms/NILS/` folder, `ILSAlgorithm` would keep working normally (and
vice versa).

The practical consequence is that `Metropolis`, `AcceptBest`, `ByIterMax`,
`Diversification`, etc. exist **twice** — once inside `NILS`, once inside
`ILS` — as genuinely distinct Julia types, even though they share the same
name and identical behavior. That's why you always qualify with the prefix
of the algorithm you're using:

```julia
NILSAlgorithm(res; acceptCriteria = NILS.Metropolis(100.0, 0.995), ...)
ILSAlgorithm(res;  acceptCriteria = ILS.Metropolis(100.0, 0.995), ...)
```

The only code genuinely shared across all algorithms — independent of that
isolation — is what doesn't depend on "how" the search is orchestrated:
`constructSol!` (greedy initial construction), `search!`/the neighborhoods
(`TwoOptStar`, `IntraShift`, `InterShift`, `InterSwap`), `computeLabels`, and
the `PenaltyManager`. That lives in the root module (`GenericILS`), not
inside `NILS`/`ILS`.
