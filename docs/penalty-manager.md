# Penalty Manager

`GenericILS` doesn't reject infeasible solutions during the search — it
**penalizes** them. A route that violates capacity, a time window, or your
`CustomResource`, keeps being explored by local search, just with an
artificially inflated cost. This is what lets the search "cross" infeasible
regions of the solution space to reach good feasible solutions on the other
side — a standard technique in VRP local search.

Who controls the intensity of that penalization is the `PenaltyManager`,
shared by any `Algorithm` (unlike `AcceptCriteria`/`StoppingCriteria`, it's
not duplicated per algorithm — it's used directly by the generic
`objectiveValue` calculation).

```julia
setPenaltyManager!(solver, StandardPenaltyManager(...))
```

## The three penalty "lanes"

Every `PenaltyManager` keeps three independent penalties, one per resource
on the `Solver`:

- `penaltyCustom` — penalizes infeasibility of your `CustomResource`
  (`sol.totalInfeas`).
- `penaltyStandard1` — penalizes violation (`warp`) of `StandardResource{1}`.
- `penaltyStandard2` — penalizes violation (`warp`) of `StandardResource{2}`.

If you don't use one of the `StandardResource` (left "loose", with no
effective constraint — see [tutorial](tutorial.md#step-2--building-the-resources)),
zero out its `penaltyStandardN` and its adjustment factors; otherwise that
penalty stays active without ever being violated, no practical effect, but
clutters `printInfo`.

## `StandardPenaltyManager`

Incremental adjustment, each outer-loop iteration: if the last solution was
feasible in that lane, the penalty *decreases* (less pressure); if it was
infeasible, it *increases*.

```julia
setPenaltyManager!(solver, StandardPenaltyManager(
    penaltyCustom = 1.0, penaltyCustomIncrease = 0.01, penaltyCustomDecrease = 0.01,
    penaltyStandard1 = 1.0, penaltyStandard1Increase = 0.01, penaltyStandard1Decrease = 0.01,
    penaltyStandard2 = 0.0, penaltyStandard2Increase = 0.0, penaltyStandard2Decrease = 0.0
))
```

For each lane `X ∈ {Custom, Standard1, Standard2}`:

- `penaltyX` — initial penalty value.
- `penaltyXIncrease` — fraction to increase by when the last solution was
  infeasible in that lane: `penaltyX *= (1 + penaltyXIncrease)`, up to a
  cap of `10000.0`.
- `penaltyXDecrease` — fraction to decrease by when it was feasible:
  `penaltyX *= (1 - penaltyXDecrease)`, down to a floor of `0.1`.

Larger `Increase`/`Decrease` values make the penalty react faster to the
recent feasibility rate, which can oscillate more; small values (like
`0.01` in the example) give a smoother response.

**Zeroing out a lane** (`penaltyStandard2 = 0.0, penaltyStandard2Increase =
0.0, penaltyStandard2Decrease = 0.0` in the example above) effectively
disables penalization for that resource — useful when you don't use
`StandardResource{2}`.

## `TargetRatePenaltyManager`

Instead of reacting to each individual iteration, accumulates a **window**
of `updatePeriod` iterations and adjusts the penalty based on the observed
feasibility *rate* within the window, compared to a target
(`targetFeasRate`).

```julia
setPenaltyManager!(solver, TargetRatePenaltyManager(
    penaltyCustom = 100.0, penaltyCustomIncrease = 0.4, penaltyCustomDecrease = 0.2,
    penaltyStandard1 = 100.0, penaltyStandard1Increase = 0.4, penaltyStandard1Decrease = 0.2,
    penaltyStandard2 = 100.0, penaltyStandard2Increase = 0.4, penaltyStandard2Decrease = 0.2,
    targetFeasRate = 0.7,       # target: 70% of solutions in the window should be feasible
    feasRateTolerance = 0.05,   # dead band around the target
    updatePeriod = 30           # window size, in iterations
))
```

Every `updatePeriod` iterations:
- if the observed feasibility rate `ρ` fell **below**
  `targetFeasRate - feasRateTolerance`, that lane's penalty **increases**
  (the search is generating too many infeasible solutions);
- if it rose **above** `targetFeasRate + feasRateTolerance`, the penalty
  **decreases** (the search is being too conservative, worth relaxing);
- inside the tolerance band, the penalty doesn't change.

This is the mechanism used by HGS (Hybrid Genetic Search) and tends to be
more stable than `StandardPenaltyManager`'s per-iteration adjustment, at the
cost of reacting with a delay of up to `updatePeriod` iterations.

## Which one to use

| If you want... | Use |
|---|---|
| a simple response that reacts every iteration | `StandardPenaltyManager` |
| to maintain a specific feasibility rate throughout the search (e.g. 70%) | `TargetRatePenaltyManager` |
