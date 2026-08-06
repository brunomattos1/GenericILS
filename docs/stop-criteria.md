# Stop Criteria

A `StoppingCriteria` decides when the algorithm's outer loop should stop
iterating and move on to the final step (Set Partitioning). It's checked
every iteration via `stop(criteria, solver)`.

Like `AcceptCriteria`, `StoppingCriteria` is defined independently inside
each algorithm (`NILS.StoppingCriteria`, `ILS.StoppingCriteria`) — swap
`NILS.` for `ILS.` depending on the algorithm in use. See why in
[Algorithms](algorithms.md#why-are-nilsmetropolis-and-ilsmetropolis-different-types).

Pass the chosen criterion to the algorithm's constructor:

```julia
algorithm = NILSAlgorithm(res; stopCriteria = NILS.ByIterMax(200), ...)
```

## `ByIterMax(maxIter)`

Stops after `maxIter` outer-loop iterations without enough improvement to
reset the counter (the iteration counter is reset every time a new best
feasible solution is found — see [Penalty Manager](penalty-manager.md) and
`updateBestFeasible!`).

```julia
stopCriteria = NILS.ByIterMax(200)
```

The most predictable criterion in terms of run time (fixed iteration
count), though actual wall-clock time still varies with instance size.

## `ByTime(maxTime)`

Stops once `maxTime` seconds have passed since the algorithm's `run!`
started.

```julia
stopCriteria = NILS.ByTime(600.0)   # 10 minutes
```

Use it when you need a predictable time budget, independent of instance
size — for example, to compare instances of different sizes under the same
time ceiling.

## `ByTemperature(minTemp)`

Stops once the current `AcceptCriteria`'s temperature drops below
`minTemp`. **Only makes sense paired with an `AcceptCriteria` that has a
`temperature` field** (`Metropolis`, `MetropolisTimed`) — calling `stop`
with `ByTemperature` and, say, `AcceptBest()` (which has no temperature) is
a configuration error.

```julia
acceptCriteria = NILS.Metropolis(100.0, 0.995)
stopCriteria   = NILS.ByTemperature(1.0)
```

The search stops once `acceptCriteria.temperature <= 1.0`. Combined with a
geometric cooling `alpha`, this gives a deterministic iteration count:
`log(minTemp / temperature) / log(alpha)`.

## Choosing the right criterion

| If you want... | Use |
|---|---|
| a fixed time ceiling, independent of the accept criterion's cooling | `ByTime` |
| to stop once `Metropolis`/`MetropolisTimed` "cools down" | `ByTemperature` |
| a deterministic iteration count, independent of wall-clock time | `ByIterMax` |
