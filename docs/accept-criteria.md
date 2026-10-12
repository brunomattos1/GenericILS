# Accept Criteria

An `AcceptCriteria` decides, at every iteration of the search, whether the
newly generated candidate solution replaces the current solution
(`currSol`) and/or the loop's best solution (`bestSol`). It's the mechanism
that gives an ILS/NILS its ability to escape local optima — occasionally
accepting worse solutions.

`AcceptCriteria` is defined **independently** inside each algorithm
(`NILS.AcceptCriteria`, `ILS.AcceptCriteria`) — see
[Algorithms](algorithms.md#why-are-nilsmetropolis-and-ilsmetropolis-different-types)
for why. The types below exist, with the same behavior, in both modules:
swap `NILS.` for `ILS.` depending on the algorithm you're using.

Pass the chosen criterion to the algorithm's constructor:

```julia
algorithm = NILSAlgorithm(res; acceptCriteria = NILS.Metropolis(100.0, 0.995), ...)
```

## `AcceptBest()`

Only accepts the candidate if it's strictly better than the best solution
seen so far (`bestSol`). No randomness — pure hill-climbing within the
outer loop.

```julia
acceptCriteria = NILS.AcceptBest()
```

Use it when you want deterministic behavior and you already trust
diversification to come from elsewhere (perturbation, multiple restarts).

## `Metropolis(temperature, alpha)`

Classic simulated annealing. Accepts the candidate as the new current
solution whenever it improves; otherwise, accepts it with probability
`exp(-Δ / temperature)`, where `Δ` is the cost worsening. Temperature decays
geometrically each iteration: `temperature *= alpha`.

```julia
acceptCriteria = NILS.Metropolis(100.0, 0.995)
#                                 ^initial temperature  ^cooling factor (alpha < 1)
```

- higher `temperature` → accepts more worse solutions early on (more
  exploration).
- `alpha` close to 1 → slow cooling, more "hot" iterations.

The best feasible solution found (`bestFeasSol`, on the `Solver`) is always
updated regardless of the acceptance decision — `Metropolis` only controls
what becomes the *current* solution for the next iteration, not what's
reported as the final result.

Pair it with [`ByTemperature`](stop-criteria.md#bytemperatureminttemp) as
the stop criterion — the search ends once the temperature cools enough.

## `MetropolisTimed(initialTemperature, temperature, maxTime, p)`

A `Metropolis` variant where the temperature is recomputed each iteration as
a function of elapsed time, rather than a fixed multiplicative factor:

```
temperature = initialTemperature * (1 - elapsed_time / maxTime) ^ p
```

That is, the temperature hits zero exactly when `maxTime` seconds have
passed since the search started, following a curve controlled by `p`
(`p = 1`: linear decay; `p > 1`: slower decay early on and faster near the
end; `p < 1`: the opposite).

```julia
acceptCriteria = NILS.MetropolisTimed(
    100.0,   # initialTemperature
    100.0,   # temperature (mutable current value — usually equal to the above)
    600.0,   # maxTime, in seconds
    2.0      # p
)
```

Use it when the stop criterion is time-based (`ByTime`) and you want the
cooling schedule to exactly track the available time budget, instead of
depending on how many iterations fit in that time.

## `RandomWalk()`

Always accepts the candidate as the current solution if it's better than
the current one. Unlike `AcceptBest`, "better" here compares against
`currSol`, not `bestSol` — meaning the current solution can drift worse
over time without ever being "pulled back" to the best one seen, except
through the `bestFeasSol` bookkeeping.

```julia
acceptCriteria = NILS.RandomWalk()
```

A more permissive criterion, mostly useful for debugging or as a baseline
to compare against more elaborate criteria.
