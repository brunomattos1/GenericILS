"""Supertype of accept criteria: decide which candidate becomes the current/best solution."""
abstract type AcceptCriteria end
"""Supertype of stopping criteria."""
abstract type StoppingCriteria end

"""Perturbation intensity: number of random shifts and swaps applied per iteration."""
struct Diversification
    shift::Int
    swap::Int
end
Diversification(; shift = 2, swap = 0) = Diversification(shift, swap)

"""
Metropolis acceptance: worse candidates are accepted with probability `exp(-Δ / temperature)`;
the temperature is multiplied by `alpha` at every iteration.
"""
mutable struct Metropolis <: AcceptCriteria
    temperature::Float64
    alpha::Float64
end

"""
Metropolis acceptance whose temperature decays with elapsed time: `initialTemperature * (1 - t/maxTime)^p`,
reaching 0 at `maxTime` seconds.
"""
mutable struct MetropolisTimed <: AcceptCriteria
    initialTemperature::Float64
    temperature::Float64
    maxTime::Float64
    p::Float64
end

"""Variant of `MetropolisTimed` driven by iterations (`accept!` with the `(currSol, bestSol)` signature)."""
mutable struct MetropolisTimedIter <: AcceptCriteria
    initialTemperature::Float64
    temperature::Float64
    maxTime::Float64
    p::Float64
    iter::Int
end

"""Only candidates that improve the best solution are accepted."""
struct AcceptBest <: AcceptCriteria
end

"""Every candidate is accepted; the best feasible solution is still tracked."""
struct RandomWalk <: AcceptCriteria
end

"""Stop after `maxTime` seconds."""
struct ByTime <: StoppingCriteria
    maxTime::Float64
end

"""Stop when the accept criterion temperature falls to `minTemp`."""
struct ByTemperature <: StoppingCriteria
    minTemp::Float64
end

"""Iteration-driven version of `ByTemperature`."""
struct ByTemperatureIter <: StoppingCriteria
    minTemp::Float64
end

"""Stop after `maxIter` iterations without a new best feasible solution."""
struct ByIterMax <: StoppingCriteria
    maxIter::Int
end

"""
Single-layer ILS: each iteration perturbs, runs one local search (RVND) and accepts/rejects the result.
Ends with Set Partitioning over the stored routes.
"""
mutable struct ILSAlgorithm{AC <: AcceptCriteria, SC <: StoppingCriteria, FL, BL} <: Algorithm
    acceptCriteria::AC
    stopCriteria::SC
    diversification::Diversification
    candidateSol::Solution{FL, BL}
    currSol::Solution{FL, BL}
    bestSol::Solution{FL, BL}
    iter::Int
    startTime::Float64
    timeLimitILS::Float64
    timeLimitSP::Float64
    aggressivePool::Bool
    routeStorage::Vector{Vector{Int}}
    costStorage::Vector{Float64}
    routeLookup::Dict{Vector{Int}, Int}
end

"""
Build an `ILSAlgorithm`. `res` is only used to infer the label types. Defaults: Metropolis(100, 0.995) with
`ByTemperature(1.0)`.
"""
function ILSAlgorithm(res::AbstractResources;
    acceptCriteria::AcceptCriteria = Metropolis(100.0, 0.995),
    stopCriteria::StoppingCriteria = ByTemperature(1.0),
    diversification = Diversification(shift = 2, swap = 0),
    timeLimitILS = 3600.0,
    timeLimitSP = 3600.0,
    aggressivePool = false)
    FL = typeof(myInitStateForward(res.customResource))
    BL = typeof(myInitStateBackward(res.customResource))
    ILSAlgorithm{typeof(acceptCriteria), typeof(stopCriteria), FL, BL}(
        acceptCriteria, stopCriteria, diversification,
        Solution{FL, BL}(), Solution{FL, BL}(), Solution{FL, BL}(),
        0, 0.0, timeLimitILS, timeLimitSP, aggressivePool,
        Vector{Vector{Int}}(), Vector{Float64}(), Dict{Vector{Int}, Int}()
    )
end

getDiversification(algo::ILSAlgorithm) = algo.diversification
setDiversification!(algo::ILSAlgorithm, diversification::Diversification) = (algo.diversification = diversification; algo)

getAcceptCriteria(algo::ILSAlgorithm) = algo.acceptCriteria
setAcceptCriteria!(algo::ILSAlgorithm{AC}, acceptCriteria::AC) where {AC} = (algo.acceptCriteria = acceptCriteria; algo)

getStopCriteria(algo::ILSAlgorithm) = algo.stopCriteria
setStopCriteria!(algo::ILSAlgorithm{AC, SC}, stopCriteria::SC) where {AC, SC} = (algo.stopCriteria = stopCriteria; algo)

"""Time limits in seconds as `(ils, sp)`."""
getTimeLimits(algo::ILSAlgorithm) = (ils = algo.timeLimitILS, sp = algo.timeLimitSP)
"""Set both time limits (search phase and Set Partitioning)."""
function setTimeLimits!(algo::ILSAlgorithm, timeLimitILS::Float64, timeLimitSP::Float64)
    algo.timeLimitILS = timeLimitILS
    algo.timeLimitSP  = timeLimitSP
    return algo
end

"""Set the time limit (seconds) of the search phase."""
function setTimeLimitILS(solver::Solver, time::Float64)
    solver.algorithm.timeLimitILS = time
end

"""Set the time limit (seconds) of the Set Partitioning MIP."""
function setTimeLimitSP(solver::Solver, time::Float64)
    solver.algorithm.timeLimitSP = time
end

"""If `agg` is true, also register routes in the Set Partitioning pool during the inner iterations."""
function aggressivePool(solver::Solver, agg::Bool)
    solver.algorithm.aggressivePool = agg
end

"""Seconds since the algorithm started."""
totalTime(algo::ILSAlgorithm) = time() - algo.startTime

"""Current temperature of the accept criterion (`NaN` if it has none)."""
currentTemperature(algo::ILSAlgorithm) = hasproperty(algo.acceptCriteria, :temperature) ? algo.acceptCriteria.temperature : NaN

"""Record when (time, temperature, iteration) the best feasible solution was found."""
function registerBestFeasible!(solver::Solver, sol::Solution)
    stats = solver.statistics
    algo  = solver.algorithm
    stats.bestFeasCost     = sol.cost
    stats.foundTime        = totalTime(algo)
    stats.foundTemperature = currentTemperature(algo)
    stats.foundIter        = algo.iter
end

"""Record the number of routes stored in the Set Partitioning pool."""
function registerPoolSize!(solver::Solver)
    solver.statistics.poolSize = length(solver.algorithm.routeStorage)
end

"""Record the total running time."""
function registerTotalTime!(solver::Solver)
    solver.statistics.totalTime = totalTime(solver.algorithm)
end
