"""Supertype of accept criteria: decide which candidate becomes the current/best solution."""
abstract type AcceptCriteria end
"""Supertype of stopping criteria."""
abstract type StoppingCriteria end

"""Perturbation intensity: number of random shifts/swaps applied per outer and per inner iteration."""
struct Diversification
    outerShift::Int
    outerSwap::Int
    innerShift::Int
    innerSwap::Int
end
Diversification(; outerShift = 2, outerSwap = 0, innerShift = 2, innerSwap = 0) = Diversification(outerShift, outerSwap, innerShift, innerSwap)

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
Nested ILS: an outer loop perturbs and accepts/rejects candidates; each candidate is refined by an
inner loop (`innerIterMax` iterations of local search + light perturbation). Ends with Set Partitioning
over the stored routes.
"""
mutable struct NILSAlgorithm{AC <: AcceptCriteria, SC <: StoppingCriteria, FL, BL} <: Algorithm
    acceptCriteria::AC
    stopCriteria::SC
    diversification::Diversification
    innerIterMax::Int
    outerCandidateSol::Solution{FL, BL}
    outerCurrSol::Solution{FL, BL}
    outerBestSol::Solution{FL, BL}
    bestSol::Solution{FL, BL}
    iter::Int
    innerIter::Int
    startTime::Float64
    timeLimitILS::Float64
    timeLimitSP::Float64
    aggressivePool::Bool
    routeStorage::Vector{Vector{Int}}
    costStorage::Vector{Float64}
    routeLookup::Dict{Vector{Int}, Int}
end

"""
Build a `NILSAlgorithm`. `res` is only used to infer the label types. Defaults: Metropolis(100, 0.995) with
`ByTemperature(1.0)`.
"""
function NILSAlgorithm(res::AbstractResources;
    acceptCriteria::AcceptCriteria = Metropolis(100.0, 0.995),
    stopCriteria::StoppingCriteria = ByTemperature(1.0),
    diversification = Diversification(outerShift = 2, outerSwap = 0, innerShift = 2, innerSwap = 0),
    innerIterMax = 3,
    timeLimitILS = 3600.0,
    timeLimitSP = 3600.0,
    aggressivePool = false)
    FL = typeof(myInitStateForward(res.customResource))
    BL = typeof(myInitStateBackward(res.customResource))
    NILSAlgorithm{typeof(acceptCriteria), typeof(stopCriteria), FL, BL}(
        acceptCriteria, stopCriteria, diversification, innerIterMax,
        Solution{FL, BL}(), Solution{FL, BL}(), Solution{FL, BL}(), Solution{FL, BL}(),
        0, 0, 0.0, timeLimitILS, timeLimitSP, aggressivePool,
        Vector{Vector{Int}}(), Vector{Float64}(), Dict{Vector{Int}, Int}()
    )
end

"""Number of inner iterations without improvement before returning to the outer loop."""
getInnerIterMax(algo::NILSAlgorithm) = algo.innerIterMax
setInnerIterMax!(algo::NILSAlgorithm, innerIterMax::Int) = (algo.innerIterMax = innerIterMax; algo)

getDiversification(algo::NILSAlgorithm) = algo.diversification
setDiversification!(algo::NILSAlgorithm, diversification::Diversification) = (algo.diversification = diversification; algo)

getAcceptCriteria(algo::NILSAlgorithm) = algo.acceptCriteria
setAcceptCriteria!(algo::NILSAlgorithm{AC}, acceptCriteria::AC) where {AC} = (algo.acceptCriteria = acceptCriteria; algo)

getStopCriteria(algo::NILSAlgorithm) = algo.stopCriteria
setStopCriteria!(algo::NILSAlgorithm{AC, SC}, stopCriteria::SC) where {AC, SC} = (algo.stopCriteria = stopCriteria; algo)

"""Time limits in seconds as `(ils, sp)`."""
getTimeLimits(algo::NILSAlgorithm) = (ils = algo.timeLimitILS, sp = algo.timeLimitSP)
"""Set both time limits (search phase and Set Partitioning)."""
function setTimeLimits!(algo::NILSAlgorithm, timeLimitILS::Float64, timeLimitSP::Float64)
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
totalTime(algo::NILSAlgorithm) = time() - algo.startTime

"""Current temperature of the accept criterion (`NaN` if it has none)."""
currentTemperature(algo::NILSAlgorithm) = hasproperty(algo.acceptCriteria, :temperature) ? algo.acceptCriteria.temperature : NaN

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
