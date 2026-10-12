abstract type AcceptCriteria end
abstract type StoppingCriteria end

struct Diversification
    outerShift::Int
    outerSwap::Int
    innerShift::Int
    innerSwap::Int
end
Diversification(; outerShift = 2, outerSwap = 0, innerShift = 2, innerSwap = 0) = Diversification(outerShift, outerSwap, innerShift, innerSwap)

mutable struct Metropolis <: AcceptCriteria
    temperature::Float64
    alpha::Float64
end

mutable struct MetropolisTimed <: AcceptCriteria
    initialTemperature::Float64
    temperature::Float64
    maxTime::Float64
    p::Float64
end

mutable struct MetropolisTimedIter <: AcceptCriteria
    initialTemperature::Float64
    temperature::Float64
    maxTime::Float64
    p::Float64
    iter::Int
end

struct AcceptBest <: AcceptCriteria
end

struct RandomWalk <: AcceptCriteria
end

struct ByTime <: StoppingCriteria
    maxTime::Float64
end

struct ByTemperature <: StoppingCriteria
    minTemp::Float64
end

struct ByTemperatureIter <: StoppingCriteria
    minTemp::Float64
end

struct ByIterMax <: StoppingCriteria
    maxIter::Int
end

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
    route_storage::Vector{Vector{Int}}
    cost_storage::Vector{Float64}
    route_lookup::Dict{Vector{Int}, Int}
end

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

getInnerIterMax(algo::NILSAlgorithm) = algo.innerIterMax
setInnerIterMax!(algo::NILSAlgorithm, innerIterMax::Int) = (algo.innerIterMax = innerIterMax; algo)

getDiversification(algo::NILSAlgorithm) = algo.diversification
setDiversification!(algo::NILSAlgorithm, diversification::Diversification) = (algo.diversification = diversification; algo)

getAcceptCriteria(algo::NILSAlgorithm) = algo.acceptCriteria
setAcceptCriteria!(algo::NILSAlgorithm{AC}, acceptCriteria::AC) where {AC} = (algo.acceptCriteria = acceptCriteria; algo)

getStopCriteria(algo::NILSAlgorithm) = algo.stopCriteria
setStopCriteria!(algo::NILSAlgorithm{AC, SC}, stopCriteria::SC) where {AC, SC} = (algo.stopCriteria = stopCriteria; algo)

getTimeLimits(algo::NILSAlgorithm) = (ils = algo.timeLimitILS, sp = algo.timeLimitSP)
function setTimeLimits!(algo::NILSAlgorithm, timeLimitILS::Float64, timeLimitSP::Float64)
    algo.timeLimitILS = timeLimitILS
    algo.timeLimitSP  = timeLimitSP
    return algo
end

function setTimeLimitILS(solver::Solver, time::Float64)
    solver.algorithm.timeLimitILS = time
end

function setTimeLimitSP(solver::Solver, time::Float64)
    solver.algorithm.timeLimitSP = time
end

function aggressivePool(solver::Solver, agg::Bool)
    solver.algorithm.aggressivePool = agg
end

totalTime(algo::NILSAlgorithm) = time() - algo.startTime

currentTemperature(algo::NILSAlgorithm) = hasproperty(algo.acceptCriteria, :temperature) ? algo.acceptCriteria.temperature : NaN

function registerBestFeasible!(solver::Solver, sol::Solution)
    stats = solver.statistics
    algo  = solver.algorithm
    stats.bestFeasCost     = sol.cost
    stats.foundTime        = totalTime(algo)
    stats.foundTemperature = currentTemperature(algo)
    stats.foundIter        = algo.iter
end

function registerPoolSize!(solver::Solver)
    solver.statistics.poolSize = length(solver.algorithm.route_storage)
end

function registerTotalTime!(solver::Solver)
    solver.statistics.totalTime = totalTime(solver.algorithm)
end
