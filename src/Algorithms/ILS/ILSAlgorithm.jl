abstract type AcceptCriteria end
abstract type StoppingCriteria end

struct Diversification
    shift::Int
    swap::Int
end
Diversification(; shift = 2, swap = 0) = Diversification(shift, swap)

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
    route_storage::Vector{Vector{Int}}
    cost_storage::Vector{Float64}
    route_lookup::Dict{Vector{Int}, Int}
end

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

getTimeLimits(algo::ILSAlgorithm) = (ils = algo.timeLimitILS, sp = algo.timeLimitSP)
function setTimeLimits!(algo::ILSAlgorithm, timeLimitILS::Float64, timeLimitSP::Float64)
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

totalTime(algo::ILSAlgorithm) = time() - algo.startTime

currentTemperature(algo::ILSAlgorithm) = hasproperty(algo.acceptCriteria, :temperature) ? algo.acceptCriteria.temperature : NaN

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
