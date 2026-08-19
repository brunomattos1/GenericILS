abstract type StoppingCriteria end

struct ByGenerations <: StoppingCriteria
    maxGenerations::Int
end

struct ByTime <: StoppingCriteria
    maxTime::Float64
end

mutable struct HGSAlgorithm{SC <: StoppingCriteria, FL, BL} <: Algorithm
    stopCriteria::SC
    muMax::Int
    lambda::Int
    nClosest::Int
    nElite::Int
    nbIterNonProd::Int
    granularK::Int
    population::Union{Nothing, Population}
    child::Union{Nothing, Individual}
    ws::Union{Nothing, Solution{FL, BL}}  # shared local-search workspace, reused across every individual's education
    iterationsSinceImprovement::Int
    generation::Int
    startTime::Float64
    visited::BitVector
    freeSlots::Vector{Int}
    split::SplitBuffers
    granularNeighbors::Vector{Vector{Int}}  # customer -> k nearest other customers, built once in run!
    customerRoute::Vector{Int}              # customer -> current route index in ws (RouteIndex.jl)
    customerPos::Vector{Int}                # customer -> current position within that route's visits
    whenLastTested::Vector{Int}             # customer -> ws.timeStamp when last fully tested (don't-look bit)
    indices::Vector{Int}                    # scratch 1:n, shuffled per RVND! pass
    timeLimitSP::Float64
    route_storage::Vector{Vector{Int}}
    cost_storage::Vector{Float64}
    route_lookup::Dict{Vector{Int}, Int}
end

function HGSAlgorithm(res::AbstractResources;
    stopCriteria::StoppingCriteria = ByGenerations(1000),
    muMax::Int = 25,
    lambda::Int = 40,
    nClosest::Int = 5,
    nElite::Int = 4,
    nbIterNonProd::Int = 20000,
    granularK::Int = 10,
    timeLimitSP::Float64 = 60.0)

    FL = typeof(myInitStateForward(res.customResource))
    BL = typeof(myInitStateBackward(res.customResource))

    HGSAlgorithm{typeof(stopCriteria), FL, BL}(
        stopCriteria, muMax, lambda, nClosest, nElite, nbIterNonProd, granularK,
        nothing, nothing, nothing,
        0, 0, 0.0,
        BitVector(), Int[], SplitBuffers(),
        Vector{Int}[], Int[], Int[], Int[], Int[],
        timeLimitSP, Vector{Vector{Int}}(), Vector{Float64}(), Dict{Vector{Int}, Int}()
    )
end

getStopCriteria(algo::HGSAlgorithm) = algo.stopCriteria
setStopCriteria!(algo::HGSAlgorithm{SC}, stopCriteria::SC) where {SC} = (algo.stopCriteria = stopCriteria; algo)

getMuMax(algo::HGSAlgorithm) = algo.muMax
setMuMax!(algo::HGSAlgorithm, muMax::Int) = (algo.muMax = muMax; algo)

getLambda(algo::HGSAlgorithm) = algo.lambda
setLambda!(algo::HGSAlgorithm, lambda::Int) = (algo.lambda = lambda; algo)

getNClosest(algo::HGSAlgorithm) = algo.nClosest
setNClosest!(algo::HGSAlgorithm, nClosest::Int) = (algo.nClosest = nClosest; algo)

getNElite(algo::HGSAlgorithm) = algo.nElite
setNElite!(algo::HGSAlgorithm, nElite::Int) = (algo.nElite = nElite; algo)

getNbIterNonProd(algo::HGSAlgorithm) = algo.nbIterNonProd
setNbIterNonProd!(algo::HGSAlgorithm, nbIterNonProd::Int) = (algo.nbIterNonProd = nbIterNonProd; algo)

getGranularK(algo::HGSAlgorithm) = algo.granularK
setGranularK!(algo::HGSAlgorithm, granularK::Int) = (algo.granularK = granularK; algo)

getTimeLimitSP(algo::HGSAlgorithm) = algo.timeLimitSP
setTimeLimitSP!(algo::HGSAlgorithm, timeLimitSP::Float64) = (algo.timeLimitSP = timeLimitSP; algo)

totalTime(algo::HGSAlgorithm) = time() - algo.startTime

function registerBestFeasible!(solver::Solver, sol::Solution)
    stats = solver.statistics
    algo = solver.algorithm
    stats.bestFeasCost = sol.cost
    stats.foundTime = totalTime(algo)
    stats.foundTemperature = NaN
    stats.foundIter = algo.generation
end

function registerPoolSize!(solver::Solver)
    solver.statistics.poolSize = length(solver.algorithm.route_storage)
end

function registerTotalTime!(solver::Solver)
    solver.statistics.totalTime = totalTime(solver.algorithm)
end
