abstract type Move end
abstract type AcceptCriteria end
abstract type StoppingCriteria end

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

struct BestInsertion
    cost::Float64
    dist::Float64
    route::Int
    customer::Int
    pos::Int
    infeas::Int
    warpStd1::Float64
    warpStd2::Float64
end
BestInsertion() = BestInsertion(Inf, Inf, 0, 0, 0, typemax(Int), Inf, Inf)

struct BestMove
    cost::Float64
    dist::Float64
    firstRoute::Int
    secondRoute::Int
    firstIdx::Int
    secondIdx::Int
end
BestMove(;
    cost::Float64 = Inf,
    dist::Float64 = Inf,
    firstRoute::Int = 0,
    secondRoute::Int = 0,
    firstIdx::Int = 0,
    secondIdx::Int = 0
) = BestMove(cost, dist, firstRoute, secondRoute, firstIdx, secondIdx)

struct Shift <: Move
    routeFrom::Int
    routeTo::Int
    fromIdx::Int
    toIdx::Int
end

struct OptStar <: Move
    firstRoute::Int
    secondRoute::Int
    firstIdx::Int
    secondIdx::Int
end

struct ViolationInfo
    firstRouteInfeas::Int
    secondRouteInfeas::Int
    firstRouteLabelCost::Float64
    secondRouteLabelCost::Float64
end

struct Cost
    dist::Float64
    route1::Int
    route2::Int
    violInfo::ViolationInfo
    warpStd1::Tuple{Float64, Float64}
    warpStd2::Tuple{Float64, Float64}
end

mutable struct Vertex
    id::Int
    resInterval::Vector{Tuple{Float64, Float64}}
end

Base.:(==)(a::Vertex, b::Vertex) = a.id == b.id
Base.hash(v::Vertex, h::UInt) = hash(v.id, h)

struct ProblemData
    vertices::Vector{Vertex}
    costMatrix::Matrix{Float64}
    maxNbRoutes::Int
end
ProblemData() = ProblemData(Vector{Vertex}(), zeros(2,2), 0)

mutable struct Solver{N, AC <: AcceptCriteria, SC <: StoppingCriteria, R <: AbstractResources, FL, BL}
    seed::Random.MersenneTwister
    parameters::Parameters
    data::ProblemData
    outerCandidateSol::Solution{FL, BL}
    outerCurrSol::Solution{FL, BL}
    outerBestSol::Solution{FL, BL}
    bestFeasSol::Solution{FL, BL}
    currSol::Solution{FL, BL}
    bestSol::Solution{FL, BL}
    diversification::Diversification
    acceptCriteria::AC
    stopCriteria::SC
    iter::Int
    innerIter::Int
    startTime::Float64
    neighborhoods::N
    active_neighs::Vector{Int}
    res::R

    forwardLabels::Vector{Vector{FL}}
    backwardLabels::Vector{Vector{BL}}
    prevLabelF::FL
    prevLabelStdF::FL
    prevLabelB::BL
    prevLabelStdB::BL
    buffer::Vector{Int}
    buffer2opt::Vector{Int}
    bufferRoute::Vector{Int}
    bufferSol::Solution{FL, BL}
    route_storage::Vector{Vector{Int}}
    cost_storage::Vector{Float64}
    route_lookup::Dict{Vector{Int}, Int}
    timeStamp::Int
    timeLimitILS::Float64
    timeLimitSP::Float64
    aggressivePool::Bool
end

function Solver(;
    seed = 1,
    parameters = Parameters(restarts = restarts, outerIterMax = outerIterMax, innerIterMax = innerIterMax,
        penaltyCustom = 100.0, penaltyCustomIncrease = 0.01, penaltyCustomDecrease = 0.01,
        penaltyStandard1 = 100.0, penaltyStandard1Increase = 0.01, penaltyStandard1Decrease = 0.01,
        penaltyStandard2 = 100.0, penaltyStandard2Increase = 0.01, penaltyStandard2Decrease = 0.01
    ),
    data = ProblemData(),
    diversification = Diversification(outerShift = 2, outerSwap = 0, innerShift = 2, innerSwap = 0),
    acceptCriteria = Metropolis(100.0, 0.995),
    stopCriteria = ByTemperature(1.0),
    iter = 0,
    innerIter = 0,
    startTime = 0.0,
    neighborhoods = (),
    active_neighs = Int[],
    res,
    timeStamp = 0,
    timeLimitILS = 3600.0,
    timeLimitSP = 3600.0,
    aggressivePool = false)

    prevLabelF    = myInitStateForward(res.customResource)
    prevLabelStdF = myInitStateForward(res.customResource)
    prevLabelB    = myInitStateBackward(res.customResource)
    prevLabelStdB = myInitStateBackward(res.customResource)

    FL = typeof(prevLabelF)
    BL = typeof(prevLabelB)
    R  = typeof(res)

    route_storage = Vector{Vector{Int}}()
    cost_storage  = Vector{Float64}()
    route_lookup  = Dict{Vector{Int}, Int}()

    Solver(
        Random.MersenneTwister(seed), parameters, data,
        Solution{FL, BL}(), Solution{FL, BL}(), Solution{FL, BL}(),
        Solution{FL, BL}(), Solution{FL, BL}(), Solution{FL, BL}(),
        diversification, acceptCriteria, stopCriteria,
        iter, innerIter, startTime, neighborhoods, active_neighs, res,
        Vector{Vector{FL}}(), Vector{Vector{BL}}(),
        prevLabelF, prevLabelStdF, prevLabelB, prevLabelStdB,
        Vector{Int}(), Vector{Int}(), Vector{Int}(),
        Solution{FL, BL}(),
        route_storage, cost_storage, route_lookup,
        timeStamp, timeLimitILS, timeLimitSP, aggressivePool
    )
end

new_solution(::Solver{N, AC, SC, R, FL, BL}) where {N, AC, SC, R, FL, BL} = Solution{FL, BL}()

function setTimeLimitILS(solver::Solver, time::Float64)
    solver.timeLimitILS = time
end

function setTimeLimitSP(solver::Solver, time::Float64)
    solver.timeLimitSP = time
end

function aggressivePool(solver::Solver, agg::Bool)
    solver.aggressivePool = agg
end

getCurrSol(solver::Solver) = solver.currSol
getBestSol(solver::Solver) = solver.bestFeasSol

getBestRoutes(solver::Solver) = solver.bestFeasSol.routes

getBestRoutes(solver::Solver, routes::AbstractVector{Int}) = solver.bestFeasSol.routes[routes]

getBestRoutes(solver::Solver, route::Int) = [solver.bestFeasSol.routes[route]]

getRoutes(sol::Union{Solution, UserSolution}) = sol.routes

getCost(sol::Union{Solution, UserSolution}) = sol.cost

getDistance(sol::Union{Solution, UserSolution}) = sol.dist

getCostMatrix(solver::Solver) = solver.data.costMatrix
