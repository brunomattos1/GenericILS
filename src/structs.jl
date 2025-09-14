abstract type Move end

struct BestInsertion
    cost::Float64
    dist::Float64
    route::Int
    customer::Int
    pos::Int
    infeas::Int
    warp::Float64
end
BestInsertion() = BestInsertion(Inf, Inf, 0, 0, 0, typemax(Int), Inf)

struct BestMove 
    cost::Float64
    dist::Float64
    firstRoute::Int
    secondRoute::Int
    firstIdx::Int
    secondIdx::Int
    infeas::Tuple{Int, Int}
    warps::Tuple{Float64, Float64}
end
BestMove(; 
    cost::Float64 = Inf,
    dist::Float64 = Inf,
    firstRoute::Int = 0,
    secondRoute::Int = 0,
    firstIdx::Int = 0,
    secondIdx::Int = 0,
    infeas::Tuple{Int,Int} = (typemax(Int), typemax(Int)),
    warps::Tuple{Float64,Float64} = (Inf, Inf)
) = BestMove(cost, dist, firstRoute, secondRoute, firstIdx, secondIdx, infeas, warps)

struct Insertion <: Move
    route::Int
    customer::Int
    pos::Int
end

struct Shift <: Move
    routeFrom::Int
    routeTo::Int
    fromIdx::Int
    toIdx::Int
end

struct Swap <: Move
    firstRoute::Int
    secondRoute::Int
    firstIdx::Int
    secondIdx::Int
end

struct OptStar <: Move
    firstRoute::Int
    secondRoute::Int
    firstIdx::Int
    secondIdx::Int
end

struct Cost
    dist::Float64
    route1::Int
    route2::Int
    infeas::Tuple{Int, Int}
    warp::Tuple{Float64, Float64}
end

mutable struct Solution
    routes::Vector{Vector{Int}} # routes of the solution
    dist::Float64 # total distance
    cost::Float64 # total cost
    totalInfeas::Int
    infeas::Vector{Int}
    totalWarp::Float64
    warps::Vector{Float64}
    feasiblesF::Vector{Int} # number of feasible customers per route forward sense
    feasiblesB::Vector{Int} # number of feasible customers per route backward sense
    lastFeasibleF::Vector{Int} # last feasible position for each route forward sense
    lastFeasibleB::Vector{Int} # last feasible position for each route backward sense
    forwardLabels::Vector{Vector{ForwardLabel}}
    backwardLabels::Vector{Vector{BackwardLabel}}
end

Solution() = Solution(Vector{Vector{Int}}(), 0.0, 0.0, 0, Vector{Int}(), 0.0, Vector{Float64}(), Vector{Int}(), Vector{Int}(), Vector{Int}(), Vector{Int}(), Vector{ForwardLabel}[], Vector{BackwardLabel}[])

getCost(solution::Solution) = solution.cost
getRoute(solution::Solution, r::Int) = solution.routes[r]
getRoutes(solution::Solution) = solution.routes
getResViolation(solution::Solution) = sum(solution.resViolation)
#getResViolation(solution::Solution, r::Int) = solution.resViolation[r]


mutable struct Parameters
    restarts::Int
    outerIterMax::Int
    innerIterMax::Int
    penaltyCustom::Float64
    penaltyStandard::Float64
    penaltyCustomFactor::Float64
    penaltyStandardFactor::Float64

end
Parameters() = Parameters(10, 100, 5, 10, 10, 0.05, 0.01)

function updatePenalty(parameters::Parameters, sol::Solution)
    if sol.totalInfeas == 0
        parameters.penaltyCustom = max((1 - parameters.penaltyCustomFactor)*parameters.penaltyCustom, 1.0)
    else
        parameters.penaltyCustom = min((1 + parameters.penaltyCustomFactor)*parameters.penaltyCustom, 1000.0)
    end
    if sol.totalWarp <= 1e-6
        parameters.penaltyStandard = max((1 - parameters.penaltyStandardFactor)*parameters.penaltyStandard, 1.0)
    else
        parameters.penaltyStandard = min((1 + parameters.penaltyStandardFactor)*parameters.penaltyStandard, 1000.0)
    end
end

struct Diversification
    outerShift::Int
    outerSwap::Int
    innerShift::Int
    innerSwap::Int
end
Diversification() = Diversification(2, 0, 2, 0)

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

mutable struct Solver
    seed::Random.MersenneTwister
    params::Parameters
    data::ProblemData
    outerCurrSol::Solution
    outerBestSol::Solution
    currSol::Solution
    bestSol::Solution
    diversification::Diversification
    neighborhoods::Set{Int}
    res::Resources
    stdResource::StandardResource
    initState::Function
    extendAlongArc::Function
    concatenationCost::Function
    forwardLabels::Vector{Vector{ForwardLabel}}
    backwardLabels::Vector{Vector{BackwardLabel}}
    prevLabelF::ForwardLabel
    prevLabelB::BackwardLabel
    buffer::ForwardLabel
    # pool::Vector{Vector{Int}}
    pool::Dict{Vector{Int}, Float64}
    hashes::Set{UInt64}

end

function Solver(; 
    seed = 1,
    params = Parameters(),
    data = ProblemData(),
    outerCurrSol = Solution(),
    bestCurrSol = Solution(),
    currSol = Solution(),
    bestSol = Solution(),
    diversification = Diversification(),
    neighborhoods = Set(i for i = 1:10),
    res = Resources(CustomResource(zeros(Float64, 1, 1), 0.0),StandardResource(zeros(Float64, 1, 1), Float64[], Float64[])),
    stdResource = StandardResource(zeros(Float64, 1, 1), Float64[], Float64[]),
    initState = x -> x,
    extendAlongArc = x -> x,
    concatenationCost = x -> x,
    forwardLabels = Vector{Vector{ForwardLabel}}(),
    backwardLabels = Vector{Vector{BackwardLabel}}(),
    # prevLabelF = Label(0., 0., [0], 0),
    # prevLabelB = Label(0., 0., [0], 0),
    # pool = Vector{Vector{Int}}(),
    pool = Dict{Vector{Int}, Float64}(),
    hashes = Set{UInt64}()
)
    if DEBUG_MODE
        prevLabelF = myInitStateForward()
        prevLabelB = myInitStateBackward()
        buffer = myInitStateForward()
    else
        prevLabelF = myInitStateForward()
        prevLabelB = myInitStateBackward()
        buffer = myInitStateForward()
    end
    Solver(
        Random.MersenneTwister(seed), params, data, outerCurrSol, bestCurrSol, currSol, bestSol,
        diversification, neighborhoods, res, stdResource, initState, extendAlongArc, concatenationCost, forwardLabels, backwardLabels, prevLabelF, prevLabelB,
        buffer, pool, hashes
    )
end

getCurrSol(solver::Solver) = solver.currSol
getBestSol(solver::Solver) = solver.bestSol


getCostMatrix(solver::Solver) = solver.data.costMatrix

