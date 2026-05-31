abstract type Move end

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
    infeas::Tuple{Int, Int}
    warpsR1::Tuple{Float64, Float64}
    warpsR2::Tuple{Float64, Float64}
end
BestMove(; 
    cost::Float64 = Inf,
    dist::Float64 = Inf,
    firstRoute::Int = 0,
    secondRoute::Int = 0,
    firstIdx::Int = 0,
    secondIdx::Int = 0,
    infeas::Tuple{Int,Int} = (typemax(Int), typemax(Int)),
    warpsR1::Tuple{Float64,Float64} = (Inf, Inf),
    warpsR2::Tuple{Float64,Float64} = (Inf, Inf)

) = BestMove(cost, dist, firstRoute, secondRoute, firstIdx, secondIdx, infeas, warpsR1, warpsR2)

# struct Insertion <: Move
#     route::Int
#     customer::Int
#     pos::Int
# end

struct Shift <: Move
    routeFrom::Int
    routeTo::Int
    fromIdx::Int
    toIdx::Int
end

# struct Swap <: Move
#     firstRoute::Int
#     secondRoute::Int
#     firstIdx::Int
#     secondIdx::Int
# end

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

# struct Warps
#     warpStd1FirstRoute::Float64
#     warpStd1SecondRoute::Float64
#     warpStd2FirstRoute::Float64
#     warpStd2SecondRoute::Float64
# end


struct Cost
    dist::Float64 # distance
    route1::Int # route 1
    route2::Int # route 2
    violInfo::ViolationInfo # violation info for custom resource
    warpStd1::Tuple{Float64, Float64} # warp for the first and second route (!)
    warpStd2::Tuple{Float64, Float64} # warp for the first and second route (!)
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



mutable struct Solver{N}
    seed::Random.MersenneTwister
    parameters::Parameters
    data::ProblemData
    outerCurrSol::Solution
    outerBestSol::Solution
    bestFeasSol::Solution
    currSol::Solution
    bestSol::Solution
    diversification::Diversification
    neighborhoods::N
    active_neighs::Vector{Int}
    res::Resources

    forwardLabels::Vector{Vector{ForwardLabel}}
    backwardLabels::Vector{Vector{BackwardLabel}}
    prevLabelF::ForwardLabel
    prevLabelStdF::ForwardLabel
    prevLabelB::BackwardLabel
    prevLabelStdB::BackwardLabel
    buffer::Vector{Int}
    buffer2opt::Vector{Int}
    bufferRoute::Vector{Int}
    bufferSol::Solution
    # pool::Vector{Vector{Int}}
    # pool::Dict{Vector{Int}, Float64}
    # hashes::Set{UInt64}
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
    outerCurrSol = Solution(),
    bestCurrSol = Solution(),
    bestFeasSol = Solution(),
    currSol = Solution(),
    bestSol = Solution(),
    diversification = Diversification(outerShift = 2, outerSwap = 0, innerShift = 2, innerSwap = 0),
    neighborhoods = (),
    active_neighs = Int[],
    res = Resources(CustomResource(zeros(Float64, length(data.vertices)+1, length(data.vertices)+1), 0.0),
        StandardResource{1}(zeros(Float64, length(data.vertices)+1, length(data.vertices)+1), Float64[0.0 for i = 1:length(data.vertices)+1], Float64[typemax(Float64) for i = 1:length(data.vertices)+1]), 
        StandardResource{2}(zeros(Float64, length(data.vertices)+1, length(data.vertices)+1), Float64[0.0 for i = 1:length(data.vertices)+1], Float64[typemax(Float64) for i = 1:length(data.vertices)+1])),

    forwardLabels = Vector{Vector{ForwardLabel}}(),
    backwardLabels = Vector{Vector{BackwardLabel}}(),
    buffer = Vector{Int}(),
    buffer2opt = Vector{Int}(),
    bufferRoute = Vector{Int}(),
    bufferSol = Solution(),
    timeStamp = 0,
    timeLimitILS = 3600.0,
    timeLimitSP = 3600.0,
    aggressivePool = false)

    prevLabelF = myInitStateForward(res.customResource)
    prevLabelStdF = myInitStateForward(res.customResource)
    prevLabelB = myInitStateBackward(res.customResource)
    prevLabelStdB = myInitStateBackward(res.customResource)
    
    route_storage = Vector{Vector{Int}}()
    cost_storage = Vector{Float64}()
    route_lookup = Dict{Vector{Int}, Int}()
    
    Solver(
        Random.MersenneTwister(seed), parameters, data, outerCurrSol, bestCurrSol, bestFeasSol, currSol, bestSol,
        diversification, neighborhoods, active_neighs, res, forwardLabels, backwardLabels, prevLabelF, prevLabelStdF, prevLabelB, prevLabelStdB,
        buffer, buffer2opt, bufferRoute, bufferSol, route_storage, cost_storage, route_lookup, timeStamp, timeLimitILS, timeLimitSP,
        aggressivePool)
end


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

