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

function copy_solution!(dest::Solution, src::Solution)
    # Copy scalar fields
    dest.dist = src.dist
    dest.cost = src.cost
    dest.totalInfeas = src.totalInfeas
    dest.totalWarp = src.totalWarp

    # Copy vector fields using resize! and copyto!
    resize!(dest.infeas, length(src.infeas))
    copyto!(dest.infeas, src.infeas)

    resize!(dest.warps, length(src.warps))
    copyto!(dest.warps, src.warps)

    resize!(dest.feasiblesF, length(src.feasiblesF))
    copyto!(dest.feasiblesF, src.feasiblesF)

    resize!(dest.feasiblesB, length(src.feasiblesB))
    copyto!(dest.feasiblesB, src.feasiblesB)

    resize!(dest.lastFeasibleF, length(src.lastFeasibleF))
    copyto!(dest.lastFeasibleF, src.lastFeasibleF)

    resize!(dest.lastFeasibleB, length(src.lastFeasibleB))
    copyto!(dest.lastFeasibleB, src.lastFeasibleB)

    # Copy nested vectors (routes)
    resize!(dest.routes, length(src.routes))
    for i in 1:length(src.routes)
        # Check if the inner vector is undefined or has a different size
        if !isassigned(dest.routes, i) || length(dest.routes[i]) != length(src.routes[i])
            dest.routes[i] = copy(src.routes[i]) # Create a new, correctly sized vector
        else
            copyto!(dest.routes[i], src.routes[i]) # Reuse the existing vector
        end
    end

    # Copy nested vectors of labels (forwardLabels, backwardLabels)
    resize!(dest.forwardLabels, length(src.forwardLabels))
    for i in 1:length(src.forwardLabels)
        # Check if the inner vector is undefined or needs resizing
        if !isassigned(dest.forwardLabels, i)
            dest.forwardLabels[i] = Vector{ForwardLabel}(undef, length(src.forwardLabels[i]))
        else
            resize!(dest.forwardLabels[i], length(src.forwardLabels[i]))
        end
        copyto!(dest.forwardLabels[i], src.forwardLabels[i])
    end

    resize!(dest.backwardLabels, length(src.backwardLabels))
    for i in 1:length(src.backwardLabels)
        if !isassigned(dest.backwardLabels, i)
            dest.backwardLabels[i] = Vector{BackwardLabel}(undef, length(src.backwardLabels[i]))
        else
            resize!(dest.backwardLabels[i], length(src.backwardLabels[i]))
        end
        copyto!(dest.backwardLabels[i], src.backwardLabels[i])
    end

    return dest
end



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
    neighborhoods::Vector{Int}
    auxNeighborhoods::Vector{Int}
    res::Resources
    stdResource::StandardResource
    forwardLabels::Vector{Vector{ForwardLabel}}
    backwardLabels::Vector{Vector{BackwardLabel}}
    prevLabelF::ForwardLabel
    prevLabelStdF::ForwardLabel
    prevLabelB::BackwardLabel
    prevLabelStdB::BackwardLabel
    buffer::Vector{Int}
    buffer2opt::Vector{Int}
    bufferRoute::Vector{Int}
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
    neighborhoods = Int[i for i = 1:10],
    auxNeighborhoods = Int[],
    res = Resources(CustomResource(zeros(Float64, 1, 1), 0.0),StandardResource(zeros(Float64, 1, 1), Float64[], Float64[])),
    stdResource = StandardResource(zeros(Float64, 1, 1), Float64[], Float64[]),
    forwardLabels = Vector{Vector{ForwardLabel}}(),
    backwardLabels = Vector{Vector{BackwardLabel}}(),
    # prevLabelF = Label(0., 0., [0], 0),
    # prevLabelB = Label(0., 0., [0], 0),
    # pool = Vector{Vector{Int}}(),
    buffer = Vector{Int}(),
    buffer2opt = Vector{Int}(),
    bufferRoute = Vector{Int}(),
    pool = Dict{Vector{Int}, Float64}(),
    hashes = Set{UInt64}()
)
    if DEBUG_MODE
        prevLabelF = myInitStateForward()
        prevLabelStdF = myInitStateForward()
        prevLabelB = myInitStateBackward()
        prevLabelStdB = myInitStateBackward()
    else
        prevLabelF = myInitStateForward()
        prevLabelStdF = myInitStateForward()
        prevLabelB = myInitStateBackward()
        prevLabelStdB = myInitStateBackward()
    end
    Solver(
        Random.MersenneTwister(seed), params, data, outerCurrSol, bestCurrSol, currSol, bestSol,
        diversification, neighborhoods, auxNeighborhoods, res, stdResource, forwardLabels, backwardLabels, prevLabelF, prevLabelStdF, prevLabelB, prevLabelStdB,
        buffer, buffer2opt, bufferRoute, pool, hashes
    )
end

getCurrSol(solver::Solver) = solver.currSol
getBestSol(solver::Solver) = solver.bestSol


getCostMatrix(solver::Solver) = solver.data.costMatrix

