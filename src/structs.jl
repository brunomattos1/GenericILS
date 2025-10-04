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
    resize!(dest.forwardLabels, length(src.forwardLabels))
    resize!(dest.backwardLabels, length(src.backwardLabels))
    for i in 1:length(src.routes)
        # Check if the inner vector is undefined or has a different size
        if !isassigned(dest.routes, i)
            dest.routes[i] = similar(src.routes[i])  # aloca uma vez só
        end
        resize!(dest.routes[i], length(src.routes[i]))
        copyto!(dest.routes[i], src.routes[i])

        if !isassigned(dest.forwardLabels, i)
            dest.forwardLabels[i] = similar(src.forwardLabels[i])
        end
        resize!(dest.forwardLabels[i], length(src.forwardLabels[i]))
        copyto!(dest.forwardLabels[i], src.forwardLabels[i])

        if !isassigned(dest.backwardLabels, i)
            dest.backwardLabels[i] = similar(src.backwardLabels[i])
        end
        resize!(dest.backwardLabels[i], length(src.backwardLabels[i]))
        copyto!(dest.backwardLabels[i], src.backwardLabels[i])
    end

    # Copy nested vectors of labels (forwardLabels, backwardLabels)
    # resize!(dest.forwardLabels, length(src.forwardLabels))
    # resize!(dest.backwardLabels, length(src.backwardLabels))

    # for i in 1:length(src.forwardLabels)
    #     # Check if the inner vector is undefined or needs resizing
    #     if !isassigned(dest.forwardLabels, i)
    #         dest.forwardLabels[i] = similar(src.forwardLabels[i])
    #     end
    #     resize!(dest.forwardLabels[i], length(src.forwardLabels[i]))
    #     copyto!(dest.forwardLabels[i], src.forwardLabels[i])

    #     if !isassigned(dest.backwardLabels, i)
    #         dest.backwardLabels[i] = similar(src.backwardLabels[i])
    #     end
    #     resize!(dest.backwardLabels[i], length(src.backwardLabels[i]))
    #     copyto!(dest.backwardLabels[i], src.backwardLabels[i])
    # end

    # resize!(dest.backwardLabels, length(src.backwardLabels))
    # for i in 1:length(src.backwardLabels)
    #     if !isassigned(dest.forwardLabels, i)
    #         dest.forwardLabels[i] = similar(src.forwardLabels[i])
    #     end
    #     resize!(dest.forwardLabels[i], length(src.forwardLabels[i]))
    #     copyto!(dest.forwardLabels[i], src.forwardLabels[i])
    # end
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
    penaltyCustomIncrease::Float64
    penaltyCustomDecrease::Float64
    penaltyStandard::Float64
    penaltyStandardIncrease::Float64
    penaltyStandardDecrease::Float64

end
function Parameters(;
    restarts = 10,
    outerIterMax = 10,
    innerIterMax = 3,
    penaltyCustom = 100.0,
    penaltyCustomIncrease = 0.01,
    penaltyCustomDecrease = 0.01,
    penaltyStandard = 100.0,
    penaltyStandardIncrease = 0.01,
    penaltyStandardDecrease = 0.01)
    return Parameters(restarts, outerIterMax, innerIterMax, penaltyCustom, penaltyCustomIncrease, penaltyCustomDecrease, penaltyStandard, penaltyStandardIncrease, penaltyStandardDecrease)
end

function updatePenalty(parameters::Parameters, sol::Solution)
    if sol.totalInfeas == 0
        parameters.penaltyCustom = max((1 - parameters.penaltyCustomDecrease)*parameters.penaltyCustom, 0.1)
    else
        parameters.penaltyCustom = min((1 + parameters.penaltyCustomIncrease)*parameters.penaltyCustom, 10000.0)
    end
    if sol.totalWarp <= 1e-6
        parameters.penaltyStandard = max((1 - parameters.penaltyStandardDecrease)*parameters.penaltyStandard, 0.1)
    else
        parameters.penaltyStandard = min((1 + parameters.penaltyStandardIncrease)*parameters.penaltyStandard, 10000.0)
    end
end

struct Diversification
    outerShift::Int
    outerSwap::Int
    innerShift::Int
    innerSwap::Int
end
Diversification(; outerShift = 2, outerSwap = 0, innerShift = 2, innerSwap = 0) = Diversification(outerShift, outerSwap, innerShift, innerSwap)

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
    # pool::Dict{Vector{Int}, Float64}
    # hashes::Set{UInt64}
    route_storage::Vector{Vector{Int}}
    cost_storage::Vector{Float64}
    route_lookup::Dict{Vector{Int}, Int}

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
    buffer = Vector{Int}(),
    buffer2opt = Vector{Int}(),
    bufferRoute = Vector{Int}(),
    # pool = Dict{Vector{Int}, Float64}(),
    # hashes = Set{UInt64}()
    

)
    if DEBUG_MODE
        prevLabelF = myInitStateForward(res.customResource)
        prevLabelStdF = myInitStateForward(res.customResource)
        prevLabelB = myInitStateBackward(res.customResource)
        prevLabelStdB = myInitStateBackward(res.customResource)
    else
        prevLabelF = myInitStateForward(res.customResource)
        prevLabelStdF = myInitStateForward(res.customResource)
        prevLabelB = myInitStateBackward(res.customResource)
        prevLabelStdB = myInitStateBackward(res.customResource)
    end
    route_storage = Vector{Vector{Int}}()
    cost_storage = Vector{Float64}()
    route_lookup = Dict{Vector{Int}, Int}()
    Solver(
        Random.MersenneTwister(seed), params, data, outerCurrSol, bestCurrSol, currSol, bestSol,
        diversification, neighborhoods, auxNeighborhoods, res, stdResource, forwardLabels, backwardLabels, prevLabelF, prevLabelStdF, prevLabelB, prevLabelStdB,
        buffer, buffer2opt, bufferRoute, route_storage, cost_storage, route_lookup#pool, hashes
    )
end

getCurrSol(solver::Solver) = solver.currSol
getBestSol(solver::Solver) = solver.bestSol


getCostMatrix(solver::Solver) = solver.data.costMatrix

