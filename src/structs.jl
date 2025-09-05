mutable struct Solution
    routes::Vector{Vector{Int}} # routes of the solution
    cost::Float64 # total cost of the solution
    feasiblesF::Vector{Int} # number of feasible customers per route forward sense
    feasiblesB::Vector{Int} # number of feasible customers per route backward sense
    lastFeasibleF::Vector{Int} # last feasible position for each route forward sense
    lastFeasibleB::Vector{Int} # last feasible position for each route backward sense
    forwardLabels::Vector{Vector{ForwardLabel}}
    backwardLabels::Vector{Vector{BackwardLabel}}
end

Solution() = Solution(Vector{Vector{Int}}(), 0.0, Vector{Int}(), Vector{Int}(), Vector{Int}(), Vector{Int}(), Vector{ForwardLabel}[], Vector{BackwardLabel}[])

getCost(solution::Solution) = solution.cost
getRoute(solution::Solution, r::Int) = solution.routes[r]
getRoutes(solution::Solution) = solution.routes
getResViolation(solution::Solution) = sum(solution.resViolation)
#getResViolation(solution::Solution, r::Int) = solution.resViolation[r]


struct Parameters
    restarts::Int
    outerIterMax::Int
    innerIterMax::Int
    nbGranular::Int
end
Parameters() = Parameters(10, 100, 5, 20)

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
    res::Resource
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
    res = CapacityResource([[]], 0),
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
        prevLabelF = CapacityState(0., 0., [0], 0)
        prevLabelB = CapacityState(0., 0., [0], 0)
        buffer = CapacityState(0., 0., [0], 0)
    else
        prevLabelF = initStateForward()
        prevLabelB = initStateBackward()
        buffer = initStateForward()
    end
    Solver(
        Random.MersenneTwister(seed), params, data, outerCurrSol, bestCurrSol, currSol, bestSol,
        diversification, neighborhoods, res, initState, extendAlongArc, concatenationCost, forwardLabels, backwardLabels, prevLabelF, prevLabelB,
        buffer, pool, hashes
    )
end

getCurrSol(solver::Solver) = solver.currSol
getBestSol(solver::Solver) = solver.bestSol


getCostMatrix(solver::Solver) = solver.data.costMatrix

