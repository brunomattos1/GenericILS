"""Supertype of local-search moves."""
abstract type Move end
"""Supertype of search algorithms. A new algorithm implements `run!(algo, solver)`."""
abstract type Algorithm end

"""Fallback: algorithms must define their own `run!`."""
run!(algo::Algorithm, solver) = error("run! not implemented for $(typeof(algo))")

"""Best customer insertion found by the greedy construction (with its predicted violations)."""
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

"""Best move found by a neighborhood: objective `cost`, distance, the two routes and the two positions."""
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

"""Move a customer from `fromIdx` of `routeFrom` to `toIdx` of `routeTo`."""
struct Shift <: Move
    routeFrom::Int
    routeTo::Int
    fromIdx::Int
    toIdx::Int
end

"""2-opt* move: swap the tails of two routes after positions `firstIdx` and `secondIdx`."""
struct OptStar <: Move
    firstRoute::Int
    secondRoute::Int
    firstIdx::Int
    secondIdx::Int
end

"""Predicted infeasibility and label cost of the two routes touched by a move."""
struct ViolationInfo
    firstRouteInfeas::Int
    secondRouteInfeas::Int
    firstRouteLabelCost::Float64
    secondRouteLabelCost::Float64
end

"""Predicted effect of a move on two routes (distance, violations, std1/std2 warps)."""
struct Cost
    dist::Float64
    route1::Int
    route2::Int
    violInfo::ViolationInfo
    warpStd1::Tuple{Float64, Float64}
    warpStd2::Tuple{Float64, Float64}
end

"""
A customer identified by `id` that belongs to `group` (the depot, id `0`, is not part of
`ProblemData.vertices`). A plain customer is `Vertex(id, Group(groupId, 1, false))`.
"""
mutable struct Vertex
    id::Int
    group::Group
end

Base.:(==)(a::Vertex, b::Vertex) = a.id == b.id
Base.hash(v::Vertex, h::UInt) = hash(v.id, h)

"""
Problem data: customers, distance cost matrix (index = id + 1) and maximum number of routes.

The groups come from the vertices: `groups[g]` is the group with `groupId == g`, `members[g]` the ids of its
vertices and `groupOf[id]` the `groupId` of vertex `id`. Vertex ids must be positive and unique, group ids
must be `1:G`, every vertex of a group must carry the same `Group`, and `1 ≤ numVisits ≤ length(members[g])`.
`hasLockedGroups` tells whether some group has to stay in one route (`mustStayInRoute`); when none does, the
moves skip the checks that keep such groups together.
"""
struct ProblemData
    vertices::Vector{Vertex}
    costMatrix::Matrix{Float64}
    maxNbRoutes::Int
    groups::Vector{Group}
    members::Vector{Vector{Int}}
    groupOf::Vector{Int}
    hasLockedGroups::Bool

    function ProblemData(vertices, costMatrix, maxNbRoutes)
        maxId   = isempty(vertices) ? 0 : maximum(v -> v.id, vertices)
        nGroups = isempty(vertices) ? 0 : maximum(v -> v.group.groupId, vertices)
        groupOf = zeros(Int, maxId)
        groups  = Vector{Union{Nothing, Group}}(nothing, nGroups)
        members = [Int[] for _ in 1:nGroups]
        for v in vertices
            v.id >= 1 || throw(ArgumentError("vertex ids must be positive, got $(v.id)"))
            groupOf[v.id] == 0 || throw(ArgumentError("vertex id $(v.id) appears more than once"))
            g = v.group
            g.groupId >= 1 || throw(ArgumentError("group ids must be positive, got $(g.groupId) at vertex $(v.id)"))
            if isnothing(groups[g.groupId])
                groups[g.groupId] = g
            elseif groups[g.groupId] != g
                throw(ArgumentError("vertices of group $(g.groupId) carry different groups: $(groups[g.groupId]) and $g"))
            end
            groupOf[v.id] = g.groupId
            push!(members[g.groupId], v.id)
        end
        for g in 1:nGroups
            isnothing(groups[g]) && throw(ArgumentError("group ids must be 1:$nGroups, group $g has no vertices"))
            1 <= groups[g].numVisits <= length(members[g]) || throw(ArgumentError(
                "group $g needs $(groups[g].numVisits) visits but has $(length(members[g])) vertices"))
        end
        groups = convert(Vector{Group}, groups)
        new(vertices, costMatrix, maxNbRoutes, groups, members, groupOf, any(mustStayInRoute, groups))
    end
end
"""Empty `ProblemData`, a placeholder until `setData!` is called."""
ProblemData() = ProblemData(Vector{Vertex}(), zeros(2,2), 0)

"""Run statistics: best feasible cost (before/after Set Partitioning), when and where it was found."""
mutable struct Statistics
    bestFeasCost::Float64
    bestFeasCostBefSP::Float64
    foundTime::Float64
    foundTemperature::Float64
    foundIter::Int
    poolSize::Int
    totalTime::Float64
end

Statistics() = Statistics(Inf, Inf, NaN, NaN, 0, 0, NaN)

"""
Holds the problem, the resources, the algorithm and every buffer reused by the local search.
Build it with the keyword constructor `Solver(; res, data, neighborhoods, algorithm, ...)`.
"""
mutable struct Solver{N, R <: AbstractResources, PM <: PenaltyManager, FL, BL, AL <: Algorithm}
    seed::Random.MersenneTwister
    algorithm::AL
    penaltyManager::PM
    data::ProblemData
    bestFeasSol::Solution{FL, BL}
    currSol::Solution{FL, BL}
    neighborhoods::N
    activeNeighs::Vector{Int}
    res::R

    forwardLabels::Vector{Vector{FL}}
    backwardLabels::Vector{Vector{BL}}
    prevLabelF::FL
    prevLabelB::BL
    buffer::Vector{Int}
    buffer2opt::Vector{Int}
    bufferRoute::Vector{Int}
    bufferSol::Solution{FL, BL}
    timeStamp::Int
    MIPSolver::Any
    statistics::Statistics
end

"""
Build a solver. `res`, `neighborhoods` and `algorithm` fix its type parameters; seed, penalty manager,
data and MIP solver can be changed later with the setters.
"""
function Solver(;
    seed = 1,
    algorithm::Algorithm,
    penaltyManager = StandardPenaltyManager(
        penaltyCustom = 100.0, penaltyCustomIncrease = 0.01, penaltyCustomDecrease = 0.01,
        penaltyStandard1 = 100.0, penaltyStandard1Increase = 0.01, penaltyStandard1Decrease = 0.01,
        penaltyStandard2 = 100.0, penaltyStandard2Increase = 0.01, penaltyStandard2Decrease = 0.01
    ),
    data = ProblemData(),
    neighborhoods = (),
    activeNeighs = Int[],
    res,
    timeStamp = 0,
    MIPSolver = HiGHS.Optimizer)

    prevLabelF    = myInitStateForward(res.customResource)
    prevLabelB    = myInitStateBackward(res.customResource)

    FL = typeof(prevLabelF)
    BL = typeof(prevLabelB)
    R  = typeof(res)

    bestFeasSol = Solution{FL, BL}()
    bestFeasSol.cost = Inf

    Solver(
        Random.MersenneTwister(seed), algorithm, penaltyManager, data,
        bestFeasSol, Solution{FL, BL}(),
        neighborhoods, activeNeighs, res,
        Vector{Vector{FL}}(), Vector{Vector{BL}}(),
        prevLabelF, prevLabelB,
        Vector{Int}(), Vector{Int}(), Vector{Int}(),
        Solution{FL, BL}(),
        timeStamp, MIPSolver,
        Statistics()
    )
end

newSolution(::Solver{N, R, PM, FL, BL, AL}) where {N, R, PM, FL, BL, AL} = Solution{FL, BL}()

newRoute(::Solver{N, R, PM, FL, BL, AL}, visits::Vector{Int}) where {N, R, PM, FL, BL, AL} = Route{FL, BL}(visits)

getCurrSol(solver::Solver) = solver.currSol

"""Print the routes and cost of the best feasible solution found and return it."""
function getBestSol(solver::Solver)
    sol = solver.bestFeasSol
    r = 0
    for rt in sol.routes
        length(rt.visits) == 2 && continue
        r += 1
        println("Route #$r: ", join(rt.visits, " -> "))
    end
    println("Cost: ", sol.cost)
    return sol
end

getSeed(solver::Solver) = solver.seed
setSeed!(solver::Solver, seed::Random.MersenneTwister) = (solver.seed = seed; solver)

getAlgorithm(solver::Solver{N, R, PM, FL, BL, AL}) where {N, R, PM, FL, BL, AL} = solver.algorithm
setAlgorithm!(solver::Solver{N, R, PM, FL, BL, AL}, algorithm::AL) where {N, R, PM, FL, BL, AL} =
    (solver.algorithm = algorithm; solver)

getPenaltyManager(solver::Solver{N, R, PM}) where {N, R, PM} = solver.penaltyManager
setPenaltyManager!(solver::Solver{N, R, PM}, penaltyManager::PM) where {N, R, PM} =
    (solver.penaltyManager = penaltyManager; solver)

getRes(solver::Solver{N, R}) where {N, R} = solver.res
setRes!(solver::Solver{N, R}, res::R) where {N, R} = (solver.res = res; solver)

getData(solver::Solver) = solver.data
setData!(solver::Solver, data::ProblemData) = (solver.data = data; solver)

getNeighborhoods(solver::Solver{N}) where {N} = solver.neighborhoods
setNeighborhoods!(solver::Solver{N}, neighborhoods::N) where {N} = (solver.neighborhoods = neighborhoods; solver)

getMIPSolver(solver::Solver) = solver.MIPSolver
setMIPSolver!(solver::Solver, mipSolver) = (solver.MIPSolver = mipSolver; solver)

getStatistics(solver::Solver) = solver.statistics

getBestRoutes(solver::Solver) = solver.bestFeasSol.routes

getBestRoutes(solver::Solver, routes::AbstractVector{Int}) = solver.bestFeasSol.routes[routes]

getBestRoutes(solver::Solver, route::Int) = [solver.bestFeasSol.routes[route]]

getRoutes(sol::Union{Solution, UserSolution}) = sol.routes

getCost(sol::Union{Solution, UserSolution}) = sol.cost

getDistance(sol::Union{Solution, UserSolution}) = sol.dist

getCostMatrix(solver::Solver) = solver.data.costMatrix

"""
Store `sol` as the best feasible solution if it is feasible and cheaper. Any algorithm can use this
to publish results, independently of its own bookkeeping. Returns `true` when it was stored.
"""
function updateBestFeasSol!(solver::Solver, sol::Solution)
    if sol.cost < solver.bestFeasSol.cost - 1e-6 &&
       sol.totalInfeas == 0 && sol.totalWarpStd1 <= 1e-6 && sol.totalWarpStd2 <= 1e-6
        copySolution!(solver.bestFeasSol, sol)
        return true
    end
    return false
end

"""Record the best feasible cost reached before the Set Partitioning step."""
function registerBestFeasibleBefSP!(solver::Solver)
    solver.statistics.bestFeasCostBefSP = solver.bestFeasSol.cost
end
