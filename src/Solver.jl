abstract type Move end
abstract type Algorithm end

run!(algo::Algorithm, solver) = error("run! not implemented for $(typeof(algo))")

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
end

Base.:(==)(a::Vertex, b::Vertex) = a.id == b.id
Base.hash(v::Vertex, h::UInt) = hash(v.id, h)

struct ProblemData
    vertices::Vector{Vertex}
    costMatrix::Matrix{Float64}
    maxNbRoutes::Int
end
ProblemData() = ProblemData(Vector{Vertex}(), zeros(2,2), 0)

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

mutable struct Solver{N, R <: AbstractResources, PM <: PenaltyManager, FL, BL, AL <: Algorithm}
    seed::Random.MersenneTwister
    algorithm::AL
    penaltyManager::PM
    data::ProblemData
    bestFeasSol::Solution{FL, BL}
    currSol::Solution{FL, BL}
    neighborhoods::N
    active_neighs::Vector{Int}
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
    active_neighs = Int[],
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
        neighborhoods, active_neighs, res,
        Vector{Vector{FL}}(), Vector{Vector{BL}}(),
        prevLabelF, prevLabelB,
        Vector{Int}(), Vector{Int}(), Vector{Int}(),
        Solution{FL, BL}(),
        timeStamp, MIPSolver,
        Statistics()
    )
end

new_solution(::Solver{N, R, PM, FL, BL, AL}) where {N, R, PM, FL, BL, AL} = Solution{FL, BL}()

new_route(::Solver{N, R, PM, FL, BL, AL}, visits::Vector{Int}) where {N, R, PM, FL, BL, AL} = Route{FL, BL}(visits)

getCurrSol(solver::Solver) = solver.currSol

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

# Generic result reporting: any algorithm can call this to publish its best
# feasible solution, independently of algorithm-specific bookkeeping (iter,
# temperature, statistics, ...).
function updateBestFeasSol!(solver::Solver, sol::Solution)
    if sol.cost < solver.bestFeasSol.cost - 1e-6 &&
       sol.totalInfeas == 0 && sol.totalWarpStd1 <= 1e-6 && sol.totalWarpStd2 <= 1e-6
        copy_solution!(solver.bestFeasSol, sol)
        return true
    end
    return false
end

function registerBestFeasibleBefSP!(solver::Solver)
    solver.statistics.bestFeasCostBefSP = solver.bestFeasSol.cost
end
