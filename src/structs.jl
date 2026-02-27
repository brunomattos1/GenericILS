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

mutable struct ViolationInfo
    firstRouteInfeas::Int
    secondRouteInfeas::Int
    firstRouteLabelCost::Float64
    secondRouteLabelCost::Float64
end

struct Cost
    dist::Float64
    route1::Int
    route2::Int
    #infeas::Tuple{Int, Int}
    violInfo::ViolationInfo
    warp::Tuple{Float64, Float64}
end

abstract type AbstractSolution 
end

mutable struct Solution
    routes::Vector{Vector{Int}} # routes of the solution
    dist::Float64 # total distance
    cost::Float64 # total cost
    totalLabelCost::Float64
    labelCosts::Vector{Float64}
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
    lastEval::Array{Int, 3}#Vector{Vector{Vector{Int}}}#Dict{Tuple{Symbol, Int, Int}, Int}
    lastModif::Vector{Int}
end

mutable struct UserSolution
    routes::Vector{Vector{Int}} # routes of the solution
    dist::Float64 # total distance
    cost::Float64 # total cost
    forwardLabels::Vector{Vector{ForwardLabel}}
    backwardLabels::Vector{Vector{BackwardLabel}}
end

# Solution() = Solution(Vector{Vector{Int}}(), 0.0, 0.0, 0, Vector{Int}(), 0.0, Vector{Float64}(), Vector{Int}(), Vector{Int}(), Vector{Int}(), Vector{Int}(), Vector{ForwardLabel}[], Vector{BackwardLabel}[], Dict{Tuple{Symbol, Int, Int}, Int}(), Vector{Int}())
# Solution() = Solution(Vector{Vector{Int}}(), 0.0, 0.0, 0, Vector{Int}(), 0.0, Vector{Float64}(), Vector{Int}(), Vector{Int}(), Vector{Int}(), Vector{Int}(), Vector{ForwardLabel}[], Vector{BackwardLabel}[], Vector{Vector{Vector{Int}}}(), Vector{Int}())
Solution() = Solution(Vector{Vector{Int}}(), 0.0, 0.0, 0.0, Vector{Int}(), 0, Vector{Int}(), 0.0, Vector{Float64}(), Vector{Int}(), Vector{Int}(), Vector{Int}(), Vector{Int}(), Vector{ForwardLabel}[], Vector{BackwardLabel}[], Array{Int,3}(undef, 4, 10, 10), Vector{Int}())


function copy_solution!(dest::Solution, src::Solution)
    # Copy scalar fields
    dest.dist = src.dist
    dest.cost = src.cost
    dest.totalInfeas = src.totalInfeas
    dest.totalWarp = src.totalWarp
    dest.totalLabelCost = src.totalLabelCost
    
    # Copy vector fields using resize! and copyto!
    resize!(dest.infeas, length(src.infeas))
    copyto!(dest.infeas, src.infeas)

    resize!(dest.warps, length(src.warps))
    copyto!(dest.warps, src.warps)

    resize!(dest.labelCosts, length(src.infeas))
    copyto!(dest.labelCosts, src.labelCosts)

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

    # if dest.lastEval === nothing
    #     dest.lastEval = Dict{Tuple,Int}()
    # else
    #     empty!(dest.lastEval)
    # end
    # for (k,v) in src.lastEval
    #     dest.lastEval[k] = v
    # end
    if dest.lastEval === nothing || size(dest.lastEval) != size(src.lastEval)
        dest.lastEval = similar(src.lastEval)  # cria novo array do mesmo tamanho
    end
    copyto!(dest.lastEval, src.lastEval)

    resize!(dest.lastModif, length(src.lastModif))
    copyto!(dest.lastModif, src.lastModif)

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
    bestFeasSol::Solution
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
    params = Parameters(),
    data = ProblemData(),
    outerCurrSol = Solution(),
    bestCurrSol = Solution(),
    bestFeasSol = Solution(),
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
        Random.MersenneTwister(seed), params, data, outerCurrSol, bestCurrSol, bestFeasSol, currSol, bestSol,
        diversification, neighborhoods, auxNeighborhoods, res, stdResource, forwardLabels, backwardLabels, prevLabelF, prevLabelStdF, prevLabelB, prevLabelStdB,
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
getBestSol(solver::Solver) = solver.outerBestSol

getBestRoutes(solver::Solver) = solver.outerBestSol.routes

getBestRoutes(solver::Solver, routes::AbstractVector{Int}) = solver.outerBestSol.routes[routes]

getBestRoutes(solver::Solver, route::Int) = [solver.outerBestSol.routes[route]]

getRoutes(sol::Union{Solution, UserSolution}) = sol.routes

getCost(sol::Union{Solution, UserSolution}) = sol.cost

getDistance(sol::Union{Solution, UserSolution}) = sol.dist

# -------- Formata apenas os campos internos do state --------
function format_state(state)
    T = typeof(state)
    state_name = nameof(T)

    parts = String[]
    for f in fieldnames(T)
        value = getfield(state, f)
        push!(parts, "$(f): $(value)")
    end

    return "$state_name: ($(join(parts, ", ")))"
end

# -------- Formata o label completo --------
function print_label(route_prefix, label)

    # 1) imprime rota parcial
    print("Partial label: ")
    println(join(route_prefix, " -> "))

    state_parts = String[]
    other_parts = String[]

    for fname in fieldnames(typeof(label))
        value = getfield(label, fname)

        if isstructtype(typeof(value))
            push!(state_parts, format_state(value))
        else
            push!(other_parts, "$(fname): $(value)")
        end
    end
    # 2) imprime states abaixo da rota
    line = "  States: $(join(state_parts, ", "))"

    if !isempty(other_parts)
        line *= ", $(join(other_parts, ", "))"
    end

    println(line)
    println()
end

function print_label(label)
    state_parts = String[]
    other_parts = String[]

    for fname in fieldnames(typeof(label))
        value = getfield(label, fname)

        if isstructtype(typeof(value))
            push!(state_parts, format_state(value))
        else
            push!(other_parts, "$(fname): $(value)")
        end
    end

    all_parts = vcat(
        ["States: $(join(state_parts, ", "))"],
        other_parts
    )

    println("  " * join(all_parts, ", "))
    println()
end


function printLabels(solver::Solver, sol::Union{Solution, UserSolution})
    computeLabels(solver, sol)
    println("#"^100)
    println("FORWARD LABELS:")
    println("#"^100)

    # -------- Seu loop --------
    for r = 1:length(sol.routes)
        println("-"^100)
        println("Route $r: $(join(sol.routes[r], " -> "))")
        println("-"^100)

        for i = 1:length(sol.routes[r])
            route_prefix = sol.routes[r][1:i]
            label = sol.forwardLabels[r][i]
            print_label(route_prefix, label)
        end
    end
    println("#"^100)
    println("BACKWARD LABELS:")
    println("#"^100)

    for r = 1:length(sol.routes)
        println("-"^100)
        println("Route $r: $(join(sol.routes[r], " -> "))")
        println("-"^100)
        n = length(sol.routes[r])
        for i = 1:n
            start = n - i + 1
            route_prefix = sol.routes[r][start:n]
            label = sol.backwardLabels[r][i]
            print_label(route_prefix, label)
        end
    end
end

function printConcatenations(solver::Solver, sol::Union{Solution, UserSolution})
    for r = 1:length(sol.routes)
        println("-"^100)
        println("Route $r: $(join(sol.routes[r], " -> "))")
        println("-"^100)
        lenR = length(sol.routes[r])
        for i in 1:lenR
            prefix = sol.routes[r][1:i]
            suffix = sol.routes[r][i:end]
            concat = myConcatenationCost(
                solver.res,
                1,
                sol.forwardLabels[r][i],
                sol.backwardLabels[r][lenR - i + 1]
            )
            println("Concatenating $(join(prefix, " -> ")) with $(join(suffix, " -> "))")
            print_label(concat)
        end
    end
end

function createSolution(solver::Solver, dist::Float64, cost::Float64, routes::Vector{Vector{Int}})
    sol = UserSolution(
        routes,
        dist,
        cost,
        Vector{Vector{ForwardLabel}}(),
        Vector{Vector{BackwardLabel}}(),
    )
    computeLabels(solver, sol)
    return sol
end


getCostMatrix(solver::Solver) = solver.data.costMatrix

