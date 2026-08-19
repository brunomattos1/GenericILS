# A passive, at-rest solution record: giant tour + its route decomposition
# plus cached scalar evaluation/diversity data. No labels, no RVND pruning
# cache (Solver's Solution.lastEval) -- those are only useful transiently
# during education and live on the single shared workspace Solution
# (HGSAlgorithm.ws), reused across every individual, not duplicated per
# population member. routes[r] is depot-inclusive ([0, c1, ..., ck, 0]),
# matching Route.visits' convention -- see storeIndividual!.
mutable struct Individual
    giantTour::Vector{Int}
    routes::Vector{Vector{Int}}
    cost::Float64
    dist::Float64
    totalInfeas::Int
    totalWarpStd1::Float64
    totalWarpStd2::Float64
    biasedFitness::Float64
    successor::Vector{Int32}
    predecessor::Vector{Int32}
end

function Individual(solver::Solver)
    n = length(solver.data.vertices)
    Individual(
        Vector{Int}(undef, n),
        Vector{Vector{Int}}(),
        Inf, Inf, typemax(Int), Inf, Inf,
        0.0,
        zeros(Int32, n),
        zeros(Int32, n)
    )
end

isFeasible(ind::Individual) = ind.totalInfeas == 0 && ind.totalWarpStd1 <= 1e-6 && ind.totalWarpStd2 <= 1e-6

function assign!(dst::Individual, src::Individual)
    resize!(dst.giantTour, length(src.giantTour))
    copyto!(dst.giantTour, src.giantTour)

    length(dst.routes) == length(src.routes) || resize!(dst.routes, length(src.routes))
    for k in eachindex(src.routes)
        if !isassigned(dst.routes, k)
            dst.routes[k] = copy(src.routes[k])
        else
            resize!(dst.routes[k], length(src.routes[k]))
            copyto!(dst.routes[k], src.routes[k])
        end
    end

    dst.cost = src.cost
    dst.dist = src.dist
    dst.totalInfeas = src.totalInfeas
    dst.totalWarpStd1 = src.totalWarpStd1
    dst.totalWarpStd2 = src.totalWarpStd2
    dst.biasedFitness = src.biasedFitness

    resize!(dst.successor, length(src.successor))
    copyto!(dst.successor, src.successor)
    resize!(dst.predecessor, length(src.predecessor))
    copyto!(dst.predecessor, src.predecessor)
    return dst
end

# Copies the (already Split+RVND-educated) shared workspace ws into ind:
# routes (visits only, no labels), scalar cost/feasibility, and rebuilds
# successor/predecessor for diversity. Does NOT touch ind.giantTour -- the
# caller is responsible for that (see run.jl), since ws has no giant-tour
# concept of its own.
function storeIndividual!(ind::Individual, ws::Solution)
    numRoutes = length(ws.routes)
    resize!(ind.routes, numRoutes)
    for r in 1:numRoutes
        if !isassigned(ind.routes, r)
            ind.routes[r] = Int[]
        end
        resize!(ind.routes[r], length(ws.routes[r].visits))
        copyto!(ind.routes[r], ws.routes[r].visits)
    end

    ind.cost = ws.cost
    ind.dist = ws.dist
    ind.totalInfeas = ws.totalInfeas
    ind.totalWarpStd1 = ws.totalWarpStd1
    ind.totalWarpStd2 = ws.totalWarpStd2

    rebuildSuccessorsPredecessors!(ind)
    return ind
end

# Rebuilds successor/predecessor from ind.routes; each route is
# [depot, c1, ..., ck, depot] (depot = 0), so customers occupy positions
# 2:end-1.
function rebuildSuccessorsPredecessors!(ind::Individual)
    fill!(ind.successor, 0)
    fill!(ind.predecessor, 0)
    for route in ind.routes
        m = length(route)
        for k in 2:(m - 2)
            ind.successor[route[k]] = route[k + 1]
            ind.predecessor[route[k + 1]] = route[k]
        end
    end
    return ind
end
