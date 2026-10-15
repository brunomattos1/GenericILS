"""Supertype of solutions."""
abstract type AbstractSolution
end

"""
A route: its visit sequence (depot id `0` at both ends), forward/backward labels and the
incremental bookkeeping (infeasibility, warps, label cost) used by the local search.
"""
mutable struct Route{FL, BL}
    visits::Vector{Int}
    infeas::Int
    warpStd1::Float64
    warpStd2::Float64
    labelCost::Float64
    forwardLabels::Vector{FL}
    backwardLabels::Vector{BL}
    feasibleF::Int
    feasibleB::Int
    lastFeasibleF::Int
    lastFeasibleB::Int
    lastModif::Int
end

"""Empty route."""
function Route{FL, BL}() where {FL, BL}
    Route{FL, BL}(Int[], 0, 0.0, 0.0, 0.0, FL[], BL[], -1, -1, -1, -1, 0)
end

"""Route over `visits`; its labels are filled in by `computeLabels`."""
function Route{FL, BL}(visits::Vector{Int}) where {FL, BL}
    Route{FL, BL}(visits, 0, 0.0, 0.0, 0.0, FL[], BL[], -1, -1, -1, -1, 0)
end

"""
Solution manipulated by the search: routes, total distance, penalized `cost`, per-lane
infeasibility/warp totals and the `lastEval` table that avoids re-evaluating unchanged route pairs.
"""
mutable struct Solution{FL, BL}
    routes::Vector{Route{FL, BL}}
    dist::Float64
    cost::Float64
    totalInfeas::Int
    totalWarpStd1::Float64
    totalWarpStd2::Float64
    totalLabelCost::Float64
    lastEval::Array{Int, 3}
    timeStamp::Int
end

"""Empty solution."""
function Solution{FL, BL}() where {FL, BL}
    Solution{FL, BL}(
        Vector{Route{FL, BL}}(),
        0.0,
        0.0,
        0,
        0.0,
        0.0,
        0.0,
        Array{Int,3}(undef, 5, 10, 10),
        0
    )
end

"""Solution built by the user with `createSolution`; carries routes, distance and cost only."""
mutable struct UserSolution{FL, BL}
    routes::Vector{Route{FL, BL}}
    dist::Float64
    cost::Float64
end

"""Build a `UserSolution` from raw visit vectors (labels are computed later)."""
function UserSolution{FL, BL}(rawRoutes::Vector{Vector{Int}}, dist::Float64, cost::Float64) where {FL, BL}
    UserSolution{FL, BL}([Route{FL, BL}(r) for r in rawRoutes], dist, cost)
end

"""Deep-copy `src` into `dest` (routes, labels, bookkeeping and `lastEval`), reusing the memory of `dest`."""
function copySolution!(dest::Solution{FL, BL}, src::Solution{FL, BL}) where {FL, BL}
    dest.dist = src.dist
    dest.cost = src.cost
    dest.totalInfeas = src.totalInfeas
    dest.totalWarpStd1 = src.totalWarpStd1
    dest.totalWarpStd2 = src.totalWarpStd2
    dest.totalLabelCost = src.totalLabelCost

    resize!(dest.routes, length(src.routes))
    for i in 1:length(src.routes)
        if !isassigned(dest.routes, i)
            dest.routes[i] = Route{FL, BL}()
        end
        rs = src.routes[i]
        rd = dest.routes[i]

        resize!(rd.visits, length(rs.visits))
        copyto!(rd.visits, rs.visits)

        rd.infeas        = rs.infeas
        rd.warpStd1      = rs.warpStd1
        rd.warpStd2      = rs.warpStd2
        rd.labelCost     = rs.labelCost
        rd.feasibleF     = rs.feasibleF
        rd.feasibleB     = rs.feasibleB
        rd.lastFeasibleF = rs.lastFeasibleF
        rd.lastFeasibleB = rs.lastFeasibleB
        rd.lastModif     = rs.lastModif

        resize!(rd.forwardLabels, length(rs.forwardLabels))
        copyto!(rd.forwardLabels, rs.forwardLabels)

        resize!(rd.backwardLabels, length(rs.backwardLabels))
        copyto!(rd.backwardLabels, rs.backwardLabels)
    end

    if size(dest.lastEval) != size(src.lastEval)
        dest.lastEval = similar(src.lastEval)
    end
    copyto!(dest.lastEval, src.lastEval)

    dest.timeStamp = src.timeStamp
end
