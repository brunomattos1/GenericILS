"""
A group of vertices. Exactly `numVisits` of its regular (non optional) members must be visited by the solution,
each visited member once; the optional members do not count and may be visited or not. When `sameRoute` is
`true` all the visited members, optional or not, must be in the same route.

A plain customer is a group of one vertex with `numVisits = 1`; an optional customer is a group of one optional
vertex with `numVisits = 0`.
"""
struct Group
    groupId::Int
    numVisits::Int
    sameRoute::Bool
end

"""
Whether the visits of `group` have to stay in a single route: it is `sameRoute` and can have several visits, that
is, it needs more than one or has optional members (`nOptional` of them) that may add to those it needs.
With at most one visit there is nothing to keep together.
"""
mustStayInRoute(group::Group, nOptional::Int = 0) = group.sameRoute && group.numVisits + nOptional > 1

"""Number of visits that group `g`, which must stay in one route, has in `route` (all of its visits are there)."""
function visitsInRoute(data, route::Vector{Int}, g::Int)
    data.nOptional[g] == 0 && return data.groups[g].numVisits     # nothing varies
    visits = 0
    for v in route
        v != 0 && data.groupOf[v] == g && (visits += 1)
    end
    return visits
end

"""
Whether the block of `k` entries of `route` starting at `i` can go to another route without splitting a group:
for every group that must stay in one route, the block holds either none of its visits or all of them. (The
visits of such a group are all in `route`, so a block that holds only some of them would leave the others
behind.) `data` is the `ProblemData`.
"""
function canLeaveRoute(data, route::Vector{Int}, i::Int, k::Int)
    data.hasLockedGroups || return true
    for p in i:i+k-1
        g = data.groupOf[route[p]]
        data.locked[g] || continue
        inBlock = 0
        for q in i:i+k-1
            data.groupOf[route[q]] == g && (inBlock += 1)
        end
        inBlock == visitsInRoute(data, route, g) || return false
    end
    return true
end

"""
Fill `safe[i]` for `i in 1:length(visits)` with whether cutting `visits` after position `i` (head `1:i`, tail
`i+1:end`) keeps every group that must stay in one route on one side of the cut. `seen` is scratch space with one
zero per group, and is left zeroed.
"""
function safeCuts!(safe::Vector{Bool}, seen::Vector{Int}, data, visits::Vector{Int})
    n = length(visits)
    resize!(safe, n)
    if !data.hasLockedGroups
        fill!(safe, true)
        return safe
    end
    for p in 2:n-1                                   # visits of each such group in this route
        g = data.groupOf[visits[p]]
        data.locked[g] && (seen[g] += 1)
    end
    open = 0
    safe[1] = true
    for p in 2:n-1
        g = data.groupOf[visits[p]]
        if data.locked[g]
            left = seen[g]               # still positive before the first visit of the group, then -(visits left)
            if left > 0
                open += 1
                left = -left
            end
            left += 1
            seen[g] = left
            left == 0 && (open -= 1)
        end
        safe[p] = open == 0
    end
    safe[n] = true
    return safe
end

"""
Group constraints broken by `routes` (vectors of visits, depot id `0` at both ends), as human-readable
messages; empty when the routes satisfy them. A solution satisfies them when every vertex is visited at most
once, every group has exactly `numVisits` visited regular members (the optional ones do not count) and the
visited members of a `sameRoute` group, optional or not, are in a single route.
"""
function groupViolations(data, routes::AbstractVector)
    msgs = String[]
    nGroups = length(data.groups)
    count = zeros(Int, nGroups)
    firstRoute = zeros(Int, nGroups)
    seen = Set{Int}()
    for (r, visits) in enumerate(routes)
        for v in visits
            v == 0 && continue
            if v < 1 || v > length(data.groupOf) || data.groupOf[v] == 0
                push!(msgs, "route $r visits unknown vertex $v")
                continue
            end
            v in seen && push!(msgs, "vertex $v is visited more than once")
            push!(seen, v)
            g = data.groupOf[v]
            data.isOptional[v] || (count[g] += 1)
            if data.groups[g].sameRoute
                firstRoute[g] == 0 && (firstRoute[g] = r)
                firstRoute[g] != r && push!(msgs, "group $g is split between routes $(firstRoute[g]) and $r")
            end
        end
    end
    for g in 1:nGroups
        count[g] != data.groups[g].numVisits &&
            push!(msgs, "group $g has $(count[g]) visited members, expected $(data.groups[g].numVisits)")
    end
    return unique!(msgs)
end

groupViolations(data, sol::Union{Solution, UserSolution}) = groupViolations(data, [rt.visits for rt in sol.routes])
