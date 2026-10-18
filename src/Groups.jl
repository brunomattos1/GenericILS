"""
A group of vertices. Exactly `numVisits` of its members must be visited by the solution (each visited
member once); when `sameRoute` is `true` all the visited members must be in the same route.

A plain customer is a group of one vertex with `numVisits = 1`.
"""
struct Group
    groupId::Int
    numVisits::Int
    sameRoute::Bool
end

"""
Whether the neighborhoods and the perturbation of the package can break the group constraints of `groups`.
They are only group-aware for groups whose constraints they preserve by construction: a group that
needs several visits in the same route cannot be split by moving a single customer.
"""
supportsGroups(groups::AbstractVector{Group}) = !any(g -> g.sameRoute && g.numVisits > 1, groups)

"""Throw if the group constraints of `data` are not supported by the search yet (see `supportsGroups`)."""
function assertGroupsSupported(data)
    supportsGroups(data.groups) || error(
        "The local search does not support groups that require several visits in the same route yet " *
        "(`sameRoute = true` with `numVisits > 1`). Construction works; use `groupViolations` to validate solutions.")
    return nothing
end

"""
Group constraints broken by `routes` (vectors of visits, depot id `0` at both ends), as human-readable
messages; empty when the routes satisfy them. A solution satisfies them when every vertex is visited at most
once, every group has exactly `numVisits` visited members and the visited members of a `sameRoute`
group are in a single route.
"""
function groupViolations(data, routes::AbstractVector{<:AbstractVector{Int}})
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
            count[g] += 1
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
