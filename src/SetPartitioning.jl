"""
Add to `sp` the Set Partitioning constraints over the pooled `routes`, where binary `λ[r]` selects route `r`:

- every group is visited exactly `numVisits` times (a plain customer, alone in its group, exactly once);
- every vertex is visited at most once (only stated for groups with several vertices: for a single one the
  constraint above already says it);
- at most `maxNbRoutes` routes.

A route that holds part of a group that has to stay in one route (`sameRoute` with `numVisits > 1`), or more
visits of a group than it needs, cannot be part of a valid solution: its `λ` is fixed to 0. Routes of the pool
come from solutions that already keep such groups together, so this only guards against pools built by hand.

Returns `false`, without a usable model, when some group cannot reach its `numVisits` with the routes of the pool.
"""
function addPartitioningConstraints!(sp, solver::Solver, routes::Vector{Vector{Int}}, λ)
    data = solver.data
    nGroups = length(data.groups)
    vertexRoutes = [Int[] for _ in eachindex(data.groupOf)]
    groupRoutes = [Tuple{Int, Int}[] for _ in 1:nGroups]     # (route, visits of the group in it)
    count = zeros(Int, nGroups)
    touched = Int[]

    for (r, route) in enumerate(routes)
        empty!(touched)
        for v in route
            v == 0 && continue
            push!(vertexRoutes[v], r)
            g = data.groupOf[v]
            count[g] == 0 && push!(touched, g)
            count[g] += 1
        end
        usable = all(touched) do g
            group = data.groups[g]
            count[g] <= group.numVisits && (!mustStayInRoute(group) || count[g] == group.numVisits)
        end
        for g in touched
            usable && push!(groupRoutes[g], (r, count[g]))
            count[g] = 0
        end
        usable || fix(λ[r], 0; force = true)
    end

    for g in 1:nGroups
        isempty(groupRoutes[g]) && return false
        @constraint(sp, sum(c * λ[r] for (r, c) in groupRoutes[g]) == data.groups[g].numVisits)
        length(data.members[g]) > 1 || continue
        for v in data.members[g]
            length(vertexRoutes[v]) > 1 && @constraint(sp, sum(λ[r] for r in vertexRoutes[v]) <= 1)
        end
    end
    @constraint(sp, sum(λ[r] for r in eachindex(routes)) <= data.maxNbRoutes)
    return true
end
