"""
Add to `sp` the Set Partitioning constraints over the pooled `routes`, where binary `λ[r]` selects route `r`:

- every group is visited exactly `numVisits` times by its regular (non optional) vertices; a plain customer,
  alone in its group, exactly once. A group that needs no visit (only optional vertices) has no such constraint;
- every vertex is visited at most once (stated unless it is the only vertex of a group that needs a visit: the
  constraint above already says it);
- a group that has to stay in one route and has optional vertices is in at most one of the selected routes: its
  visits are all in the same one, but the optional ones are not in the quota that already says so;
- at most `maxNbRoutes` routes.

A route that holds part of a group that has to stay in one route (`sameRoute` with several visits), or more
regular visits of a group than it needs, cannot be part of a valid solution: its `λ` is fixed to 0. Routes of the
pool come from solutions that already keep such groups together, so this only guards against pools built by hand.

Returns `false`, without a usable model, when some group cannot reach its `numVisits` with the routes of the pool.
"""
function addPartitioningConstraints!(sp, solver::Solver, routes::Vector{Vector{Int}}, λ)
    data = solver.data
    nGroups = length(data.groups)
    vertexRoutes = [Int[] for _ in eachindex(data.groupOf)]
    groupRoutes = [Tuple{Int, Int}[] for _ in 1:nGroups]     # (route, regular visits of the group in it)
    linkRoutes = [Int[] for _ in 1:nGroups]                  # routes with a visit of a group that has optional vertices and stays in one route
    count = zeros(Int, nGroups)
    present = fill(false, nGroups)
    touched = Int[]

    for (r, route) in enumerate(routes)
        empty!(touched)
        for v in route
            v == 0 && continue
            push!(vertexRoutes[v], r)
            g = data.groupOf[v]
            present[g] || push!(touched, g)
            present[g] = true
            data.isOptional[v] || (count[g] += 1)
        end
        usable = all(touched) do g
            group = data.groups[g]
            count[g] <= group.numVisits && (!data.locked[g] || group.numVisits == 0 || count[g] == group.numVisits)
        end
        for g in touched
            if usable
                count[g] > 0 && push!(groupRoutes[g], (r, count[g]))
                data.locked[g] && data.nOptional[g] > 0 && push!(linkRoutes[g], r)
            end
            count[g] = 0
            present[g] = false
        end
        usable || fix(λ[r], 0; force = true)
    end

    for g in 1:nGroups
        if data.groups[g].numVisits > 0
            isempty(groupRoutes[g]) && return false
            @constraint(sp, sum(c * λ[r] for (r, c) in groupRoutes[g]) == data.groups[g].numVisits)
        end
        for v in data.members[g]
            (length(data.members[g]) > 1 || data.isOptional[v]) && length(vertexRoutes[v]) > 1 || continue
            @constraint(sp, sum(λ[r] for r in vertexRoutes[v]) <= 1)
        end
        length(linkRoutes[g]) > 1 && @constraint(sp, sum(λ[r] for r in linkRoutes[g]) <= 1)
    end
    @constraint(sp, sum(λ[r] for r in eachindex(routes)) <= data.maxNbRoutes)
    return true
end
