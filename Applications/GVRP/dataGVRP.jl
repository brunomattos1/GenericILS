"""
Partition customers `1:n` into clusters of (at most) `clusterSize` vertices, greedily: each cluster starts at
the unassigned customer farthest from the depot and takes its nearest unassigned neighbors. `dist` has the
depot at index 1. Returns the vertex ids of each cluster.
"""
function buildClusters(dist::Matrix{Float64}, clusterSize::Int)
    free = collect(1:size(dist, 1) - 1)
    clusters = Vector{Vector{Int}}()
    while !isempty(free)
        seed = free[argmax(dist[1, free .+ 1])]
        byDistance = sortperm(dist[seed + 1, free .+ 1]; alg = MergeSort)
        members = free[byDistance[1:min(clusterSize, length(free))]]
        push!(clusters, members)
        filter!(v -> !(v in members), free)
    end
    return clusters
end

"""
One `Vertex` per customer, in the group of its cluster: the solution visits `numVisits` vertices of each
cluster (at most the cluster size), each visited vertex once. `numVisits = 1` is the generalized VRP.
"""
function buildVertices(clusters::Vector{Vector{Int}}, numVisits::Int)
    vertices = Vertex[]
    for (g, members) in enumerate(clusters)
        group = Group(g, min(numVisits, length(members)), false)
        append!(vertices, Vertex(id, group) for id in members)
    end
    return sort!(vertices; by = v -> v.id)
end

"""
Demand of each vertex (depot first, demand 0): whichever vertices of a cluster are visited, the visits carry
the demand of the whole cluster, i.e. the sum of the original demands of its customers split among the visits.
"""
function clusterDemands(demand::Vector{Float64}, clusters::Vector{Vector{Int}}, vertices::Vector{Vertex})
    q = zeros(Float64, length(demand))
    for (g, members) in enumerate(clusters)
        visits = vertices[members[1]].group.numVisits
        for id in members
            q[id + 1] = sum(demand[m + 1] for m in members) / visits
        end
    end
    return q
end
