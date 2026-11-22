using GenericILS
using Random
using VrpSolver
using JuMP: @variable, @objective, @constraint
include("dataGVRP.jl")

# The VRPSolver model of the GVRP (Bektas et al. 2011), to test how the packing sets of a VRPSolver model become the groups
# of the heuristic. The model is only built and read by heuristicModelFromVRPSolver, never optimized, so BaPCod is not
# needed. The capacity is a standard resource and the only constraint of a route, so the model has no custom resource
# (no C++ files, and the adapter uses the placeholder resource); the cost is in the variables of the edges.

############################ the VRPSolver model ############################

"""
Build the VRPSolver model of `gvrp` (the result of `readGVRP`) with the distances `dist` (depot first): the vertices are
`0:n` (0 is the depot), one integer variable `x` per edge, and every cluster is a packing set, so a route visits at most
one vertex of each, and a master constraint that makes exactly one visit of each cluster (degree 2 in the vertices of the
cluster). The cuts of the BCP (capacity, elementarity sets) are not needed to read the model and are left out.
"""
function buildVrpModel(gvrp, dist::Matrix{Float64})
    n  = length(gvrp.demand) - 1
    V  = 0:n                                                        # vertices of the graph, the depot is 0
    Q  = gvrp.capacity
    demand(i) = gvrp.demand[i+1]
    E  = [(i, j) for i in V for j in V if i < j]                    # an edge for every pair of vertices
    δ(i) = [(min(i, j), max(i, j)) for j in V if j != i]            # edges incident to i
    clusters = gvrp.clusters
    sort(reduce(vcat, clusters)) == collect(1:n) || error("the clusters must partition the customers")

    gvrpModel = VrpModel()
    @variable(gvrpModel.formulation, x[e in E], Int)
    @objective(gvrpModel.formulation, Min, sum(dist[i+1, j+1] * x[(i, j)] for (i, j) in E))
    @constraint(gvrpModel.formulation, visit[c in eachindex(clusters)], sum(x[e] for i in clusters[c] for e in δ(i)) == 2.0)

    G = VrpGraph(gvrpModel, collect(V), 0, 0, (1, gvrp.vehicles))   # source = sink = depot
    capRes = add_resource!(G, main = true)
    for i in V
        set_resource_bounds!(G, i, capRes, 0.0, Q)                  # accumulated load in [0, Q]
    end

    # two arcs for each edge; the load of an arc is the mean of the demands of its ends (a route consumes the sum of the
    # demands of its customers, the depot has none)
    for (i, j) in E
        for (a, b) in ((i, j), (j, i))
            arc = add_arc!(G, a, b)
            add_arc_var_mapping!(G, arc, x[(i, j)])
            set_arc_consumption!(G, arc, capRes, (demand(a) + demand(b)) / 2)
        end
    end
    add_graph!(gvrpModel, G)

    # each cluster is a packing set: a route visits at most one of its vertices
    set_vertex_packing_sets!(gvrpModel, [[(G, i) for i in cluster] for cluster in clusters])
    return gvrpModel
end

############################ packing sets and groups ############################

"""
Check that the packing sets of the model are the groups of the heuristic: the group `c` is the cluster `c` (members,
one visit), every customer is in the group of its cluster, and the vertices are those of the application
(`buildVertices`). Returns the number of groups.
"""
function checkGroups(h, gvrp)
    data, clusters = h.data, gvrp.clusters
    length(data.groups) == length(clusters) || error("$(length(data.groups)) groups for $(length(clusters)) clusters")
    for (c, cluster) in enumerate(clusters)
        Set(data.members[c]) == Set(cluster) || error("group $c has the members $(data.members[c]), the cluster is $cluster")
        data.groups[c] == Group(c, 1, false) || error("group $c is $(data.groups[c]): one visit, no optional vertices, no same route")
        all(data.groupOf[v] == c for v in cluster) || error("a vertex of the cluster $c is in another group")
    end
    # the vertices of the heuristic are those that the application builds from the clusters (the order of the ids is the same)
    reference = buildVertices(clusters, 1)
    [(v.id, v.group) for v in data.vertices] == [(v.id, v.group) for v in reference] ||
        error("the vertices of the adapter are not the ones of buildVertices")
    h.userIds == collect(0:length(reference)) || error("the ids of the model are not 0:n")
    return length(data.groups)
end

"""Check the rest of the data against the instance: costs, capacity of the standard resource and number of routes."""
function checkData(h, gvrp, dist)
    n = length(gvrp.demand) - 1
    off = [(i, j) for i in 1:n+1 for j in 1:n+1 if i != j]
    all(h.data.costMatrix[i, j] == dist[i, j] for (i, j) in off) || error("the cost matrix is not the distance matrix")
    h.maxNbRoutes == gvrp.vehicles == h.data.maxNbRoutes || error("number of routes $(h.maxNbRoutes), expected $(gvrp.vehicles)")
    h.res.customResource isa GenericILS.NoResource || error("a model without custom resource should get the placeholder")
    std = h.res.stdResource1
    all(std.ub .== gvrp.capacity) && all(std.lb .== 0.0) || error("the capacity should be the bound of the standard resource")
    # a route consumes the demand of its customers (the depot has none): d[i, j] + d[j, i] = demand(i) + demand(j) over each edge
    all(std.d[i, j] + std.d[j, i] ≈ gvrp.demand[i] + gvrp.demand[j] for (i, j) in off) || error("arc consumption is not the demand")
end

############################ solution ############################

"""Check `sol` against the instance, independently of the labels: capacity, one visit per cluster, routes, and cost."""
function checkSolution(gvrp, dist, sol::Solution)
    routes = [r.visits for r in sol.routes if length(r.visits) > 2]
    length(routes) <= gvrp.vehicles || error("$(length(routes)) routes, at most $(gvrp.vehicles)")
    visited = Int[]
    cost = 0.0
    for visits in routes
        visits[1] == 0 && visits[end] == 0 || error("a route does not start and end at the depot")
        sum(gvrp.demand[v+1] for v in visits) <= gvrp.capacity + 1e-6 || error("a route exceeds the capacity")
        cost += sum(dist[visits[k]+1, visits[k+1]+1] for k in 1:length(visits)-1)
        append!(visited, visits[2:end-1])
    end
    clusterOf = Dict(v => c for (c, cluster) in enumerate(gvrp.clusters) for v in cluster)
    sort([clusterOf[v] for v in visited]) == collect(1:length(gvrp.clusters)) || error("each cluster must be visited exactly once")
    isapprox(cost, sol.dist; rtol = 1e-9) || error("cost $(sol.dist) of the solution is not $cost")
    return cost
end

"""Build the model of `instance`, check how it was adapted and solve it with NILS; returns the cost."""
function main(instance::String, innerIterMax::Int, seed::Int)
    gvrp  = readGVRP(joinpath(@__DIR__, "data", instance))
    dist  = euclideanMatrix(gvrp.coords)
    model = buildVrpModel(gvrp, dist)
    h     = heuristicModelFromVRPSolver(model)                     # no custom resource: no C++ files
    println("$(length(h.data.vertices)) customers in $(checkGroups(h, gvrp)) groups (packing sets), $(h.maxNbRoutes) routes")
    checkData(h, gvrp, dist)

    algorithm = NILSAlgorithm(h.res;
        acceptCriteria = NILS.AcceptBest(),
        stopCriteria = NILS.ByIterMax(100),
        diversification = NILS.Diversification(outerShift = 3, outerSwap = 3, innerShift = 1, innerSwap = 1),
        innerIterMax = innerIterMax
    )
    solver = Solver(
        res = h.res,
        data = h.data,
        neighborhoods = (
            TwoOptStar(),
            IntraShift(),
            InterShift{1}(),
            InterShift{2}(),
            InterSwap{1, 1}(),
            InterSwap{2, 1}(),
            InterSwap{2, 2}(),
            IntraGroupExchange()
        ),
        algorithm = algorithm
    )
    setSeed!(solver, Random.MersenneTwister(seed))
    setPenaltyManager!(solver, StandardPenaltyManager(
        penaltyCustom = 0.0, penaltyCustomIncrease = 0.0, penaltyCustomDecrease = 0.0,
        penaltyStandard1 = 1.0, penaltyStandard1Increase = 0.01, penaltyStandard1Decrease = 0.01,
        penaltyStandard2 = 0.0, penaltyStandard2Increase = 0.0, penaltyStandard2Decrease = 0.0
    ))

    t = @elapsed solve!(solver)
    sol = getBestSol(solver)
    isempty(groupViolations(h.data, sol)) || error("group violations: $(groupViolations(h.data, sol))")
    cost = checkSolution(gvrp, dist, sol)
    println("Cost: $cost ($(round(t, digits = 1)) s)")
    return cost
end

if abspath(PROGRAM_FILE) == @__FILE__
    main("M-n101-k10-C34-V4.vrp", 5, 1)
end
