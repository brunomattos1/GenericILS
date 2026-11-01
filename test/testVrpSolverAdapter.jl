# heuristicModelFromVRPSolver: the data of the heuristic built from a VRPSolver model (VrpGraph of VrpSolver.jl). The models
# are small complete problems built here, and the result is compared with the same problem built by hand:
#   :resource  the cost lives in the custom resource (cost variable z); the capacity is a standard resource
#   :arcs      the cost lives in the variables of the arcs; the capacity is the custom resource and the time windows
#              are a standard resource
# VrpSolver.jl is not a dependency of the package, and the custom resources are compiled from C++, so runtests.jl includes
# this file only when both are available.

import VrpSolver
using JuMP: @variable, @objective

const VS = VrpSolver

# the structs of the model, with the layout of the C++ structs of test/cpp/{cvrp,distance}
struct AdapterCvrpArc
    d::Cdouble
end
struct AdapterCvrpConst
    Q::Cdouble
end
struct AdapterDistArc
    w::Cdouble
end
struct AdapterDistVertex
    c::Cdouble
end
struct AdapterEmpty end

"""
A VRPSolver model of a complete problem with `nCust` customers (see the top of the file for `kind`). `ids` are the ids of the
vertices in the model: the first one is the depot unless `depotPos` says otherwise. `sets` groups the customers (by
position among the customers) in packing sets; the default is one set for each. With `directed` (kind `:arcs`) every
arc has a variable of its own and the costs of the two directions differ. `mutate(model, G, parts)` changes the
model at the end, to build wrong ones. Returns the instance and the pieces of the model.
"""
function adapterModel(kind::Symbol; nCust = 8, Q = 25.0, seed = 5, depotPos = 1, ids = nothing, sets = nothing, directed = false, mutate = nothing)
    dist, demands, Q = randomInstance(nCust; seed = seed, Q = Q)
    nv = nCust + 1
    rng = MersenneTwister(seed)
    vertexCost = vcat(1.5, Float64.(rand(rng, 1:5, nCust)))
    # the parameters of the arcs are not symmetric, so a transposed matrix would be noticed
    (kind == :resource || directed) && (dist = dist .* (1 .+ rand(rng, nv, nv)))
    t = dist ./ 10 .* (1 .+ 0.3 .* rand(rng, nv, nv))
    lb = vcat(0.0, rand(rng, nCust) .* 15)
    ub = vcat(500.0, lb[2:end] .+ 60)
    ids === nothing && (ids = collect(0:nCust))
    userOf = ids                                           # position h (1 = depot) -> id in the model
    nodes = vcat(userOf[2:depotPos], userOf[1], userOf[depotPos+1:end])
    sets === nothing && (sets = [[c] for c in 1:nCust])
    U = ceil(Int, sum(demands) / Q) + 3

    model = VS.VrpModel()
    edges = directed ? [(p, q) for p in 1:nv for q in 1:nv if p != q] : [(a, b) for a in 1:nv for b in a+1:nv]
    @variable(model.formulation, x[e in edges], Int)
    @variable(model.formulation, z >= 0)
    if kind == :resource
        @objective(model.formulation, Min, z)
    else
        @objective(model.formulation, Min, sum(dist[e...] * x[e] for e in edges))
    end
    G = VS.VrpGraph(model, nodes, userOf[1], userOf[1], (1, U))
    if kind == :resource
        capacity = VS.add_resource!(G, main = true)
        custom = VS.add_resource!(G, custom = true, cost_var = z)
        VS.set_const_custom_res_params!(G, custom, AdapterEmpty())
        for h in 1:nv
            VS.set_resource_bounds!(G, userOf[h], capacity, 0.0, Q)
            VS.set_vertex_custom_res_params!(G, userOf[h], custom, AdapterDistVertex(vertexCost[h]))
        end
    else
        timeRes = VS.add_resource!(G, main = true)
        custom = VS.add_resource!(G, custom = true)
        capacity = timeRes
        VS.set_const_custom_res_params!(G, custom, AdapterCvrpConst(Q))
        for h in 1:nv
            VS.set_resource_bounds!(G, userOf[h], timeRes, lb[h], ub[h])
        end
    end
    for (a, b) in edges
        for (p, q) in (directed ? ((a, b),) : ((a, b), (b, a)))
            arc = VS.add_arc!(G, userOf[p], userOf[q])
            VS.add_arc_var_mapping!(G, arc, x[(a, b)])
            if kind == :resource
                VS.set_arc_consumption!(G, arc, capacity, demands[q])
                VS.set_arc_custom_res_params!(G, arc, custom, AdapterDistArc(dist[p, q]))
            else
                VS.set_arc_consumption!(G, arc, timeRes, t[p, q])
                VS.set_arc_custom_res_params!(G, arc, custom, AdapterCvrpArc(demands[q]))
            end
        end
    end
    VS.add_graph!(model, G)
    VS.set_vertex_packing_sets!(model, [[(G, userOf[1+c]) for c in set] for set in sets])
    parts = (; x, z, capacity, custom, ids = userOf)
    mutate === nothing || mutate(model, G, parts)
    inst = (; nCust, nv, Q, dist, demands, vertexCost, t, lb, ub, U, sets, userOf)
    return model, G, parts, inst
end

const CPP_DIR_ADAPTER = Dict(:resource => joinpath(@__DIR__, "cpp", "distance"), :arcs => joinpath(@__DIR__, "cpp", "cvrp"))

"The same problem built by hand with the pieces that the adapter should produce."
function handBuilt(kind::Symbol, inst)
    (; nCust, nv, Q, dist, demands, vertexCost, t, lb, ub, U, sets) = inst
    groupOf = zeros(Int, nCust)
    for (g, set) in enumerate(sets), c in set
        groupOf[c] = g
    end
    vertices = [Vertex(c, Group(groupOf[c], 1, false)) for c in 1:nCust]
    neutral = (zeros(nv, nv), zeros(nv), fill(Inf, nv))
    if kind == :resource
        custom = cppDistance(dist, vertexCost)
        std1 = (arcDemands(demands), zeros(nv), fill(Q, nv))
        costMatrix = zeros(nv, nv)
    else
        d = arcDemands(demands)
        custom = loadCppResource(cppExample("cvrp")...; nv = nv, arcParams = (i, j) -> (d = d[i, j],), constParams = (Q = Q,))
        std1 = (t, lb, ub)
        costMatrix = dist
    end
    # the model has no arc from a vertex to itself: those entries of the matrices are zero
    for m in (std1[1], costMatrix), i in 1:nv
        m[i, i] = 0.0
    end
    res = Resources(custom, StandardResource{1}(std1...), StandardResource{2}(neutral...))
    return ProblemData(vertices, costMatrix, U), res
end

function adapterSolver(data, res)
    solver = Solver(res = res, data = data, neighborhoods = NEIGHBORHOODS, algorithm = nilsAlgorithm(stop = NILS.ByIterMax(15))(res))
    setSeed!(solver, MersenneTwister(2))
    return solver
end

"The adapter's result for `kind`, compared with the problem built by hand."
function checkAdapter(kind::Symbol; kwargs...)
    model, G, parts, inst = adapterModel(kind; kwargs...)
    h = heuristicModelFromVRPSolver(model; cppDir = CPP_DIR_ADAPTER[kind])
    data, res = handBuilt(kind, inst)

    @test h.userIds == vcat(inst.userOf[1], inst.userOf[2:end])
    @test h.maxNbRoutes == inst.U == h.data.maxNbRoutes
    @test h.data.costMatrix == data.costMatrix
    @test [(v.id, v.group, v.isOptional) for v in h.data.vertices] == [(v.id, v.group, v.isOptional) for v in data.vertices]
    for (a, b) in ((h.res.stdResource1, res.stdResource1), (h.res.stdResource2, res.stdResource2))
        @test a.d == b.d && a.lb == b.lb && a.ub == b.ub
    end
    @test GenericILS.isCostResource(h.res.customResource) == (kind == :resource)
    compared, infeasible = checkSameCosts(res.customResource, h.res.customResource, MersenneTwister(4), inst.nCust)
    @test compared > 500

    # the search finds the same solution on both
    solvers = [adapterSolver(h.data, h.res), adapterSolver(data, res)]
    for s in solvers
        quiet(() -> solve!(s))
    end
    @test solvers[1].bestFeasSol.cost < Inf
    @test solvers[1].bestFeasSol.cost == solvers[2].bestFeasSol.cost
    @test [r.visits for r in solvers[1].bestFeasSol.routes] == [r.visits for r in solvers[2].bestFeasSol.routes]
    return h
end

@testset "heuristicModelFromVRPSolver" begin
    @testset "cost in the custom resource (cost variable)" begin
        h = checkAdapter(:resource)
        @test all(iszero, h.data.costMatrix)
        @test h.data.vertices[3].group == Group(3, 1, false)
    end

    @testset "cost in the variables of the arcs" begin
        h = checkAdapter(:arcs)
        @test !all(iszero, h.data.costMatrix)
    end

    @testset "cost of every arc: one variable for each directed arc" begin
        h = checkAdapter(:arcs; directed = true)
        @test h.data.costMatrix != h.data.costMatrix'
    end

    @testset "ids of the model, the depot in the middle, customers that share a packing set" begin
        h = checkAdapter(:arcs; ids = [7, 40, 30, 20, 10, 50, 60, 70, 80], depotPos = 4, sets = [[1, 2], [3], [4, 5], [6], [7, 8]])
        @test h.userIds[1] == 7                                   # the depot, which is in the middle of the nodes of the graph
        @test length(h.data.groups) == 5 && h.data.members[1] == [1, 2] && h.data.members[3] == [4, 5]
        @test all(g -> g.numVisits == 1, h.data.groups)
        checkAdapter(:resource; ids = [3, 9, 8, 7, 6, 5, 4, 2, 1], depotPos = 9)
    end

    @testset "what has no equivalent is an error" begin
        fails(kind, mutate, text; kwargs...) = begin
            model, = adapterModel(kind; mutate = mutate, kwargs...)
            err = try
                heuristicModelFromVRPSolver(model; cppDir = CPP_DIR_ADAPTER[kind])
                nothing
            catch e
                e
            end
            @test err isa ArgumentError
            err isa ArgumentError && @test occursin(text, err.msg)
        end
        resourceOf(G, f) = first(r for r in G.resources if f(r))

        fails(:arcs, (m, G, p) -> push!(m.graphs, G), "2 graphs")
        fails(:arcs, (m, G, p) -> (G.cycle_problem = false), "same vertex")
        fails(:arcs, (m, G, p) -> deleteat!(G.arcs, 5), "must be complete")
        fails(:arcs, (m, G, p) -> VS.add_arc!(G, p.ids[2], p.ids[3]), "parallel arcs")
        fails(:arcs, (m, G, p) -> VS.add_arc!(G, p.ids[2], p.ids[2]), "itself")
        fails(:arcs, (m, G, p) -> (resourceOf(G, r -> r.is_main).is_disposable = false), "not disposable")
        fails(:arcs, (m, G, p) -> (resourceOf(G, r -> r.is_main).is_binary = true), "binary")
        fails(:arcs, (m, G, p) -> (resourceOf(G, r -> r.is_custom).is_custom = false), "exactly one custom resource")
        fails(:arcs, (m, G, p) -> (VS.add_resource!(G); VS.add_resource!(G)), "at most two standard")
        fails(:arcs, (m, G, p) -> (G.arcs[3].res_bounds[p.capacity] = (1.0, 2.0)), "different bounds")
        fails(:arcs, (m, G, p) -> (G.vertices[3].packing_set = -1), "no packing set")
        fails(:arcs, (m, G, p) -> (G.vertices[1].packing_set = 1), "depot cannot belong")
        fails(:arcs, (m, G, p) -> (m.packing_sets_type = VS.ArcSet), "packing sets on vertices")
        fails(:resource, (m, G, p) -> delete!(G.vertices[3].custom_data, p.custom), "some have parameters")
        fails(:resource, (m, G, p) -> @objective(m.formulation, Min, 2.0 * p.z), "exactly the cost variable")
        fails(:resource, (m, G, p) -> @objective(m.formulation, Min, p.z + p.x[(1, 2)]), "exactly the cost variable")
        # the cost variable and isCostResource() of the C++ must agree
        fails(:arcs, (m, G, p) -> (resourceOf(G, r -> r.is_custom).cost_var = p.z; @objective(m.formulation, Min, p.z)),
            "isCostResource() of the C++ is false")
        fails(:resource, (m, G, p) -> (resourceOf(G, r -> r.is_custom).cost_var = nothing;
                @objective(m.formulation, Min, sum(p.x))), "has no cost variable")
        # a variable of the objective that no arc is mapped to
        fails(:arcs, (m, G, p) -> @objective(m.formulation, Min, p.z), "no arc is mapped to")

        model, = adapterModel(:arcs)
        @test_throws ArgumentError heuristicModelFromVRPSolver(model; cppDir = mktempdir())      # no C++ files there
        @test_throws ArgumentError heuristicModelFromVRPSolver(42; cppDir = ".")                  # not a VrpModel
    end
end
