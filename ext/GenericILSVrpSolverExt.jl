"""
Adapter from a VRPSolver model (`VrpModel` of VrpSolver.jl) to the data of the heuristic. It is loaded when both
GenericILS and VrpSolver are loaded; see `heuristicModelFromVRPSolver`.
"""
module GenericILSVrpSolverExt

import GenericILS
import VrpSolver
using GenericILS: ProblemData, Vertex, Group, Resources, StandardResource, loadCppResource

unsupported(msg::AbstractString) = throw(ArgumentError("heuristicModelFromVRPSolver: " * msg))

"A value of type `T` (isbits) with all its bytes zero, for the pairs of vertices that have no arc."
zeroOf(::Type{T}) where T = sizeof(T) == 0 ? T() : reinterpret(T, zeros(UInt8, sizeof(T)))[1]

"`values` must be all `nothing` or all of the same type; returns that type or `nothing`."
function commonType(values, what::AbstractString)
    present = [v for v in values if v !== nothing]
    isempty(present) && return nothing
    length(present) == length(values) || unsupported("$what: some have parameters and some do not")
    T = typeof(present[1])
    all(v -> typeof(v) === T, present) || unsupported("$what: the parameters are not all of the same type")
    isbitstype(T) || unsupported("$what: the parameters of $T are not plain data (isbits)")
    return T
end

function GenericILS.heuristicModelFromVRPSolver(model::VrpSolver.VrpModel; cppDir::AbstractString)
    # ---- one graph, one depot ----
    length(model.graphs) == 1 || unsupported("the model has $(length(model.graphs)) graphs, only one is supported")
    G = model.graphs[1]
    G.cycle_problem || unsupported("the source and the sink must be the same vertex (routes start and end at the depot)")
    model.packing_sets_type == VrpSolver.VertexSet ||
        unsupported("the customers must be defined with set_vertex_packing_sets! (packing sets on vertices)")

    # ---- resources: the custom one and up to two standard ones ----
    custom = [r for r in G.resources if r.is_custom]
    length(custom) == 1 || unsupported("the graph needs exactly one custom resource, it has $(length(custom))")
    customRes = custom[1]
    any(r -> r.is_binary, G.resources) && unsupported("binary resources are not supported")
    standard = [r for r in G.resources if !r.is_custom && !r.is_automatic]
    length(standard) <= 2 || unsupported("at most two standard resources are supported, the graph has $(length(standard))")
    for r in standard
        r.is_disposable || unsupported("resource $(r.id) is not disposable")
    end

    # ---- vertices: the source and the sink are the depot (matrix index 1), the customers follow in the order of the graph ----
    customers = [v.id for v in G.vertices if v.id != G.source_id && v.id != G.sink_id]
    n = length(customers)
    nv = n + 1
    matrixIndex = Dict{Int,Int}(G.source_id => 1, G.sink_id => 1)
    for (k, v) in enumerate(customers)
        matrixIndex[v] = k + 1
    end
    internalOf = vcat(G.source_id, customers)               # matrix index -> vertex of the graph
    userIds = [G.vertices[v].user_id for v in internalOf]
    G.vertices[G.source_id].packing_set == -1 || unsupported("the depot cannot belong to a packing set")
    for v in customers
        G.vertices[v].packing_set >= 1 ||
            unsupported("customer $(G.vertices[v].user_id) is in no packing set: every vertex but the depot must be in one")
    end

    # ---- arcs: exactly one per ordered pair of different vertices ----
    arcOf = Dict{Tuple{Int,Int},VrpSolver.VrpArc}()
    for arc in G.arcs
        i, j = matrixIndex[arc.tail], matrixIndex[arc.head]
        i == j && unsupported("arc $(arc.id) goes from the depot to itself or from a vertex to itself")
        haskey(arcOf, (i, j)) && unsupported("parallel arcs from $(userIds[i]) to $(userIds[j])")
        arcOf[(i, j)] = arc
    end
    for i in 1:nv, j in 1:nv
        i != j && !haskey(arcOf, (i, j)) && unsupported("the graph has no arc from $(userIds[i]) to $(userIds[j]): it must be complete")
    end

    # ---- standard resources: the consumption of the arcs and the interval at the vertex that they enter ----
    function standardSlot(r)
        d = zeros(nv, nv)
        lb = fill(NaN, nv)
        ub = fill(NaN, nv)
        for ((i, j), arc) in arcOf
            d[i, j] = arc.res_consumption[r.id]
            bounds = arc.res_bounds[r.id]
            if isnan(lb[j])
                lb[j], ub[j] = bounds
            elseif (lb[j], ub[j]) != bounds
                unsupported("resource $(r.id): the arcs that enter vertex $(userIds[j]) have different bounds; only bounds on vertices are supported")
            end
        end
        return d, lb, ub
    end
    slots = Any[standardSlot(r) for r in standard]
    while length(slots) < 2                                   # an unused slot: no consumption, no limit
        push!(slots, (zeros(nv, nv), zeros(nv), fill(Inf, nv)))
    end

    # ---- objective: the cost variable of the custom resource, or the variables of the arcs ----
    terms = Dict(var => coeff for (var, coeff) in VrpSolver.get_obj_terms(model.formulation) if !iszero(coeff))
    costMatrix = zeros(nv, nv)
    if customRes.cost_var !== nothing
        (length(terms) == 1 && get(terms, customRes.cost_var, 0.0) == 1.0) ||
            unsupported("the objective must be exactly the cost variable of the custom resource (an objective that mixes it with other terms is not supported yet)")
    else
        mapped = Set(var for arc in G.arcs for (var, _) in arc.vars)
        for var in keys(terms)
            var in mapped || unsupported("the objective has a variable that no arc is mapped to")
        end
        for ((i, j), arc) in arcOf
            costMatrix[i, j] = sum((coeff * get(terms, var, 0.0) for (var, coeff) in arc.vars); init = 0.0)
        end
    end

    # ---- the custom resource: its parameters are what the model stored, with the layout of the C++ structs ----
    arcData = Dict((i, j) => get(arc.custom_data, customRes.id, nothing) for ((i, j), arc) in arcOf)
    vertexData = [get(G.vertices[internalOf[k]].custom_data, customRes.id, nothing) for k in 1:nv]
    arcType = commonType(collect(values(arcData)), "arcs")
    vertexType = commonType(vertexData, "vertices")
    arcParams = (arcType === nothing || sizeof(arcType) == 0) ? nothing :
        (i, j) -> (v = get(arcData, (i, j), nothing); v === nothing ? zeroOf(arcType) : v)
    vertexParams = (vertexType === nothing || sizeof(vertexType) == 0) ? nothing : (k -> vertexData[k])
    constParams = customRes.custom_data === nothing ? (;) : customRes.custom_data
    cpp, hpp = joinpath(cppDir, "rcsp_custom_res_impl.cpp"), joinpath(cppDir, "rcsp_custom_res_impl.hpp")
    isfile(cpp) && isfile(hpp) || unsupported("rcsp_custom_res_impl.cpp and rcsp_custom_res_impl.hpp not found in $cppDir")
    resource = loadCppResource(cpp, hpp; nv = nv, constParams = constParams, arcParams = arcParams, vertexParams = vertexParams)
    if (customRes.cost_var !== nothing) != resource.isCost
        unsupported(customRes.cost_var !== nothing ?
            "the custom resource has a cost variable but isCostResource() of the C++ is false" :
            "isCostResource() of the C++ is true but the custom resource has no cost variable")
    end

    # ---- packing sets are the groups: one visit to one vertex of each ----
    packingSets = sort!(unique(G.vertices[v].packing_set for v in customers))
    groupOf = Dict(ps => g for (g, ps) in enumerate(packingSets))
    vertices = [Vertex(k, Group(groupOf[G.vertices[customers[k]].packing_set], 1, false)) for k in 1:n]

    maxNbRoutes = G.multiplicity[2]                           # the lower bound L of the multiplicity is not used
    resources = Resources(resource, StandardResource{1}(slots[1]...), StandardResource{2}(slots[2]...))
    return (data = ProblemData(vertices, costMatrix, maxNbRoutes), res = resources, maxNbRoutes = maxNbRoutes, userIds = userIds)
end

end # module
