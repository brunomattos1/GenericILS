"""
    heuristicModelFromVRPSolver(model; cppDir)

Build the data of the heuristic from a VRPSolver model (`VrpModel` of VrpSolver.jl). It needs `VrpSolver` to be loaded
(`using VrpSolver` next to `using GenericILS`), and returns a `NamedTuple`:

- `data`: the `ProblemData`: customers, the groups (each packing set is a group that needs one visit), the cost matrix
  (zero when the cost is the cost variable of the custom resource, otherwise the cost of the variables mapped to each
  arc) and the maximum number of routes;
- `res`: the `Resources`: the custom resource loaded from the C++ files with the parameters stored in the model (see
  [`loadCppResource`](@ref)), and the standard resources of the graph (up to two; the arc consumption and the interval
  at the vertex that the arc enters);
- `maxNbRoutes`: the upper bound `U` of the multiplicity of the graph;
- `userIds`: `userIds[i + 1]` is the id in the model of the vertex `i` of the heuristic (`userIds[1]` is the depot).

`cppDir` is the folder with `rcsp_custom_res_impl.hpp` and `rcsp_custom_res_impl.cpp` of the model, which the model does
not store. The depot is vertex 0 and the customers are `1:n` in the order of the graph.

What has no equivalent in the heuristic throws an `ArgumentError`: more than one graph, a source different from the
sink, a graph that is not complete or has parallel arcs, binary or non-disposable resources, more than two standard
resources or other than one custom resource, bounds that depend on the arc, a vertex outside the packing sets, and an
objective that is neither the cost variable of the custom resource nor made of variables mapped to arcs. The lower
bound `L` of the number of routes is not used.
"""
function heuristicModelFromVRPSolver end

heuristicModelFromVRPSolver(model; kwargs...) =
    throw(ArgumentError("heuristicModelFromVRPSolver needs a VrpModel of VrpSolver.jl and `using VrpSolver`"))
