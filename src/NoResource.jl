"""
Placeholder custom resource for a problem whose constraints are all standard resources and whose cost is in the cost
matrix: it has no state and adds nothing. Not exported (applications define their own `NoResource` of the same kind);
`heuristicModelFromVRPSolver` uses it when the model has no custom resource.
"""
struct NoResource <: AbstractResource end

"""Empty state of [`NoResource`](@ref)."""
struct NoState end

isSymmetric(::NoResource) = false
isCostResource(::NoResource) = false

initStateForward(::NoResource) = (NoState(), 0.0)
initStateBackward(::NoResource) = (NoState(), 0.0)

extendAlongArc(::NoResource, ::ForwardLabel, ::Tuple{Int, Int}) = (NoState(), 0.0)
extendAlongArc(::NoResource, ::BackwardLabel, ::Tuple{Int, Int}) = (NoState(), 0.0)
extendToVertex(::NoResource, label::ForwardLabel, ::Int) = (label.state, label.cost)
extendToVertex(::NoResource, label::BackwardLabel, ::Int) = (label.state, label.cost)
concatenationCost(::NoResource, ::Int, ::ForwardLabel, ::BackwardLabel) = (NoState(), 0.0)
