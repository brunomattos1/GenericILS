################ user-defined (GenericILS as a package) ################
using GenericILS
import GenericILS: isSymmetric, isCostResource, initStateForward, initStateBackward,
    extendAlongArc, concatenationCost, AbstractResource, ForwardLabel, BackwardLabel

"""
Nothing to track: the only constraint of the problem, the maximum length of a path, is a standard resource, and the
profit is part of the arc costs (see `arcCosts`), so the custom resource is a placeholder.
"""
struct NoResource <: AbstractResource end

GenericILS.isSymmetric() = false

GenericILS.isCostResource() = false

"""Empty state of the placeholder resource."""
struct NoState end

initStateForward(::NoResource) = (NoState(), 0.0)
initStateBackward(::NoResource) = (NoState(), 0.0)

extendAlongArc(::NoResource, ::ForwardLabel, ::Tuple{Int, Int}) = (NoState(), 0.0)
extendAlongArc(::NoResource, ::BackwardLabel, ::Tuple{Int, Int}) = (NoState(), 0.0)
concatenationCost(::NoResource, ::Int, ::ForwardLabel, ::BackwardLabel) = (NoState(), 0.0)
