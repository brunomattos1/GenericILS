################ user-defined (GenericILS as a package) ################
using GenericILS
import GenericILS: isSymmetric, isCostResource, initStateForward, initStateBackward,
    extendAlongArc, concatenationCost, AbstractResource, ForwardLabel, BackwardLabel

"""Cumulative-cost VRP: node weights `w` and travel times `t`; the route cost is the weighted sum of arrival times."""
struct CustomResource <: AbstractResource
    w::Vector{Float64}   # node weights
    t::Matrix{Float64}   # travel time matrix
    Wmax::Float64
    Tmax::Float64
end

GenericILS.isSymmetric() = false

GenericILS.isCostResource() = true

"""Accumulated cost `S` and elapsed time `T` of the prefix."""
struct ForwardState
    S::Float64
    T::Float64
end

"""Accumulated cost `S` and total weight `W` of the suffix."""
struct BackwardState
    S::Float64
    W::Float64
end

"""Forward state of the empty prefix."""
function initStateForward(res::CustomResource)
    return (ForwardState(0.0, 0.0), 0.0)
end

"""Backward state of the empty suffix."""
function initStateBackward(res::CustomResource)
    return (BackwardState(0.0, 0.0), 0.0)
end

"""Forward extension along `a = (i, j)` (matrix indices): appends `j`. The returned cost is the accumulated cost of the partial route."""
function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
    S = label.state.S + res.w[a[2]] * (label.state.T + res.t[a...])
    T = label.state.T + res.t[a...]
    return (ForwardState(S, T), S)
end

"""Backward extension along `a = (next, cur)`: prepends `cur` to the suffix that starts at `next`. The returned cost is the accumulated cost of the partial route."""
function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
    S = label.state.S + (label.state.W + res.w[a[1]]) * res.t[a[2], a[1]]
    W = label.state.W + res.w[a[1]]
    return (BackwardState(S, W), S)
end

"""Join the forward and the backward label at the same vertex: cost/feasibility of the whole route."""
function concatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    return (ForwardState(0.0, 0.0), forwardLabel.state.S + backwardLabel.state.S + forwardLabel.state.T * backwardLabel.state.W)
end
