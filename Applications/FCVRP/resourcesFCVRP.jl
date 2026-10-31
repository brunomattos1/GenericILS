################ user-defined (GenericILS as a package) ################
using GenericILS
import GenericILS: isSymmetric, isCostResource, initStateForward, initStateBackward,
    extendAlongArc, concatenationCost, AbstractResource, ForwardLabel, BackwardLabel

"""Capacity: `d[i, j]` is the demand picked up when entering `j`, `Q` the vehicle capacity."""
struct CustomResource <: AbstractResource
    d::Matrix{Float64}
    Q::Float64
end

GenericILS.isSymmetric() = false

GenericILS.isCostResource() = false

"""Accumulated demand `q` of the prefix."""
struct ForwardState
    q::Float64
end

"""Accumulated demand `q` of the suffix."""
struct BackwardState
    q::Float64
end

"""Forward state of the empty prefix."""
function initStateForward(res::CustomResource)
    return (ForwardState(0.0), 0.0)
end

"""Backward state of the empty suffix."""
function initStateBackward(res::CustomResource)
    return (BackwardState(0.0), 0.0)
end

"""Forward extension along `a = (i, j)` (matrix indices): appends `j`. The returned cost is `0.0`, or `Inf` when a rule is violated."""
function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
    qCustom = label.state.q + res.d[(1, a[2])...]
    if qCustom > res.Q + 1e-5
        return (ForwardState(qCustom), Inf)
    else
        return (ForwardState(qCustom), 0.0)
    end
end

"""Backward extension along `a = (next, cur)`: prepends `cur` to the suffix that starts at `next`. The returned cost is `0.0`, or `Inf` when a rule is violated."""
function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
    qCustom = label.state.q + res.d[(1, a[1])...]
    if qCustom > res.Q + 1e-5
        return (BackwardState(qCustom), Inf)
    else
        return (BackwardState(qCustom), 0.0)
    end
end

"""Join the forward and the backward label at the same vertex: cost/feasibility of the whole route."""
function concatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    if forwardLabel.state.q + backwardLabel.state.q > res.Q + 1e-5
        return (ForwardState(forwardLabel.state.q + backwardLabel.state.q), Inf)
    else
        return (ForwardState(forwardLabel.state.q + backwardLabel.state.q), 0.0)
    end
end
