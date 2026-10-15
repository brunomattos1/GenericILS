################ user-defined (GenericILS as a package) ################
using GenericILS
import GenericILS: isSymmetric, isCostResource, initStateForward, initStateBackward,
    extendAlongArc, concatenationCost, AbstractResource, ForwardLabel, BackwardLabel

"""Simultaneous pickup and delivery: pickup `p` and delivery `d` demands of the arc head and capacity `Q`."""
struct CustomResource <: AbstractResource
    p::Matrix{Float64} # pickup demand of the arc head
    d::Matrix{Float64} # delivery demand of the arc head
    Q::Float64         # vehicle capacity
end

GenericILS.isSymmetric() = false

GenericILS.isCostResource() = false

"""Accumulated pickup `P` and remaining delivery capacity `D` of the prefix."""
struct ForwardState
    P::Float64 # accumulated pickup demand
    D::Float64 # remaining delivery capacity
end

"""Accumulated pickup `P` and remaining delivery capacity `D` of the suffix."""
struct BackwardState
    P::Float64
    D::Float64
end

"""Forward state of the empty prefix."""
function initStateForward(res::CustomResource)
    return (ForwardState(0.0, res.Q), 0.0)
end

"""Backward state of the empty suffix."""
function initStateBackward(res::CustomResource)
    return (BackwardState(res.Q, 0.0), 0.0)
end

"""Forward extension along `a = (i, j)` (matrix indices): appends `j`. The returned cost is `0.0`, or `Inf` when a rule is violated."""
function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
    P′ = label.state.P + res.p[a[1], a[2]]
    D′ = min(label.state.D - res.d[a[1], a[2]], res.Q - P′)
    if P′ > res.Q + 1e-5 || res.d[a[1], a[2]] > label.state.D + 1e-5
        return (ForwardState(P′, D′), Inf)
    else
        return (ForwardState(P′, D′), 0.0)
    end
end

"""Backward extension along `a = (next, cur)`: prepends `cur` to the suffix that starts at `next`. The returned cost is `0.0`, or `Inf` when a rule is violated."""
function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
    D′ = label.state.D + res.d[a[1], a[2]]
    P′ = min(label.state.P - res.p[a[1], a[2]], res.Q - D′)
    if D′ > res.Q + 1e-5 || res.p[a[1], a[2]] > label.state.P + 1e-5
        return (BackwardState(P′, D′), Inf)
    else
        return (BackwardState(P′, D′), 0.0)
    end
end

"""Join the forward and the backward label at the same vertex: cost/feasibility of the whole route."""
function concatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    p_ = 0.0
    d_ = 0.0
    if forwardLabel.last > 0
        p_ = res.p[1, forwardLabel.last + 1]
    end
    if backwardLabel.last > 0
        d_ = res.d[1, backwardLabel.last + 1]
    end
    if forwardLabel.state.P - p_ > backwardLabel.state.P + 1e-5 || backwardLabel.state.D - d_ > forwardLabel.state.D + 1e-5
        return (ForwardState(0.0, 0.0), Inf)
    else
        return (ForwardState(0.0, 0.0), 0.0)
    end
end
