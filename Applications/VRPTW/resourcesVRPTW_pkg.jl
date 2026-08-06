################ user-defined (GenericILS as a package) ################
using GenericILS
import GenericILS: isSymmetric, isCostResource, initStateForward, initStateBackward,
    extendAlongArc, concatenationCost, AbstractResource, ForwardLabel, BackwardLabel

struct CustomResource <: AbstractResource
    d::Matrix{Float64}
    Q::Float64
end

GenericILS.isSymmetric() = false

GenericILS.isCostResource() = false

struct ForwardState
    q::Float64
end

struct BackwardState
    q::Float64
end

function initStateForward(res::CustomResource)
    return (ForwardState(0.0), 0.0)
end

function initStateBackward(res::CustomResource)
    return (BackwardState(0.0), 0.0)
end

function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
    q_custom = label.state.q + res.d[(1, a[2])...]
    if q_custom > res.Q + 1e-5
        return (ForwardState(q_custom), Inf)
    else
        return (ForwardState(q_custom), 0.0)
    end
end

function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
    q_custom = label.state.q + res.d[(1, a[1])...]
    if q_custom > res.Q + 1e-5
        return (BackwardState(q_custom), Inf)
    else
        return (BackwardState(q_custom), 0.0)
    end
end

function concatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    if forwardLabel.state.q + backwardLabel.state.q > res.Q + 1e-5
        return (ForwardState(forwardLabel.state.q + backwardLabel.state.q), Inf)
    else
        return (ForwardState(forwardLabel.state.q + backwardLabel.state.q), 0.0)
    end
end
