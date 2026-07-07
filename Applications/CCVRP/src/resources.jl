struct CustomResource <: AbstractResource
    w::Vector{Float64}   # node weights
    t::Matrix{Float64}   # travel time matrix
    Wmax::Float64
    Tmax::Float64
end

function isSymmetric()
    return false
end

function isCostResource()
    return true
end

struct ForwardState
    S::Float64
    T::Float64
end

struct BackwardState
    S::Float64
    W::Float64
end

function initStateForward(res::CustomResource)
    return (ForwardState(0.0, 0.0), 0.0)
end

function initStateBackward(res::CustomResource)
    return (BackwardState(0.0, 0.0), 0.0)
end

function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
    S = label.state.S + res.w[a[2]] * (label.state.T + res.t[a...])
    T = label.state.T + res.t[a...]
    return (ForwardState(S, T), S)
end

function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
    S = label.state.S + (label.state.W + res.w[a[1]]) * res.t[a[2], a[1]]
    W = label.state.W + res.w[a[1]]
    return (BackwardState(S, W), S)
end

function concatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    return (ForwardState(0.0, 0.0), forwardLabel.state.S + backwardLabel.state.S + forwardLabel.state.T * backwardLabel.state.W)
end
