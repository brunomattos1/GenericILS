################ user-defined ################
struct CustomResource
    d::Matrix{Float64}
    Q::Float64
end

function isSymmetric()
    return false
end

function isCostResource()
    return false
end

struct ForwardState
    q::Float64
end

struct BackwardState
    q::Float64
end

struct StandardResource{ID}
    d::Matrix{Float64}
    lb::Vector{Float64}
    ub::Vector{Float64}
end

struct StandardState
    q::Float64
    stdWarp::Float64
end

mutable struct Resources
    customResource::Union{CustomResource, Nothing}
    stdResource1::Union{StandardResource{1}, Nothing}
    stdResource2::Union{StandardResource{2}, Nothing}
end

struct ForwardLabel
    state::ForwardState
    cost::Float64
    std1State::StandardState
    std2State::StandardState
    last::Int
end

struct BackwardLabel
    state::BackwardState
    cost::Float64
    std1State::StandardState
    std2State::StandardState
    last::Int
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
        newState = (ForwardState(forwardLabel.state.q + backwardLabel.state.q), Inf)
        return newState
    else
        newState = (ForwardState(forwardLabel.state.q + backwardLabel.state.q), 0.0)
        return newState
    end
end