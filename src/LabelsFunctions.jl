abstract type AbstractResource end
abstract type AbstractResources end
abstract type Label end

struct StandardResource{ID}
    d::Matrix{Float64}
    lb::Vector{Float64}
    ub::Vector{Float64}
end

struct StandardState
    q::Float64
    stdWarp::Float64
end

struct ForwardLabel{S}
    state::S
    cost::Float64
    std1State::StandardState
    std2State::StandardState
    last::Int
end

struct BackwardLabel{S}
    state::S
    cost::Float64
    std1State::StandardState
    std2State::StandardState
    last::Int
end

initStateForward(res::AbstractResource) = error("not implemented")
initStateBackward(res::AbstractResource) = error("not implemented")

isSymmetric() = error("not implemented")
isCostResource() = error("not implemented")

@inline function get_state(label::ForwardLabel, ::Val{1})
    return label.std1State
end

@inline function get_state(label::BackwardLabel, ::Val{1})
    return label.std1State
end

@inline function get_state(label::ForwardLabel, ::Val{2})
    return label.std2State
end

@inline function get_state(label::BackwardLabel, ::Val{2})
    return label.std2State
end

function extendAlongArc(res::StandardResource{ID}, label::ForwardLabel, a::Tuple{Int, Int}) where ID
    state = get_state(label, Val(ID))

    q_new = max(min(state.q + res.d[a...], res.ub[a[2]]), res.lb[a[2]])
    warp_new = state.stdWarp + max(state.q + res.d[a...] - res.ub[a[2]], 0.0)
    return (StandardState(q_new, warp_new))
end

function extendAlongArc(res::StandardResource{ID}, label::BackwardLabel, a::Tuple{Int, Int}) where ID
    a = (a[2], a[1])
    state = get_state(label, Val(ID))

    q_new = min(state.q - res.d[a...], res.ub[a[1]])
    warp_new = state.stdWarp
    if q_new < res.lb[a[1]]
        warp_new += res.lb[a[1]] - q_new
        q_new = res.lb[a[1]]
    end
    return (StandardState(q_new, warp_new))
end

function concatenationCost(res::StandardResource{ID}, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel) where ID
    fwState = get_state(forwardLabel, Val(ID))
    bwState = get_state(backwardLabel, Val(ID))
    q_std = min(fwState.q, bwState.q)
    q_warp = max(fwState.q - bwState.q, 0) + (fwState.stdWarp + bwState.stdWarp)
    return StandardState(q_std, q_warp)
end

function myInitStateForward(res::AbstractResource)
    state, cost = initStateForward(res)
    return ForwardLabel(state, cost, StandardState(0.0, 0.0), StandardState(0.0, 0.0), 0)
end

function myInitStateBackward(res::AbstractResource)
    state, cost = initStateBackward(res)
    return BackwardLabel(state, cost, StandardState(Inf, 0.0), StandardState(Inf, 0.0), 0)
end

function myExtendAlongArc(res::AbstractResource, label::ForwardLabel, a::Tuple{Int, Int})
    state, cost = extendAlongArc(res, label, a)
    return ForwardLabel(state, cost, label.std1State, label.std2State, a[2] - 1)
end

function myExtendAlongArc(res::AbstractResource, label::BackwardLabel, a::Tuple{Int, Int})
    state, cost = extendAlongArc(res, label, a)
    return BackwardLabel(state, cost, label.std1State, label.std2State, a[2] - 1)
end

function myConcatenationCost(res::AbstractResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    state, cost = concatenationCost(res, v, forwardLabel, backwardLabel)
    return ForwardLabel(state, cost, forwardLabel.std1State, forwardLabel.std2State, backwardLabel.last)
end

function myExtendAlongArc(res::AbstractResources, label::ForwardLabel, a::Tuple{Int, Int})
    state, cost = extendAlongArc(res.customResource, label, a)
    std1 = extendAlongArc(res.stdResource1, label, a)
    std2 = extendAlongArc(res.stdResource2, label, a)
    return ForwardLabel(state, cost, std1, std2, a[2] - 1)
end

function myExtendAlongArc(res::AbstractResources, label::BackwardLabel, a::Tuple{Int, Int})
    state, cost = extendAlongArc(res.customResource, label, a)
    std1 = extendAlongArc(res.stdResource1, label, a)
    std2 = extendAlongArc(res.stdResource2, label, a)
    return BackwardLabel(state, cost, std1, std2, a[2] - 1)
end

function myConcatenationCost(res::AbstractResources, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    state, cost = concatenationCost(res.customResource, v, forwardLabel, backwardLabel)
    std1 = concatenationCost(res.stdResource1, v, forwardLabel, backwardLabel)
    std2 = concatenationCost(res.stdResource2, v, forwardLabel, backwardLabel)
    return ForwardLabel(state, cost, std1, std2, backwardLabel.last)
end
