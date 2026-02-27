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

################ user-defined ################

################ dont touch ################

abstract type StandardResources end

struct StandardState
    q::Float64
    stdWarp::Float64
end

struct StandardResource{ID}
    d::Matrix{Float64}
    lb::Vector{Float64}
    ub::Vector{Float64}
end

struct Resources
    customResource::CustomResource
    stdResource1::StandardResource{1}
    stdResource2::StandardResource{2}
end

abstract type Label end

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

################ dont touch ################

################ user-defined ################

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

################ user-defined ################

################ dont touch ################

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




function myInitStateForward(res::CustomResource)
    return ForwardLabel(initStateForward(res)..., StandardState(0.0, 0.0), StandardState(0.0, 0.0), 0)
end

function myInitStateBackward(res::CustomResource)
    return BackwardLabel(initStateBackward(res)..., StandardState(Inf, 0.0), StandardState(Inf, 0.0), 0)
end

function myExtendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
    return ForwardLabel(extendAlongArc(res, label, a)..., label.std1State, label.std2State, a[2] - 1)
end

function myExtendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
    return BackwardLabel(extendAlongArc(res, label, a)..., label.std1State, label.std2State, a[2] - 1)
end

function myConcatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    return ForwardLabel(concatenationCost(res, v, forwardLabel, backwardLabel)..., forwardLabel.std1State, forwardLabel.std2State, backwardLabel.last)
end

function myExtendAlongArc(res::Resources, label::ForwardLabel, a::Tuple{Int, Int})
    return ForwardLabel(extendAlongArc(res.customResource, label, a)..., extendAlongArc(res.stdResource1, label, a), extendAlongArc(res.stdResource2, label, a), a[2] - 1)
end

function myExtendAlongArc(res::Resources, label::BackwardLabel, a::Tuple{Int, Int})
    return BackwardLabel(extendAlongArc(res.customResource, label, a)..., extendAlongArc(res.stdResource1, label, a), extendAlongArc(res.stdResource2, label, a), a[2] - 1)
end

function myConcatenationCost(res::Resources, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    return ForwardLabel(concatenationCost(res.customResource, v, forwardLabel, backwardLabel)..., concatenationCost(res.stdResource1, v, forwardLabel, backwardLabel),concatenationCost(res.stdResource2, v, forwardLabel, backwardLabel),  backwardLabel.last)
end

################ dont touch ################
