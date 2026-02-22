const DEBUG_MODE = false
################ user-defined ################
struct CustomResource
    w::Vector{Float64}
    t::Matrix{Float64}
    Wmax::Float64
    Tmax::Float64
end

function isSymmetric()
    return false
end

function isCostResource()
    return true
end

struct CustomState
    q::Float64
end

struct ForwardState
    S::Float64
    T::Float64
end

struct BackwardState
    S::Float64
    W::Float64
end

################ user-defined ################

################ dont touch ################

struct StandardState
    q::Float64
    stdWarp::Float64
end

struct StandardResource
    d::Matrix{Float64}
    lb::Vector{Float64}
    ub::Vector{Float64}
end

struct Resources
    customResource::CustomResource
    stdResource::StandardResource
end

struct ForwardLabel
    #custom_res::CustomState
    state::ForwardState
    cost::Float64
    std_res::StandardState
    last::Int
end

struct BackwardLabel
    #custom_res::CustomState
    state::BackwardState
    cost::Float64
    std_res::StandardState
    last::Int
end

################ dont touch ################

################ user-defined ################

function initStateForward(res::CustomResource)
    return (ForwardState(0.0, 0.0), 0.0)
end

function initStateBackward(res::CustomResource)
    return (BackwardState(0.0, 0.0), 0.0)
end

function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
    S = label.state.S + res.w[a[2]]*(label.state.T + res.t[a...])
    T = label.state.T + res.t[a...]
    return (ForwardState(S, T), S)
end

function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
    S = label.state.S + (label.state.W + res.w[a[1]])*res.t[a[2], a[1]]
    W = label.state.W + res.w[a[1]]
    return (BackwardState(S, W), S)
end

function concatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    return (ForwardState(0.0, 0.0), forwardLabel.state.S + backwardLabel.state.S + forwardLabel.state.T * backwardLabel.state.W)
end

################ user-defined ################

################ dont touch ################

# label: 2 -> 0, a = (2, 3)

function extendAlongArc(res::StandardResource, label::ForwardLabel, a::Tuple{Int, Int})
    q_new = max(min(label.std_res.q + res.d[a...], res.ub[a[2]]), res.lb[a[2]])
    warp_new = label.std_res.stdWarp + max(label.std_res.q + res.d[a...] - res.ub[a[2]], 0.0)
    return (StandardState(q_new, warp_new))
end

function extendAlongArc(res::StandardResource, label::BackwardLabel, a::Tuple{Int, Int})
    a = (a[2], a[1])

    q_new = min(label.std_res.q - res.d[a...], res.ub[a[1]])
    warp_new = label.std_res.stdWarp
    # warp_new += max(res.lb[a[1]] - label.std_res.q - res.d[a...], 0.0)
    if q_new < res.lb[a[1]]
        warp_new += res.lb[a[1]] - q_new
        q_new = res.lb[a[1]]
    end
    return (StandardState(q_new, warp_new))
end

function concatenationCost(res::StandardResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    q_std = min(forwardLabel.std_res.q, backwardLabel.std_res.q)
    q_warp = max(forwardLabel.std_res.q - backwardLabel.std_res.q, 0) + (forwardLabel.std_res.stdWarp + backwardLabel.std_res.stdWarp)
    return StandardState(q_std, q_warp)
end # todo: retornar só o warp

function myInitStateForward(res::CustomResource)
    return ForwardLabel(initStateForward(res)..., StandardState(0.0, 0.0), 0)
end

function myInitStateBackward(res::CustomResource)
    return BackwardLabel(initStateBackward(res)..., StandardState(Inf, 0.0), 0)
end

function myExtendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
    return ForwardLabel(extendAlongArc(res, label, a)..., label.std_res, a[2] - 1)
end

function myExtendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
    return BackwardLabel(extendAlongArc(res, label, a)..., label.std_res, a[2] - 1)
end

function myConcatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    return ForwardLabel(concatenationCost(res, v, forwardLabel, backwardLabel)..., forwardLabel.std_res, backwardLabel.last)
end

function myExtendAlongArc(res::Resources, label::ForwardLabel, a::Tuple{Int, Int})
    return ForwardLabel(extendAlongArc(res.customResource, label, a)..., extendAlongArc(res.stdResource, label, a), a[2] - 1)
end

function myExtendAlongArc(res::Resources, label::BackwardLabel, a::Tuple{Int, Int})
    return BackwardLabel(extendAlongArc(res.customResource, label, a)..., extendAlongArc(res.stdResource, label, a), a[2] - 1)
end

function myConcatenationCost(res::Resources, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    return ForwardLabel(concatenationCost(res.customResource, v, forwardLabel, backwardLabel)..., concatenationCost(res.stdResource, v, forwardLabel, backwardLabel), backwardLabel.last)
end

################ dont touch ################
