const DEBUG_MODE = false


struct CustomResource
    d::Matrix{Float64}
    Q::Float64
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

function isSymmetric()
    return false
end


@static if DEBUG_MODE
    struct CustomState
        q::Float64
    end

    struct StandardState
        q::Float64
        stdWarp::Float64
    end

    struct ForwardLabel
        custom_res::CustomState
        cost::Float64
        std_res::StandardState
        path::Vector{Int}
        last::Int
    end

    struct BackwardLabel
        custom_res::CustomState
        cost::Float64
        std_res::StandardState
        path::Vector{Int}
        last::Int
    end
    function myInitStateForward()
        return ForwardLabel(initStateForward()..., StandardState(0.0, 0.0), [0], 0)
    end

    function myInitStateBackward()
        return BackwardLabel(initStateBackward()..., StandardState(Inf, 0.0), [0], 0)
    end

    function myExtendAlongArc(res::Resources, label::ForwardLabel, a::Tuple{Int, Int})
        return ForwardLabel(extendAlongArc(res.customResource, label, a)..., extendAlongArc(res.stdResource, label, a), vcat(label.path, a[2]-1), a[2]-1)
    end

    function myExtendAlongArc(res::Resources, label::BackwardLabel, a::Tuple{Int, Int})
        return BackwardLabel(extendAlongArc(res.customResource, label, a)..., extendAlongArc(res.stdResource, label, a), vcat(a[2]-1, label.path), a[2]-1)
    end

    function myConcatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
        return ForwardLabel(concatenationCost(res.customResource, v, forwardLabel, backwardLabel)..., concatenationCost(res.stdResource, v, forwardLabel, backwardLabel), vcat(forwardLabel.path, backwardLabel.path), backwardLabel.last)
    end

    function myExtendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
        return ForwardLabel(extendAlongArc(res, label, a)..., label.std_res, vcat(label.path, a[2]-1), a[2]-1)
    end

    function myExtendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
        return BackwardLabel(extendAlongArc(res, label, a)..., label.std_res, vcat(a[2]-1, label.path), a[2]-1)
    end

    function myConcatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
        return ForwardLabel(concatenationCost(res, v, forwardLabel, backwardLabel)..., forwardLabel.std_res, vcat(forwardLabel.path, backwardLabel.path), backwardLabel.last)
    end
else
    struct CustomState
        q::Float64
    end

    struct StandardState
        q::Float64
        stdWarp::Float64
    end

    struct ForwardLabel
        custom_res::CustomState
        cost::Float64
        std_res::StandardState
        last::Int
    end

    struct BackwardLabel
        custom_res::CustomState
        cost::Float64
        std_res::StandardState
        last::Int
    end
    function myInitStateForward(res::CustomResource)
        return ForwardLabel(initStateForward(res::CustomResource)..., StandardState(0.0, 0.0), 0)
    end

    function myInitStateBackward(res::CustomResource)
        return BackwardLabel(initStateBackward(res::CustomResource)..., StandardState(Inf, 0.0), 0)
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

    function myExtendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
        return ForwardLabel(extendAlongArc(res, label, a)..., label.std_res, a[2] - 1)
    end

    function myExtendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
        return BackwardLabel(extendAlongArc(res, label, a)..., label.std_res, a[2] - 1)
    end

    function myConcatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
        return ForwardLabel(concatenationCost(res, v, forwardLabel, backwardLabel)..., forwardLabel.std_res, backwardLabel.last)
    end
end




# CUSTOM, user-dependent

function initStateForward(res::CustomResource)
    return (CustomState(0.0), 0.0)
end

function initStateBackward(res::CustomResource)
    return (CustomState(0.0), 0.0)
end

function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
    q_custom = label.custom_res.q + res.d[(1, a[2])...]
    if q_custom > res.Q + 1e-5
        return (CustomState(q_custom), Inf)
    else
        return (CustomState(q_custom), 0.0)
    end
end

function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
    q_custom = label.custom_res.q + res.d[(1, a[1])...]
    if q_custom > res.Q + 1e-5
        return (CustomState(q_custom), Inf)
    else
        return (CustomState(q_custom), 0.0)
    end
end

function concatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    if forwardLabel.custom_res.q + backwardLabel.custom_res.q > res.Q + 1e-5
        newState = (CustomState(forwardLabel.custom_res.q + backwardLabel.custom_res.q), Inf)
        return newState
    else
        newState = (CustomState(forwardLabel.custom_res.q + backwardLabel.custom_res.q), 0.0)
        return newState
    end
end

# STANDARD, user-independent

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
