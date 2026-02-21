const DEBUG_MODE = false


struct CustomResource
    t::Matrix{Float64} # time cost
    u::Vector{Int} # upper window
    r::Vector{Int} # release dates
    q::Matrix{Int} # demands
    Q::Float64 # capacity
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
        T::Float64
        U::Float64
        RD::Float64
        cap::Float64
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
        T::Float64
        U::Float64
        RD::Float64
        cap::Float64
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
        return ForwardLabel(initStateForward()..., StandardState(0.0, 0.0), 0)
    end

    function myInitStateBackward(res::CustomResource)
        return BackwardLabel(initStateBackward()..., StandardState(Inf, 0.0), 0)
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

    function myInitStateForward(res::CustomResource)
        return ForwardLabel(initStateForward()..., StandardState(0.0, 0.0), 0)
    end

    function myInitStateBackward(res::CustomResource)
        return BackwardLabel(initStateBackward()..., StandardState(Inf, 0.0), 0)
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

function initStateForward()
    return (CustomState(0.0, Inf, 0.0, 0.0), 0.0)
end

function initStateBackward()
    return (CustomState(0.0, Inf, 0.0, 0.0), 0.0)
end

function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
    T′ = label.custom_res.T + res.t[a...]
    U′ = min(label.custom_res.U, res.u[a[2]] - T′)
    RD′ = max(label.custom_res.RD, res.r[a[2]])
    cap′ = label.custom_res.cap + res.q[1, a[2]]
    state = CustomState(T′, U′, RD′, cap′)

    if T′ > res.u[a[2]] + 1e-12 || U′ < max(0, RD′) - 1e-12 || cap′ > res.Q + 1e-12
        return (state, Inf)
    end
    return (state, 0.0)
end

function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
    T′ = 0.0
    U′ = min(label.custom_res.U - res.t[a...], res.u[a[2]])
    RD′ = max(label.custom_res.RD, res.r[a[2]])
    cap′ = label.custom_res.cap + res.q[1, a[1]]
    state = CustomState(T′, U′, RD′, cap′)
    
    if U′ < RD′ - 1e-6 || cap′ > res.Q + 1e-6
        return (state, Inf)
    end
    return (state, 0.0)
end

function concatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    if (max(forwardLabel.custom_res.RD, backwardLabel.custom_res.RD) <= min(backwardLabel.custom_res.U - forwardLabel.custom_res.T, forwardLabel.custom_res.U)) && (forwardLabel.custom_res.cap + backwardLabel.custom_res.cap) <= res.Q + 1e-6
        return (CustomState(0.0, 0.0, 0.0, 0.0), 0.0)
    else
        return (CustomState(0.0, 0.0, 0.0, 0.0), Inf)
    end
end

# STANDARD, user-independent


function extendAlongArc(res::StandardResource, label::ForwardLabel, a::Tuple{Int, Int})
    q_std = max(label.std_res.q + res.d[a...], res.lb[a[2]])
    if q_std > res.ub[a[2]] + 1e-12
        q_std = res.ub[a[2]]
    end
    q_warp = label.std_res.stdWarp
    if label.std_res.q + res.d[a...] > res.ub[a[2]] + 1e-12
        q_warp += label.std_res.q + res.d[a...] - res.ub[a[2]] - 1e-12
    end
    # q_std = max(min(label.std_res.q + res.d[a...], res.ub[a[2]]), res.lb[a[2]])
    # q_warp = label.std_res.stdWarp + max(label.std_res.q + res.d[a...] - res.ub[a[2]] - 1e-12, 0.0)
    return (StandardState(q_std, q_warp))
end

function extendAlongArc(res::StandardResource, label::BackwardLabel, a::Tuple{Int, Int})
    a = (a[2], a[1])

    q_std = min(label.std_res.q - res.d[a...], res.ub[a[1]])
    q_warp = label.std_res.stdWarp
    if q_std < res.lb[a[1]] - 1e-12
        q_warp += res.lb[a[1]] - q_std
    end
    if q_std < res.lb[a[1]] - 1e-12
        q_std = res.lb[a[1]]
    end
    return (StandardState(q_std, q_warp))
end

function concatenationCost(res::StandardResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    q_std = min(forwardLabel.std_res.q, backwardLabel.std_res.q)
    q_warp = max(forwardLabel.std_res.q - backwardLabel.std_res.q, 0) + (forwardLabel.std_res.stdWarp + backwardLabel.std_res.stdWarp)
    return StandardState(q_std, q_warp)
end
