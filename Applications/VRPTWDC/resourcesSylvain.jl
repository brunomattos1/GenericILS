const DEBUG_MODE = false


struct CustomResource
    d::Matrix{Float64}
    t::Matrix{Float64}
    l::Vector{Float64}
    u::Vector{Float64}
    D::Float64
    Q::Float64
end

struct StandardResource1
    d::Matrix{Float64}
    lb::Vector{Float64}
    ub::Vector{Float64}
end

struct StandardResource2
    d::Matrix{Float64}
    lb::Vector{Float64}
    ub::Vector{Float64}
end

struct Resources
    customResource::CustomResource
    stdResource1::StandardResource
    stdResource2::StandardResource
end

function isSymmetric()
    return false
end

function isCostResource()
    return false
end

struct ForwardState
    q::Float64
    ET::Float64
    RD::Float64
    TB::Float64
end

struct BackwardState
    q::Float64
    ET::Float64
    RD::Float64
    TB::Float64
end

struct StandardState
    q::Float64
    stdWarp::Float64
end

struct ForwardLabel
    state::ForwardState
    cost::Float64
    std_res::StandardState
    last::Int
end

struct BackwardLabel
    state::BackwardState
    cost::Float64
    std_res::StandardState
    last::Int
end
function myInitStateForward(res::CustomResource)
    return ForwardLabel(initStateForward(res)..., StandardState(0.0, 0.0), 0)
end

function myInitStateBackward(res::CustomResource)
    return BackwardLabel(initStateBackward(res)..., StandardState(Inf, 0.0), 0)
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


# CUSTOM, user-dependent

function initStateForward(res::CustomResource)
    return (ForwardState(0.0, 0.0, 0.0, res.u[1]), 0.0)
end

function initStateBackward(res::CustomResource)
    return (BackwardState(0.0, res.u[1], 0.0, res.u[1]), 0.0)
end

function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
    TB = label.state.TB
    RD = label.state.RD
    lj = res.l[a[2]]
    uj = res.u[a[2]]

    cap = label.state.q + res.d[(1, a[2])...]
    ET⁰ = label.state.ET + res.t[a...]
    ET′ = max(ET⁰, lj)
    wj = ET′ - ET⁰
    TB′ = max(0, min(TB - wj, uj - ET′))
    RD′ = RD + res.t[a...] + max(0, wj - TB)

    if label.cost == Inf
        return (ForwardState(cap, ET′, RD′, TB′), Inf)
    end
    if ET⁰ > uj || cap > res.Q || RD′ > res.D
        return (ForwardState(cap, ET′, RD′, TB′), Inf)
    else
        return (ForwardState(cap, ET′, RD′, TB′), 0.0)
    end
end

function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
    TB = label.state.TB
    RD = label.state.RD
    li = res.l[a[2]]
    ui = res.u[a[2]]
    cap = label.state.q + res.d[(1, a[1])...]

    ET⁰ = label.state.ET - res.t[(a[2],a[1])...]
    ET′ = min(ET⁰, ui)
    wi = ET⁰ - ET′
    TB′ = max(0, min(TB - wi, ET′ - li))

    RD′ = RD + res.t[(a[2], a[1])...] + max(0, wi - TB)
    if label.cost == Inf
        return (BackwardState(cap, ET′, RD′, TB′), Inf)
    end
    if ET⁰ < li || cap > res.Q || RD′ > res.D
        return (BackwardState(cap, ET′, RD′, TB′), Inf)
    else
        return (BackwardState(cap, ET′, RD′, TB′), 0.0)
    end
end

function concatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    W = max(0, (backwardLabel.state.ET - backwardLabel.state.TB) - (forwardLabel.state.ET + forwardLabel.state.TB))
    if forwardLabel.cost == Inf || backwardLabel.cost == Inf
        return (ForwardState(0.0, 0.0, 0.0, 0.0), Inf)
    end 
    # @show W
    if forwardLabel.state.RD + backwardLabel.state.RD + W > res.D || forwardLabel.state.ET > backwardLabel.state.ET || forwardLabel.state.q + backwardLabel.state.q > res.Q
        return (ForwardState(0.0, 0.0, 0.0, 0.0), Inf)
    else
        return (ForwardState(0.0, 0.0, 0.0, 0.0), 0.0)
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
