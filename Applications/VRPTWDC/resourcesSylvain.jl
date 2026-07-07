################ user-defined ################
struct CustomResource <: AbstractResource
    d::Matrix{Float64}  # demand matrix
    t::Matrix{Float64}  # time matrix
    l::Vector{Float64}  # lower bounds (ready times)
    u::Vector{Float64}  # upper bounds (due dates)
    D::Float64          # route duration limit
    Q::Float64          # vehicle capacity
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
    wj  = ET′ - ET⁰
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

    ET⁰ = label.state.ET - res.t[(a[2], a[1])...]
    ET′ = min(ET⁰, ui)
    wi  = ET⁰ - ET′
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
    if forwardLabel.state.RD + backwardLabel.state.RD + W > res.D || forwardLabel.state.ET > backwardLabel.state.ET || forwardLabel.state.q + backwardLabel.state.q > res.Q
        return (ForwardState(0.0, 0.0, 0.0, 0.0), Inf)
    else
        return (ForwardState(0.0, 0.0, 0.0, 0.0), 0.0)
    end
end
