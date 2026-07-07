################ user-defined ################
struct CustomResource <: AbstractResource
    t::Matrix{Float64} # time cost
    u::Vector{Int}     # upper window (due dates)
    r::Vector{Int}     # release dates
    q::Matrix{Int}     # demands
    Q::Float64         # capacity
end

function isSymmetric()
    return false
end

function isCostResource()
    return false
end

struct ForwardState
    T::Float64
    U::Float64
    RD::Float64
    cap::Float64
end

struct BackwardState
    T::Float64
    U::Float64
    RD::Float64
    cap::Float64
end

function initStateForward(res::CustomResource)
    return (ForwardState(0.0, Inf, 0.0, 0.0), 0.0)
end

function initStateBackward(res::CustomResource)
    return (BackwardState(0.0, Inf, 0.0, 0.0), 0.0)
end

function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
    T′ = label.state.T + res.t[a...]
    U′ = min(label.state.U, res.u[a[2]] - T′)
    RD′ = max(label.state.RD, res.r[a[2]])
    cap′ = label.state.cap + res.q[1, a[2]]
    state = ForwardState(T′, U′, RD′, cap′)

    if T′ > res.u[a[2]] + 1e-12 || U′ < max(0, RD′) - 1e-12 || cap′ > res.Q + 1e-12
        return (state, Inf)
    end
    return (state, 0.0)
end

function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
    T′ = 0.0
    U′ = min(label.state.U - res.t[a...], res.u[a[2]])
    RD′ = max(label.state.RD, res.r[a[2]])
    cap′ = label.state.cap + res.q[1, a[1]]
    state = BackwardState(T′, U′, RD′, cap′)

    if U′ < RD′ - 1e-6 || cap′ > res.Q + 1e-6
        return (state, Inf)
    end
    return (state, 0.0)
end

function concatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    if (max(forwardLabel.state.RD, backwardLabel.state.RD) <= min(backwardLabel.state.U - forwardLabel.state.T, forwardLabel.state.U)) && (forwardLabel.state.cap + backwardLabel.state.cap) <= res.Q + 1e-6
        return (ForwardState(0.0, 0.0, 0.0, 0.0), 0.0)
    else
        return (ForwardState(0.0, 0.0, 0.0, 0.0), Inf)
    end
end
