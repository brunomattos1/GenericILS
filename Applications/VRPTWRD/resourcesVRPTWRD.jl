################ user-defined (GenericILS as a package) ################
using GenericILS
import GenericILS: isSymmetric, isCostResource, initStateForward, initStateBackward,
    extendAlongArc, concatenationCost, AbstractResource, ForwardLabel, BackwardLabel

"""Release dates `r` with due dates `u`, travel times `t`, demands `q` and capacity `Q`."""
struct CustomResource <: AbstractResource
    t::Matrix{Float64} # time cost
    u::Vector{Int}     # upper window (due dates)
    r::Vector{Int}     # release dates
    q::Matrix{Int}     # demands
    Q::Float64         # capacity
end

GenericILS.isSymmetric() = false

GenericILS.isCostResource() = false

"""Elapsed time `T`, slack `U`, latest release date `RD` and load `cap` of the prefix."""
struct ForwardState
    T::Float64
    U::Float64
    RD::Float64
    cap::Float64
end

"""Elapsed time `T`, slack `U`, latest release date `RD` and load `cap` of the suffix."""
struct BackwardState
    T::Float64
    U::Float64
    RD::Float64
    cap::Float64
end

"""Forward state of the empty prefix."""
function initStateForward(res::CustomResource)
    return (ForwardState(0.0, Inf, 0.0, 0.0), 0.0)
end

"""Backward state of the empty suffix."""
function initStateBackward(res::CustomResource)
    return (BackwardState(0.0, Inf, 0.0, 0.0), 0.0)
end

"""Forward extension along `a = (i, j)` (matrix indices): appends `j`. The returned cost is `0.0`, or `Inf` when a rule is violated."""
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

"""Backward extension along `a = (next, cur)`: prepends `cur` to the suffix that starts at `next`. The returned cost is `0.0`, or `Inf` when a rule is violated."""
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

"""Join the forward and the backward label at the same vertex: cost/feasibility of the whole route."""
function concatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    if (max(forwardLabel.state.RD, backwardLabel.state.RD) <= min(backwardLabel.state.U - forwardLabel.state.T, forwardLabel.state.U)) && (forwardLabel.state.cap + backwardLabel.state.cap) <= res.Q + 1e-6
        return (ForwardState(0.0, 0.0, 0.0, 0.0), 0.0)
    else
        return (ForwardState(0.0, 0.0, 0.0, 0.0), Inf)
    end
end
