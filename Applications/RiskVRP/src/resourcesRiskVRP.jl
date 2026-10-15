################ user-defined (GenericILS as a package) ################
using GenericILS
import GenericILS: isSymmetric, isCostResource, initStateForward, initStateBackward,
    extendAlongArc, concatenationCost, AbstractResource, ForwardLabel, BackwardLabel

"""Risk-constrained VRP: demand of the tail vertex `dt`, cost matrix `c` and the maximum risk `Rmax` of a route."""
struct CustomResource <: AbstractResource
    dt::Matrix{Float64}  # demand of tail vertex
    c::Matrix{Float64}   # cost matrix
    Rmax::Float64        # maximum risk of a route
end

GenericILS.isSymmetric() = false

GenericILS.isCostResource() = false

"""Accumulated risk `R`, demand `D` and cost `C` of the prefix."""
struct ForwardState
    R::Float64  # accumulated risk
    D::Float64  # accumulated demand (used in forward)
    C::Float64  # accumulated cost (used in backward)
end

"""Accumulated risk `R`, demand `D` and cost `C` of the suffix."""
struct BackwardState
    R::Float64
    D::Float64
    C::Float64
end

"""Forward state of the empty prefix."""
function initStateForward(res::CustomResource)
    return (ForwardState(0.0, 0.0, 0.0), 0.0)
end

"""Backward state of the empty suffix."""
function initStateBackward(res::CustomResource)
    return (BackwardState(0.0, 0.0, 0.0), 0.0)
end

"""Forward extension along `a = (i, j)` (matrix indices): appends `j`. The returned cost is `0.0`, or `Inf` when a rule is violated."""
function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
    R′ = label.state.R + (label.state.D + res.dt[a[1], a[2]]) * res.c[a[1], a[2]]
    D′ = label.state.D + res.dt[a[1], a[2]]
    if R′ > res.Rmax + 1e-5
        return (ForwardState(R′, D′, label.state.C), Inf)
    else
        return (ForwardState(R′, D′, label.state.C), 0.0)
    end
end

"""Backward extension along `a = (next, cur)`: prepends `cur` to the suffix that starts at `next`. The returned cost is `0.0`, or `Inf` when a rule is violated."""
function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
    R′ = label.state.R + (label.state.C + res.c[a[2], a[1]]) * res.dt[a[2], a[1]]
    C′ = label.state.C + res.c[a[2], a[1]]
    if R′ > res.Rmax + 1e-5
        return (BackwardState(R′, label.state.D, C′), Inf)
    else
        return (BackwardState(R′, label.state.D, C′), 0.0)
    end
end

"""Join the forward and the backward label at the same vertex: cost/feasibility of the whole route."""
function concatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    R′ = forwardLabel.state.R + backwardLabel.state.R + forwardLabel.state.D * backwardLabel.state.C
    if R′ > res.Rmax + 1e-5
        return (ForwardState(0.0, 0.0, 0.0), Inf)
    else
        return (ForwardState(0.0, 0.0, 0.0), 0.0)
    end
end
