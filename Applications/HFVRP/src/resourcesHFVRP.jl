################ user-defined (GenericILS as a package) ################
using GenericILS
import GenericILS: isSymmetric, isCostResource, initStateForward, initStateBackward,
    extendAlongArc, concatenationCost, AbstractResource, ForwardLabel, BackwardLabel

const TOL = 0.0001

"""
Heterogeneous fleet: `nv` vehicle types (`fixed` cost, `factor` on distance, capacity `cap`), demands `d` and
distances `dist`. The whole route cost lives in this resource.
"""
struct CustomResource <: AbstractResource
    nv::Int64
    fixed::Vector{Float64}
    factor::Vector{Float64}
    cap::Vector{Float64}
    d::Matrix{Float64}
    dist::Matrix{Float64}
end

GenericILS.isSymmetric() = false

GenericILS.isCostResource() = true

"""Vehicle type `v`, accumulated cost, load and distance of the prefix."""
struct ForwardState
    v::Int64
    cost::Float64
    load::Float64
    dist::Float64
end

"""Vehicle type `v`, accumulated cost, load and distance of the suffix."""
struct BackwardState
    v::Int64
    cost::Float64
    load::Float64
    dist::Float64
end

"""Forward state of the empty prefix."""
function initStateForward(res::CustomResource)
    return (ForwardState(1, res.fixed[1], 0.0, 0.0), res.fixed[1])
end

"""Backward state of the empty suffix."""
function initStateBackward(res::CustomResource)
    return (BackwardState(1, res.fixed[1], 0.0, 0.0), res.fixed[1])
end

"""Forward extension along `a = (i, j)` (matrix indices): appends `j`. The returned cost is the accumulated cost of the partial route."""
function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
    load_ = label.state.load + res.d[a[1], a[2]]
    dist_ = label.state.dist + res.dist[a[1], a[2]]
    cost_ = label.state.cost

    k_ = findfirst(k -> load_ < res.cap[k] + TOL, 1:res.nv)
    k_ = isnothing(k_) ? res.nv : k_
    if a == (1,1)
        cost_ = 0.0
    else
        cost_ += (res.fixed[k_] - res.fixed[label.state.v]) +
                (res.factor[k_] - res.factor[label.state.v]) * label.state.dist +
                res.factor[k_] * res.dist[a[1], a[2]]
    end
    return isnothing(findfirst(k -> load_ < res.cap[k] + TOL, 1:res.nv)) ?
        (ForwardState(res.nv, cost_, load_, dist_), 100000.0) :
        (ForwardState(k_, cost_, load_, dist_), cost_)
end

"""Backward extension along `a = (next, cur)`: prepends `cur` to the suffix that starts at `next`. The returned cost is the accumulated cost of the partial route."""
function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
    load_ = label.state.load + res.d[a[1], a[2]]
    dist_ = label.state.dist + res.dist[a[1], a[2]]
    cost_ = label.state.cost

    k_ = findfirst(k -> load_ < res.cap[k] + TOL, 1:res.nv)
    k_ = isnothing(k_) ? res.nv : k_
    if a == (1,1)
        cost_ = 0.0
    else
        cost_ += (res.fixed[k_] - res.fixed[label.state.v]) +
                (res.factor[k_] - res.factor[label.state.v]) * label.state.dist +
                res.factor[k_] * res.dist[a[1], a[2]]
    end
    return isnothing(findfirst(k -> load_ < res.cap[k] + TOL, 1:res.nv)) ?
        (BackwardState(res.nv, cost_, load_, dist_), 100000.0) :
        (BackwardState(k_, cost_, load_, dist_), cost_)
end

"""Join the forward and the backward label at the same vertex: cost/feasibility of the whole route."""
function concatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    concDist = forwardLabel.state.dist + backwardLabel.state.dist
    concLoad = forwardLabel.state.load + backwardLabel.state.load - res.d[1, forwardLabel.last+1]

    k_ = findfirst(k -> concLoad < res.cap[k] + TOL, 1:res.nv)
    if isnothing(k_)
        return (ForwardState(res.nv, 100000.0, concLoad, concDist), 100000.0)
    else
        concCost = res.fixed[k_] + res.factor[k_] * concDist
        return (ForwardState(k_, concCost, concLoad, concDist), concCost)
    end
end
