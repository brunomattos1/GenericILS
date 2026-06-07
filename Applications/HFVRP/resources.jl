################ user-defined ################
const MAX_VEHICLES = 6
const TOL = 0.0001

struct CustomResource <: AbstractResource
    nv::Int64
    fixed::Vector{Float64}
    factor::Vector{Float64}
    cap::Vector{Float64}
    d::Matrix{Float64}   # head demand matrix
    dist::Matrix{Float64}
end

function isSymmetric()
    return false
end

function isCostResource()
    return true
end

struct ForwardState
    v::Int64     # cheapest feasible vehicle type
    cost::Float64
    load::Float64
    dist::Float64
end

struct BackwardState
    v::Int64
    cost::Float64
    load::Float64
    dist::Float64
end

function initStateForward(res::CustomResource)
    return (ForwardState(1, res.fixed[1], 0.0, 0.0), res.fixed[1])
end

function initStateBackward(res::CustomResource)
    return (BackwardState(1, res.fixed[1], 0.0, 0.0), res.fixed[1])
end

function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
    load_ = label.state.load + res.d[a[1], a[2]]
    dist_ = label.state.dist + res.dist[a[1], a[2]]
    cost_ = label.state.cost

    k_ = 0
    for k = 1:res.nv
        if load_ < res.cap[k] + TOL
            k_ = k
            break
        end
    end

    if k_ == 0
        k_ = res.nv
        cost_ += (res.fixed[k_] - res.fixed[label.state.v]) + (res.factor[k_] - res.factor[label.state.v])*label.state.dist + res.factor[k_]*res.dist[a[1], a[2]]
        return (ForwardState(res.nv, cost_, load_, dist_), 0.0)
    end

    cost_ += (res.fixed[k_] - res.fixed[label.state.v]) + (res.factor[k_] - res.factor[label.state.v])*label.state.dist + res.factor[k_]*res.dist[a[1], a[2]]
    return (ForwardState(k_, cost_, load_, dist_), cost_)
end

function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
    load_ = label.state.load + res.d[a[1], a[2]]
    dist_ = label.state.dist + res.dist[a[1], a[2]]
    cost_ = label.state.cost

    k_ = 0
    for k = 1:res.nv
        if load_ < res.cap[k] + TOL
            k_ = k
            break
        end
    end

    if k_ == 0
        k_ = res.nv
        cost_ += (res.fixed[k_] - res.fixed[label.state.v]) + (res.factor[k_] - res.factor[label.state.v])*label.state.dist + res.factor[k_]*res.dist[a[1], a[2]]
        return (BackwardState(res.nv, cost_, load_, dist_), 0.0)
    end

    cost_ += (res.fixed[k_] - res.fixed[label.state.v]) + (res.factor[k_] - res.factor[label.state.v])*label.state.dist + res.factor[k_]*res.dist[a[1], a[2]]
    return (BackwardState(k_, cost_, load_, dist_), cost_)
end

function concatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    conc_dist = forwardLabel.state.dist + backwardLabel.state.dist
    conc_load = forwardLabel.state.load + backwardLabel.state.load - res.d[1, forwardLabel.last+1]

    k_ = 0
    for k = 1:res.nv
        if conc_load < res.cap[k] + TOL
            k_ = k
            break
        end
    end

    if k_ == 0
        return (ForwardState(res.nv, 10000.0, conc_load, conc_dist), 0.0)
    else
        conc_cost = res.fixed[k_] + res.factor[k_]*conc_dist
        return (ForwardState(k_, conc_cost, conc_load, conc_dist), conc_cost)
    end
end
