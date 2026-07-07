struct CustomResource <: AbstractResource
    d::Matrix{Float64}    # arc head demand  (n+1 × n+1)
    dist::Matrix{Float64} # arc cost matrix  (n+1 × n+1); depot row/col uses min-depot cost
    Q::Float64            # vehicle capacity
end

function isSymmetric()
    return false
end

function isCostResource()
    return true
end

struct ForwardState
    load::Float64
    dist::Float64
end

struct BackwardState
    load::Float64
    dist::Float64
end

function initStateForward(_res::CustomResource)
    return (ForwardState(0.0, 0.0), 0.0)
end

function initStateBackward(_res::CustomResource)
    return (BackwardState(0.0, 0.0), 0.0)
end

function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
    load_ = label.state.load + res.d[a[1], a[2]]
    dist_ = label.state.dist + res.dist[a[1], a[2]]
    if load_ > res.Q + 1e-5
        return (ForwardState(load_, dist_), Inf)
    end
    return (ForwardState(load_, dist_), dist_)
end

function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
    load_ = label.state.load + res.d[a[1], a[2]]
    dist_ = label.state.dist + res.dist[a[1], a[2]]
    if load_ > res.Q + 1e-5
        return (BackwardState(load_, dist_), Inf)
    end
    return (BackwardState(load_, dist_), dist_)
end

function concatenationCost(res::CustomResource, _v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    conc_load = forwardLabel.state.load + backwardLabel.state.load - res.d[1, forwardLabel.last + 1]
    conc_dist = forwardLabel.state.dist + backwardLabel.state.dist
    if conc_load > res.Q + 1e-5
        return (ForwardState(conc_load, conc_dist), Inf)
    end
    return (ForwardState(conc_load, conc_dist), conc_dist)
end
