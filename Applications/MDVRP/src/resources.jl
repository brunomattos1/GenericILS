# struct CustomResource <: AbstractResource
#     d::Matrix{Float64}    # arc head demand  (n+1 × n+1)
#     dist::Matrix{Float64} # arc cost matrix  (n+1 × n+1); depot row/col uses min-depot cost
#     Q::Float64            # vehicle capacity
# end

struct CustomResource <: AbstractResource
    C₀::Matrix{Float64}
    C::Matrix{Float64}
end

function isSymmetric()
    return false
end

function isCostResource()
    return true
end

# struct ForwardState
#     load::Float64
#     dist::Float64
# end

# struct BackwardState
#     load::Float64
#     dist::Float64
# end

struct ForwardState
    v::Int64 # In the forward (backward), v indicates the first (last) visited vertex;
    cost::Float64 
end

struct BackwardState
    v::Int64 # In the forward (backward), v indicates the first (last) visited vertex;
    cost::Float64 
end


# function initStateForward(res::CustomResource)
#     return (ForwardState(0.0, 0.0), 0.0)
# end

# function initStateBackward(res::CustomResource)
#     return (BackwardState(0.0, 0.0), 0.0)
# end

# function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
#     load_ = label.state.load + res.d[a[1], a[2]]
#     dist_ = label.state.dist + res.dist[a[1], a[2]]
#     # if load_ > res.Q + 1e-5
#     #     return (ForwardState(load_, dist_), Inf)
#     # end
#     return (ForwardState(load_, dist_), dist_)
# end

# function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
#     load_ = label.state.load + res.d[a[1], a[2]]
#     dist_ = label.state.dist + res.dist[a[1], a[2]]
#     # if load_ > res.Q + 1e-5
#     #     return (BackwardState(load_, dist_), Inf)
#     # end
#     return (BackwardState(load_, dist_), dist_)
# end

# function concatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
#     conc_load = forwardLabel.state.load + backwardLabel.state.load - res.d[1, forwardLabel.last + 1]
#     conc_dist = forwardLabel.state.dist + backwardLabel.state.dist
#     # if conc_load > res.Q + 1e-5
#     #     return (ForwardState(conc_load, conc_dist), Inf)
#     # end
#     return (ForwardState(conc_load, conc_dist), conc_dist)
# end

function initStateForward(res::CustomResource)
    return (ForwardState(0, 0.0), 0.0)
end

function initStateBackward(res::CustomResource)
    return (BackwardState(0, 0.0), 0.0)
end

function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
    if a[1] == 1
        v′ = a[2]
        C′ = label.state.cost
        return (ForwardState(v′,C′), C′) 
    end

    v′ = label.state.v
    if a[2] == 1
        C′ = label.state.cost + res.C₀[label.state.v-1,a[1]-1]
        return (ForwardState(v′,C′), C′) 
    else
        C′ = label.state.cost + res.C[a[1]-1,a[2]-1]
        return (ForwardState(v′,C′), C′) 
    end

end

function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
    if a[1] == 1
        v′ = a[2]
        C′ = label.state.cost
        return (BackwardState(v′,C′), C′)       
    end
    v′ = label.state.v
    if a[2] == 1
        C′ = label.state.cost + res.C₀[a[1]-1,label.state.v-1]
        return (BackwardState(v′,C′), C′) 
    else
        C′ = label.state.cost + res.C[a[1]-1,a[2]-1]
        return (BackwardState(v′,C′), C′) 
    end
end

function concatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    conc = forwardLabel.state.cost + backwardLabel.state.cost
    if forwardLabel.state.v != 0 && backwardLabel.state.v != 0
        conc += res.C₀[forwardLabel.state.v-1,backwardLabel.state.v-1]
    end
    newState = (ForwardState(0,0.0), conc)
    return newState
end
