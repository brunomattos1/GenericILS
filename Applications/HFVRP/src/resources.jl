const TOL = 0.0001

struct CustomResource <: AbstractResource
    nv::Int64
    fixed::Vector{Float64}
    factor::Vector{Float64}
    cap::Vector{Float64}
    d::Matrix{Float64}
    dist::Matrix{Float64}
end

function isSymmetric()
    return false
end

function isCostResource()
    return true
end

struct ForwardState
    v::Int64
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

function concatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    conc_dist = forwardLabel.state.dist + backwardLabel.state.dist
    conc_load = forwardLabel.state.load + backwardLabel.state.load - res.d[1, forwardLabel.last+1]

    k_ = findfirst(k -> conc_load < res.cap[k] + TOL, 1:res.nv)
    if isnothing(k_)
        return (ForwardState(res.nv, 100000.0, conc_load, conc_dist), 100000.0)
    else
        conc_cost = res.fixed[k_] + res.factor[k_] * conc_dist
        return (ForwardState(k_, conc_cost, conc_load, conc_dist), conc_cost)
    end
end

# function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
#     #=
#         struct ForwardState
#             v::Int64 # vehicle of the path
#             cost::Float64 # cost of the path 
#             load::Float64 #demand served in the path
#             dist::Float64
#         end
#     =#
    
#     load_ = label.state.load + res.d[a[1],a[2]]
#     dist_ = label.state.dist + res.dist[a[1],a[2]]
#     cost_ = label.state.cost
    
#     k_ = 0
#     for k=1:res.nv
#         if load_ < res.cap[k] + TOL
#             k_ = k #finding the cheapest vehicle for the route
#             break
#         end
#     end

#     # if k_ == 0
#     #     return (ForwardState(1,10000.0,load_,dist_), 0.0) #colocar um return infeasible aqui: there is no vehicle with enough capacity for the path
#     # end

#     if k_ == 0
#         k_ = res.nv
#         cost_ += (res.fixed[k_] - res.fixed[label.state.v]) + (res.factor[k_] - res.factor[label.state.v])*label.state.dist + res.factor[k_]*res.dist[a[1],a[2]]
    
#         return (ForwardState(res.nv,cost_,load_,dist_), 10000.0) #colocar um return infeasible aqui: there is no vehicle with enough capacity for the path
#     end

#     #@show a[1],a[2], k_

#     # (res.fixed[k_] - res.fixed[label.state.v]) -> Increase in the fixed cost
#     #(res.factor[k_] - res.factor[label.state.v])*label.state.dist increase in the already traveled distance
#     # res.factor[k_]*res.dist[a[1],a[2]] -> cost of the traveled arc
#     cost_ += (res.fixed[k_] - res.fixed[label.state.v]) + (res.factor[k_] - res.factor[label.state.v])*label.state.dist + res.factor[k_]*res.dist[a[1],a[2]]
    
#     return (ForwardState(k_,cost_,load_,dist_), cost_)

# end

# function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
#     #=
#         struct BackwardState
#             v::Int64 # vehicle of the path
#             cost::Float64 # cost of the path 
#             load::Float64 #demand served in the path
#             dist::Float64
#         end
#     =#
    
#     load_ = label.state.load + res.d[a[1],a[2]]
#     dist_ = label.state.dist + res.dist[a[1],a[2]]
#     cost_ = label.state.cost
    
#     k_ = 0
#     for k=1:res.nv
#         if load_ < res.cap[k] + TOL
#             k_ = k #finding the cheapest vehicle for the route
#             break
#         end
#     end

#     if k_ == 0
#         k_ = res.nv
#         cost_ += (res.fixed[k_] - res.fixed[label.state.v]) + (res.factor[k_] - res.factor[label.state.v])*label.state.dist + res.factor[k_]*res.dist[a[1],a[2]]
    
#         return (BackwardState(res.nv,cost_,load_,dist_), 10000.0) #colocar um return infeasible aqui: there is no vehicle with enough capacity for the path
#     end

#     # (res.fixed[k_] - res.fixed[label.state.v]) -> Increase in the fixed cost
#     #(res.factor[k_] - res.factor[label.state.v])*label.state.dist increase in the already traveled distance
#     # res.factor[k_]*res.dist[a[1],a[2]] -> cost of the traveled arc
#     cost_ += (res.fixed[k_] - res.fixed[label.state.v]) + (res.factor[k_] - res.factor[label.state.v])*label.state.dist + res.factor[k_]*res.dist[a[1],a[2]]
    
#     return (BackwardState(k_,cost_,load_,dist_), cost_)
# end

# function concatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
#     #conc_cost = forwardLabel.state.cost + backwardLabel.state.cost
#     conc_dist = forwardLabel.state.dist + backwardLabel.state.dist
#     conc_load = forwardLabel.state.load + backwardLabel.state.load - res.d[1,forwardLabel.last+1]

#     #@show forwardLabel.last+1, backwardLabel.last+1
    
#     k_ = 0
#     for k=1:res.nv
#         if conc_load < res.cap[k] + TOL
#             k_ = k #finding the cheapest vehicle for the route
#             break
#         end
#     end

#     if k_ == 0
#         return (ForwardState(res.nv,10000.0,conc_load,conc_dist), 0.0) #colocar um return infeasible aqui: there is no vehicle with enough capacity for the path
#     else
#         conc_cost = res.fixed[k_] + res.factor[k_]*conc_dist
#         return (ForwardState(k_,conc_cost,conc_load,conc_dist), conc_cost)
#     end
    
# end