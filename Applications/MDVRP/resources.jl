
################ user-defined ################

const MAX_VEHICLES = 5

const TOL = 0.0001

struct CustomResource
    nv::Int64 #number of vehicles
    fixed::Vector{Float64}
    factor::Vector{Float64}
    cap::Vector{Float64}
    d::Matrix{Float64} # head demand
    dist::Matrix{Float64}
end

function isSymmetric()
    return false
end

function isCostResource()
    return true
end

struct ForwardState
    v::Int64 # vehicle of the path
    cost::Float64 # cost of the path 
    load::Float64 #demand served in the path
    dist::Float64
end

struct BackwardState
    v::Int64 # vehicle of the path
    cost::Float64 # cost of the path 
    load::Float64 #demand served in the path
    dist::Float64
end

################ user-defined ################


################ dont touch ################

struct StandardState
    q::Float64
    stdWarp::Float64
end

struct StandardResource
    d::Matrix{Float64}
    lb::Vector{Float64}
    ub::Vector{Float64}
end

struct Resources
    customResource::CustomResource
    stdResource::StandardResource
end

struct ForwardLabel
    #custom_res::CustomState
    state::ForwardState
    cost::Float64
    std_res::StandardState
    last::Int
end

struct BackwardLabel
    #custom_res::CustomState
    state::BackwardState
    cost::Float64
    std_res::StandardState
    last::Int
end

################ dont touch ################


################ user-defined ################

function initStateForward(res::CustomResource)
return (ForwardState(1, res.fixed[1],0.0,0.0), res.fixed[1]) # 1 indicates the cheapest vehicle
end

function initStateBackward(res::CustomResource)
    return (BackwardState(1, res.fixed[1],0.0,0.0), res.fixed[1])  # 1 indicates the cheapest vehicle
end

function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
    #=
        struct ForwardState
            v::Int64 # vehicle of the path
            cost::Float64 # cost of the path 
            load::Float64 #demand served in the path
            dist::Float64
        end
    =#
    
    load_ = label.state.load + res.d[a[1],a[2]]
    dist_ = label.state.dist + res.dist[a[1],a[2]]
    cost_ = label.state.cost
    
    k_ = 0
    for k=1:res.nv
        if load_ < res.cap[k] + TOL
            k_ = k #finding the cheapest vehicle for the route
            break
        end
    end

    if k_ == 0
        return (ForwardState(1,10000.0,load_,dist_), Inf) #colocar um return infeasible aqui: there is no vehicle with enough capacity for the path
    end

    #@show a[1],a[2], k_

    # (res.fixed[k_] - res.fixed[label.state.v]) -> Increase in the fixed cost
    #(res.factor[k_] - res.factor[label.state.v])*label.state.dist increase in the already traveled distance
    # res.factor[k_]*res.dist[a[1],a[2]] -> cost of the traveled arc
    cost_ += (res.fixed[k_] - res.fixed[label.state.v]) + (res.factor[k_] - res.factor[label.state.v])*label.state.dist + res.factor[k_]*res.dist[a[1],a[2]]
    
    return (ForwardState(k_,cost_,load_,dist_), cost_)

end

function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
    #=
        struct BackwardState
            v::Int64 # vehicle of the path
            cost::Float64 # cost of the path 
            load::Float64 #demand served in the path
            dist::Float64
        end
    =#
    
    load_ = label.state.load + res.d[a[1],a[2]]
    dist_ = label.state.dist + res.dist[a[1],a[2]]
    cost_ = label.state.cost
    
    k_ = 0
    for k=1:res.nv
        if load_ < res.cap[k] + TOL
            k_ = k #finding the cheapest vehicle for the route
            break
        end
    end

    if k_ == 0
        return (BackwardState(res.nv,10000.0,load_,dist_), Inf) #colocar um return infeasible aqui: there is no vehicle with enough capacity for the path
    end

    # (res.fixed[k_] - res.fixed[label.state.v]) -> Increase in the fixed cost
    #(res.factor[k_] - res.factor[label.state.v])*label.state.dist increase in the already traveled distance
    # res.factor[k_]*res.dist[a[1],a[2]] -> cost of the traveled arc
    cost_ += (res.fixed[k_] - res.fixed[label.state.v]) + (res.factor[k_] - res.factor[label.state.v])*label.state.dist + res.factor[k_]*res.dist[a[1],a[2]]
    
    return (BackwardState(k_,cost_,load_,dist_), cost_)
end

function concatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    #conc_cost = forwardLabel.state.cost + backwardLabel.state.cost
    conc_dist = forwardLabel.state.dist + backwardLabel.state.dist
    conc_load = forwardLabel.state.load + backwardLabel.state.load - res.d[1,forwardLabel.last+1]

    #@show forwardLabel.last+1, backwardLabel.last+1
    
    k_ = 0
    for k=1:res.nv
        if conc_load < res.cap[k] + TOL
            k_ = k #finding the cheapest vehicle for the route
            break
        end
    end

    if k_ == 0
        return (ForwardState(res.nv,10000.0,conc_load,conc_dist), Inf) #colocar um return infeasible aqui: there is no vehicle with enough capacity for the path
    else
        conc_cost = res.fixed[k_] + res.factor[k_]*conc_dist
        return (ForwardState(k_,conc_cost,conc_load,conc_dist), conc_cost)
    end
    
end

################ user-defined ################

################ dont touch ################

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

function myInitStateForward(res::CustomResource)
    return ForwardLabel(initStateForward(res)..., StandardState(0.0, 0.0), 0)
end

function myInitStateBackward(res::CustomResource)
    return BackwardLabel(initStateBackward(res)..., StandardState(Inf, 0.0), 0)
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

function myExtendAlongArc(res::Resources, label::ForwardLabel, a::Tuple{Int, Int})
    return ForwardLabel(extendAlongArc(res.customResource, label, a)..., extendAlongArc(res.stdResource, label, a), a[2] - 1)
end

function myExtendAlongArc(res::Resources, label::BackwardLabel, a::Tuple{Int, Int})
    return BackwardLabel(extendAlongArc(res.customResource, label, a)..., extendAlongArc(res.stdResource, label, a), a[2] - 1)
end

function myConcatenationCost(res::Resources, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    return ForwardLabel(concatenationCost(res.customResource, v, forwardLabel, backwardLabel)..., concatenationCost(res.stdResource, v, forwardLabel, backwardLabel), backwardLabel.last)
end

################ dont touch ################