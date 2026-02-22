const DEBUG_MODE = false

const N = 30

struct CustomResource
    n::Int64 #number of requests 
    d::Matrix{Float64} # demand of the arc head
    Q::Float64 # vehicle capacity
end

struct StandardResource #não podemos mexer. Só é permitido 1 recurso standard
    d::Matrix{Float64} #consumo do recurso standard nos arcos
    lb::Vector{Float64}
    ub::Vector{Float64}
end

struct Resources
    customResource::CustomResource
    stdResource::StandardResource
end

function isSymmetric()
    return false
end

function isCostResource()
    return false
end


@static if DEBUG_MODE
    struct CustomState
        q::Float64
    end

    struct StandardState
        q::Float64
        stdWarp::Float64
    end

    struct ForwardLabel
        custom_res::CustomState
        cost::Float64
        std_res::StandardState
        path::Vector{Int}
        last::Int
    end

    struct BackwardLabel
        custom_res::CustomState
        cost::Float64
        std_res::StandardState
        path::Vector{Int}
        last::Int
    end
    function myInitStateForward()
        return ForwardLabel(initStateForward()..., StandardState(0.0, 0.0), [0], 0)
    end

    function myInitStateBackward()
        return BackwardLabel(initStateBackward()..., StandardState(Inf, 0.0), [0], 0)
    end

    function myExtendAlongArc(res::Resources, label::ForwardLabel, a::Tuple{Int, Int})
        return ForwardLabel(extendAlongArc(res.customResource, label, a)..., extendAlongArc(res.stdResource, label, a), vcat(label.path, a[2]-1), a[2]-1)
    end

    function myExtendAlongArc(res::Resources, label::BackwardLabel, a::Tuple{Int, Int})
        return BackwardLabel(extendAlongArc(res.customResource, label, a)..., extendAlongArc(res.stdResource, label, a), vcat(a[2]-1, label.path), a[2]-1)
    end

    function myConcatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
        return ForwardLabel(concatenationCost(res.customResource, v, forwardLabel, backwardLabel)..., concatenationCost(res.stdResource, v, forwardLabel, backwardLabel), vcat(forwardLabel.path, backwardLabel.path), backwardLabel.last)
    end

    function myExtendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
        return ForwardLabel(extendAlongArc(res, label, a)..., label.std_res, vcat(label.path, a[2]-1), a[2]-1)
    end

    function myExtendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
        return BackwardLabel(extendAlongArc(res, label, a)..., label.std_res, vcat(a[2]-1, label.path), a[2]-1)
    end

    function myConcatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
        return ForwardLabel(concatenationCost(res, v, forwardLabel, backwardLabel)..., forwardLabel.std_res, vcat(forwardLabel.path, backwardLabel.path), backwardLabel.last)
    end
else
    struct CustomState
        load::Float64 # vehicle capacity
        visited::Vector{Bool}
    end 

    struct ForwardState
        load::Float64 # vehicle capacity
        visited::Vector{Bool}
    end

    struct BackwardState
        load::Float64 # vehicle capacity
        visited::Vector{Bool}
    end

    struct StandardState #Não mexer
        q::Float64
        stdWarp::Float64 
    end

    struct ForwardLabel #Não mexer
        state::ForwardState
        cost::Float64
        std_res::StandardState
        last::Int
    end

    struct BackwardLabel #Não mexer
        state::BackwardState
        cost::Float64
        std_res::StandardState
        last::Int
    end
    function myInitStateForward(res::CustomResource)
        return ForwardLabel(initStateForward(res::CustomResource)..., StandardState(0.0, 0.0), 0)
    end

    function myInitStateBackward(res::CustomResource)
        return BackwardLabel(initStateBackward(res::CustomResource)..., StandardState(Inf, 0.0), 0)
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

    function myExtendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
        return ForwardLabel(extendAlongArc(res, label, a)..., label.std_res, a[2] - 1)
    end

    function myExtendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
        return BackwardLabel(extendAlongArc(res, label, a)..., label.std_res, a[2] - 1)
    end

    function myConcatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
        return ForwardLabel(concatenationCost(res, v, forwardLabel, backwardLabel)..., forwardLabel.std_res, backwardLabel.last)
    end
end




# CUSTOM, user-dependent

function initStateForward(res::CustomResource)
    return (ForwardState(0.0,fill(false, N)), 0.0) #Este primeiro 0.0 é o valor inicial do recurso customized q. O segundo 0.0 é o custo do label (cumulative, por exemplo)
end

function initStateBackward(res::CustomResource)
    return (BackwardState(0.0,fill(false, N)), 0.0) #Este primeiro 0.0 é o valor inicial do recurso customized q. O segundo 0.0 é o custo do label (cumulative, por exemplo)
end

function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
    #return (CustomState(label.custom_res.load,label.custom_res.visited), 0.0)
    
    load_ = label.state.load + res.d[a[1],a[2]]
    visited_ = copy(label.state.visited)

    if load_ > res.Q + 1e-5
        return (ForwardState(load_,visited_), Inf) #violation of capacity constraint
    end

    if 2 <= a[2] <= 1 + res.n
        visited_[a[2] - 1] = true
    end

    if 1 + res.n < a[2]
        if visited_[a[2] - 1 - res.n] == false
            return (ForwardState(load_,visited_), Inf) #visiting a delivery without visiting the corresponding pickup
        end
        visited_[a[2] - 1 - res.n] = false
    end

    if a[2] == 1 && load_ > 1e-5
        return (ForwardState(load_,visited_), Inf) #not empty vehicle visiting the depot
    end

    if a[2] == 1
        for i=1:N
            if visited_[i]
                return (ForwardState(load_,visited_), Inf)
            end
        end
    end

    return (ForwardState(load_,visited_), 0.0)

end

function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
    
    #return (CustomState(label.state.load,label.state.visited), 0.0)
    
    load_ = label.state.load - res.d[a[1],a[2]]
    visited_ = copy(label.state.visited)

    if load_ > res.Q + 1e-5 || load_ < -1e-5
        return (BackwardState(load_,visited_), Inf) #violation of capacity constraint
    end

    if 1 + res.n < a[2]
        visited_[a[2] - 1 - res.n] = true
    end

    if 2 <= a[2] <= 1 + res.n
        if visited_[a[2] - 1] == false
            return (BackwardState(load_,visited_), Inf) #visiting a pickup without visiting the corresponding delivery
        end
        visited_[a[2] - 1] = false
    end

    if a[2] == 1 && load_ > 1e-5
        return (BackwardState(load_,visited_), Inf) #not empty vehicle visiting the depot
    end

    if a[2] == 1
        for i=1:N
            if visited_[i]
                return (BackwardState(load_,visited_), Inf)
            end
        end
    end


    return (BackwardState(load_,visited_), 0.0)
end

function concatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    #return (CustomState(0.0, fill(false, length(forwardLabel.state.visited))), 0.0)
    v_ = forwardLabel.last
    #@show v_
    if v_ == 0 #concantenando no depot
        return (ForwardState(0.0, fill(false, length(forwardLabel.state.visited))), 0.0)
    end 
    #@show v_
    pickup = true
    if v_ > N
        pickup = false
        v_ = v_ - N
    end
    for i=1:N
        if i != v_
            if forwardLabel.state.visited[i] != backwardLabel.state.visited[i]
                return (ForwardState(0.0, fill(false, length(forwardLabel.state.visited))), Inf)
            end
        else
            if pickup
                if !forwardLabel.state.visited[i] || backwardLabel.state.visited[i]
                    return (ForwardState(0.0, fill(false, length(forwardLabel.state.visited))), Inf)
                end
            else
                if forwardLabel.state.visited[i] || !backwardLabel.state.visited[i]
                    return (ForwardState(0.0, fill(false, length(forwardLabel.state.visited))), Inf)
                end
            end
        end
    end

    return (ForwardState(0.0, fill(false, length(forwardLabel.state.visited))), 0.0)
    
end

# STANDARD, user-independent

# label: 2 -> 0, a = (2, 3)

function extendAlongArc(res::StandardResource, label::ForwardLabel, a::Tuple{Int, Int}) #Não mexer
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
