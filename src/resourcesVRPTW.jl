const DEBUG_MODE = false

struct Resource
    d::Matrix{Float64}
    Q::Float64
    t::Matrix{Float64}
    early::Matrix{Int}
    late::Matrix{Int}
end

@static if DEBUG_MODE
    # struct CapacityState
    #     q::Float64
    #     cost::Float64
    #     path::Vector{Int}
    #     last::Int
    # end
    # struct Label
    #     cost::Float64
    #     cap_res::CapacityState
    #     time_res::TimeResource
    # end
    # Base.copy(state::CapacityState) = CapacityState(copy(state.q), copy(state.cost), copy(state.path), copy(state.last))
else
    struct CapacityState
        q::Float64
    end
    struct TimeState
        t::Float64
    end
    struct ForwardLabel
        cap_res::CapacityState
        time_res::TimeState
        cost::Float64
    end
    struct BackwardLabel
        cap_res::CapacityState
        time_res::TimeState
        cost::Float64
    end
end


function isSymmetric()
    return false
end

function initStateForward()
    return ForwardLabel(CapacityState(0.0), TimeState(0.0), 0.0)
end

function initStateBackward()
    return BackwardLabel(CapacityState(0.0), TimeState(10000.0), 0.0)
end

function extendAlongArc(res::Resource, label::ForwardLabel, a::Tuple{Int, Int})
    q = label.cap_res.q + res.d[a...]
    t = max(label.time_res.t + res.t[a...], res.early[a...])
    if t > res.late[a...]  + 1e-6 || q > res.Q + 1e-6
        return ForwardLabel(CapacityState(q), TimeState(t), Inf)
    else
        return ForwardLabel(CapacityState(q), TimeState(t), 0.0)
    end
    # if q > res.Q + 1e-6
    #     return ForwardLabel(CapacityState(q), TimeState(0.0), Inf)
    # else
    #     return ForwardLabel(CapacityState(q), TimeState(0.0), 0.0)
    # end

end

function extendAlongArc(res::Resource, label::BackwardLabel, a::Tuple{Int, Int})
    q = label.cap_res.q + res.d[a...]
    t = min(label.time_res.t, res.late[a...]) - res.t[a...]
    if t < res.early[a...] + 1e-6 || q > res.Q + 1e-6
        return BackwardLabel(CapacityState(q), TimeState(t), Inf)
    else
        return BackwardLabel(CapacityState(q), TimeState(t), 0.0)
    end
    # if  q > res.Q + 1e-6
    #     return BackwardLabel(CapacityState(q), TimeState(0.0), Inf)
    # else
    #     return BackwardLabel(CapacityState(q), TimeState(0.0), 0.0)
    # end
end

function concatenationCost(res::Resource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    q = forwardLabel.cap_res.q + backwardLabel.cap_res.q
    t = forwardLabel.time_res.t + backwardLabel.time_res.t
    if q > res.Q + 1e-5 || forwardLabel.time_res.t > backwardLabel.time_res.t#maximum(res.late) - backwardLabel.time_res.t
        newState = ForwardLabel(CapacityState(q), TimeState(t), Inf)
        return newState
    else
        newState = ForwardLabel(CapacityState(q), TimeState(t), 0.0)
        return newState
    end
    # if q > res.Q + 1e-5
    #     newState = ForwardLabel(CapacityState(q), TimeState(0.0), Inf)
    #     return newState
    # else
    #     newState = ForwardLabel(CapacityState(q), TimeState(0.0), 0.0)
    #     return newState
    # end
end