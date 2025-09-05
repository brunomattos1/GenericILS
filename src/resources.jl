const DEBUG_MODE = false


struct Resource
    d::Matrix{Float64}
    Q::Float64
end

@static if DEBUG_MODE
    struct CapacityState
        q::Float64
        cost::Float64
        path::Vector{Int}
        last::Int
    end
    struct Label
        cost::Float64
        cap_res::CapacityState
    end
    Base.copy(state::CapacityState) = CapacityState(copy(state.q), copy(state.cost), copy(state.path), copy(state.last))
else
    struct CapacityState
        q::Float64
    end
    struct ForwardLabel
        cap_res::CapacityState
        cost::Float64
    end
    struct BackwardLabel
        cap_res::CapacityState
        cost::Float64
    end
    Base.copy(state::CapacityState) = CapacityState(state.q, state.cost)

end
# Base.copy(state::CapacityState) = CapacityState(copy(state.q), copy(state.cost), copy(state.path), copy(state.last))
# Base.copy(state::CapacityState) = CapacityState(state.q, state.cost)

function isSymmetric()
    return true
end

function initStateForward()
    if DEBUG_MODE
        return CapacityState(0.0, 0.0, [0], 0)
    else
        return ForwardLabel(CapacityState(0.0), 0.0)
    end
end

function initStateBackward()
    if DEBUG_MODE
        return CapacityState(0.0, 0.0, [0], 0)
    else
        return BackwardLabel(CapacityState(0.0), 0.0)
    end
end

function extendAlongArc(res::Resource, label::ForwardLabel, a::Tuple{Int, Int})
    if DEBUG_MODE
        state.q += res.d[a...]
        append!(state.path, a[2] - 1)
        state.last = a[2] - 1
        if state.q > res.Q + 1e-5
            state.cost = Inf
            return state
        else
            state.cost = 0
            return state
        end
    else
        # q = state.q + res.d[a...]
        q = label.cap_res.q + res.d[a...]
        if q > res.Q + 1e-5
            return ForwardLabel(CapacityState(q), Inf)
        else
            return ForwardLabel(CapacityState(q), 0.0)
        end
    end
end

function extendAlongArc(res::Resource, label::BackwardLabel, a::Tuple{Int, Int})
    if DEBUG_MODE
        state.q += res.d[a...]
        append!(state.path, a[2] - 1)
        state.last = a[2] - 1
        if state.q > res.Q + 1e-5
            state.cost = Inf
            return state
        else
            state.cost = 0
            return state
        end
    else
        # q = state.q + res.d[a...]
        q = label.cap_res.q + res.d[a...]
        if q > res.Q + 1e-5
            return BackwardLabel(CapacityState(q), Inf)
        else
            return BackwardLabel(CapacityState(q), 0.0)
        end
    end
end

function concatenationCost(res::Resource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    if DEBUG_MODE
        if state1.q + state2.q > res.Q + 1e-5
            newState = CapacityState(state1.q + state2.q, Inf, vcat(state1.path, reverse(state2.path)), state2.last)
            return newState
        else
            newState = CapacityState(state1.q + state2.q, 0.0, vcat(state1.path, reverse(state2.path)), state2.last)
            return newState
        end
    else
        if forwardLabel.cap_res.q + backwardLabel.cap_res.q > res.Q + 1e-5
            newState = ForwardLabel(CapacityState(forwardLabel.cap_res.q + backwardLabel.cap_res.q), Inf)
            return newState
        else
            newState = ForwardLabel(CapacityState(forwardLabel.cap_res.q + backwardLabel.cap_res.q), 0.0)
            return newState
        end
    end
end