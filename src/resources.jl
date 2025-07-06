struct CapacityResource
    d::Matrix{Float64}
    Q::Float64
end

mutable struct CapacityState
    q::Float64
    cost::Float64
    path::Vector{Int}
    last::Int
end

Base.copy(state::CapacityState) = CapacityState(copy(state.q), copy(state.cost), copy(state.path), copy(state.last))
# Base.copy(state::CapacityState) = CapacityState(state.q, state.cost)

