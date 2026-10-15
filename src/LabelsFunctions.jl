"""Supertype of the user-defined custom resource."""
abstract type AbstractResource end
"""Supertype of the resource bundle (`Resources`)."""
abstract type AbstractResources end
"""Supertype of labels."""
abstract type Label end

"""
Built-in bounded accumulator with time-window semantics: `d[i, j]` is consumed along arc `(i, j)`,
`lb[j]`/`ub[j]` bound the accumulated value at vertex `j`; waiting is free and exceeding `ub` is
accumulated as a violation (warp). `ID` (1 or 2) selects the label slot it uses.
"""
struct StandardResource{ID}
    d::Matrix{Float64}
    lb::Vector{Float64}
    ub::Vector{Float64}
end

"""State of a standard resource: accumulated quantity `q` and accumulated violation `stdWarp`."""
struct StandardState
    q::Float64
    stdWarp::Float64
end

"""
Summary of a route prefix ending at vertex `last`: custom `state` and `cost`, plus the two standard states.
The forward label at a vertex includes that vertex.
"""
struct ForwardLabel{S}
    state::S
    cost::Float64
    std1State::StandardState
    std2State::StandardState
    last::Int
end

"""
Summary of a route suffix starting after vertex `last`: it describes what comes AFTER `last` and not
`last` itself, so `forward(x) + backward(x)` counts `x` once.
"""
struct BackwardLabel{S}
    state::S
    cost::Float64
    std1State::StandardState
    std2State::StandardState
    last::Int
end

"""Initial forward state and cost of a custom resource, as `(state, cost)`. Implemented by the user."""
initStateForward(res::AbstractResource) = error("not implemented")
"""Initial backward state and cost of a custom resource, as `(state, cost)`. Implemented by the user."""
initStateBackward(res::AbstractResource) = error("not implemented")

"""Whether the problem is symmetric (see the tutorial). Defined by the user; global to the process."""
isSymmetric() = error("not implemented")
"""Whether the custom resource contributes cost to the objective. Defined by the user; global to the process."""
isCostResource() = error("not implemented")

@inline function getState(label::ForwardLabel, ::Val{1})
    return label.std1State
end

@inline function getState(label::BackwardLabel, ::Val{1})
    return label.std1State
end

@inline function getState(label::ForwardLabel, ::Val{2})
    return label.std2State
end

@inline function getState(label::BackwardLabel, ::Val{2})
    return label.std2State
end

"""
Extend a forward label along arc `a = (i, j)` (matrix indices). Users add methods of `extendAlongArc` for
their custom resource; this one implements the standard resources (wait until `lb`, violation above `ub`).
"""
function extendAlongArc(res::StandardResource{ID}, label::ForwardLabel, a::Tuple{Int, Int}) where ID
    state = getState(label, Val(ID))

    qNew = max(min(state.q + res.d[a...], res.ub[a[2]]), res.lb[a[2]])
    warpNew = state.stdWarp + max(state.q + res.d[a...] - res.ub[a[2]], 0.0)
    return (StandardState(qNew, warpNew))
end

"""
Backward extension of a standard resource. The arc arrives as `(next, cur)` and is flipped to the travel
direction `(cur, next)`; the latest allowed value is propagated backwards.
"""
function extendAlongArc(res::StandardResource{ID}, label::BackwardLabel, a::Tuple{Int, Int}) where ID
    a = (a[2], a[1])
    state = getState(label, Val(ID))

    qNew = min(state.q - res.d[a...], res.ub[a[1]])
    warpNew = state.stdWarp
    if qNew < res.lb[a[1]]
        warpNew += res.lb[a[1]] - qNew
        qNew = res.lb[a[1]]
    end
    return (StandardState(qNew, warpNew))
end

"""
Join the forward and the backward label at the same vertex; the violation grows when the forward
arrival is later than the backward limit.
"""
function concatenationCost(res::StandardResource{ID}, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel) where ID
    fwState = getState(forwardLabel, Val(ID))
    bwState = getState(backwardLabel, Val(ID))
    qStd = min(fwState.q, bwState.q)
    qWarp = max(fwState.q - bwState.q, 0) + (fwState.stdWarp + bwState.stdWarp)
    return StandardState(qStd, qWarp)
end

"""Initial forward label: custom state plus empty standard states."""
function myInitStateForward(res::AbstractResource)
    state, cost = initStateForward(res)
    return ForwardLabel(state, cost, StandardState(0.0, 0.0), StandardState(0.0, 0.0), 0)
end

"""Initial backward label: custom state plus unbounded standard states."""
function myInitStateBackward(res::AbstractResource)
    state, cost = initStateBackward(res)
    return BackwardLabel(state, cost, StandardState(Inf, 0.0), StandardState(Inf, 0.0), 0)
end

"""Extend only the custom part of a forward label (standard states are kept)."""
function myExtendAlongArc(res::AbstractResource, label::ForwardLabel, a::Tuple{Int, Int})
    state, cost = extendAlongArc(res, label, a)
    return ForwardLabel(state, cost, label.std1State, label.std2State, a[2] - 1)
end

"""Extend only the custom part of a backward label (standard states are kept)."""
function myExtendAlongArc(res::AbstractResource, label::BackwardLabel, a::Tuple{Int, Int})
    state, cost = extendAlongArc(res, label, a)
    return BackwardLabel(state, cost, label.std1State, label.std2State, a[2] - 1)
end

"""Concatenate only the custom part of a forward and a backward label."""
function myConcatenationCost(res::AbstractResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    state, cost = concatenationCost(res, v, forwardLabel, backwardLabel)
    return ForwardLabel(state, cost, forwardLabel.std1State, forwardLabel.std2State, backwardLabel.last)
end

"""Extend a forward label with the custom resource and both standard resources."""
function myExtendAlongArc(res::AbstractResources, label::ForwardLabel, a::Tuple{Int, Int})
    state, cost = extendAlongArc(res.customResource, label, a)
    std1 = extendAlongArc(res.stdResource1, label, a)
    std2 = extendAlongArc(res.stdResource2, label, a)
    return ForwardLabel(state, cost, std1, std2, a[2] - 1)
end

"""Extend a backward label with the custom resource and both standard resources."""
function myExtendAlongArc(res::AbstractResources, label::BackwardLabel, a::Tuple{Int, Int})
    state, cost = extendAlongArc(res.customResource, label, a)
    std1 = extendAlongArc(res.stdResource1, label, a)
    std2 = extendAlongArc(res.stdResource2, label, a)
    return BackwardLabel(state, cost, std1, std2, a[2] - 1)
end

"""Concatenate a forward and a backward label with the custom resource and both standard resources."""
function myConcatenationCost(res::AbstractResources, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    state, cost = concatenationCost(res.customResource, v, forwardLabel, backwardLabel)
    std1 = concatenationCost(res.stdResource1, v, forwardLabel, backwardLabel)
    std2 = concatenationCost(res.stdResource2, v, forwardLabel, backwardLabel)
    return ForwardLabel(state, cost, std1, std2, backwardLabel.last)
end
