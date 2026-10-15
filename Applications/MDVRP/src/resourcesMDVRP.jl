################ user-defined (GenericILS as a package) ################
using GenericILS
import GenericILS: isSymmetric, isCostResource, initStateForward, initStateBackward,
    extendAlongArc, concatenationCost, AbstractResource, ForwardLabel, BackwardLabel

"""
Multi-depot cost resource over a single dummy depot: `C₀[i, j]` is the cheapest depot round trip that serves
customer `i` first and `j` last, `C[i, j]` the customer-to-customer cost.
"""
struct CustomResource <: AbstractResource
    C₀::Matrix{Float64}
    C::Matrix{Float64}
end

GenericILS.isSymmetric() = false

GenericILS.isCostResource() = true

"""First visited vertex `v` and accumulated cost of the prefix."""
struct ForwardState
    v::Int64 # In the forward (backward), v indicates the first (last) visited vertex;
    cost::Float64
end

"""Last visited vertex `v` and accumulated cost of the suffix."""
struct BackwardState
    v::Int64 # In the forward (backward), v indicates the first (last) visited vertex;
    cost::Float64
end

"""Forward state of the empty prefix."""
function initStateForward(res::CustomResource)
    return (ForwardState(0, 0.0), 0.0)
end

"""Backward state of the empty suffix."""
function initStateBackward(res::CustomResource)
    return (BackwardState(0, 0.0), 0.0)
end

"""Forward extension along `a = (i, j)` (matrix indices): appends `j`. The returned cost is the accumulated cost of the partial route."""
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

"""Backward extension along `a = (next, cur)`: prepends `cur` to the suffix that starts at `next`. The returned cost is the accumulated cost of the partial route."""
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

"""Join the forward and the backward label at the same vertex: cost/feasibility of the whole route."""
function concatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    conc = forwardLabel.state.cost + backwardLabel.state.cost
    if forwardLabel.state.v != 0 && backwardLabel.state.v != 0
        conc += res.C₀[forwardLabel.state.v-1,backwardLabel.state.v-1]
    end
    newState = (ForwardState(0,0.0), conc)
    return newState
end
