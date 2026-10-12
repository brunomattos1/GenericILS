################ user-defined (GenericILS as a package) ################
using GenericILS
import GenericILS: isSymmetric, isCostResource, initStateForward, initStateBackward,
    extendAlongArc, concatenationCost, AbstractResource, ForwardLabel, BackwardLabel

struct CustomResource <: AbstractResource
    C₀::Matrix{Float64}
    C::Matrix{Float64}
end

GenericILS.isSymmetric() = false

GenericILS.isCostResource() = true

struct ForwardState
    v::Int64 # In the forward (backward), v indicates the first (last) visited vertex;
    cost::Float64
end

struct BackwardState
    v::Int64 # In the forward (backward), v indicates the first (last) visited vertex;
    cost::Float64
end

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
