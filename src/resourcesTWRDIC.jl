const DEBUG_MODE = false


mutable struct FastBitSet32
    data::Vector{UInt32}
end

function FastBitSet32(n::Int)
    nblocks = cld(n, 32)
    FastBitSet32(fill(UInt32(0), nblocks))
end

function add!(bs::FastBitSet32, x::Int)
    block = (x - 1) >>> 5 + 1     # divide por 32
    mask  = UInt32(1) << ((x - 1) & 0x1f)
    bs.data[block] |= mask
    return bs
end

function add(bs::FastBitSet32, x::Int)
    @assert x >= 1 "Elemento x deve ser >= 1"
    nblocks = length(bs.data)
    @assert x <= nblocks*32 "Elemento x maior que o tamanho do FastBitSet32"

    # Cria uma cópia do FastBitSet32 original
    newbs = FastBitSet32(copy(bs.data))

    # Calcula bloco e bit
    block = (x - 1) ÷ 32 + 1
    pos   = (x - 1) % 32
    mask  = UInt32(1) << pos

    # Ativa o bit na cópia
    newbs.data[block] |= mask

    return newbs
end

function has(bs::FastBitSet32, x::Int)
    if x == 0
        return false
    end
    block = (x - 1) ÷ 32 + 1       # índice do bloco (1-based)
    pos   = (x - 1) % 32           # posição do bit dentro do bloco
    @assert block <= length(bs.data) "Elemento x maior que o tamanho do FastBitSet32"
    mask = UInt32(1) << pos
    return (bs.data[block] & mask) != 0
end

function union(a::FastBitSet32, b::FastBitSet32)
    nblocks = length(a.data)
    result = FastBitSet32(fill(UInt32(0), nblocks))
    @inbounds for i in 1:nblocks
        result.data[i] = a.data[i] | b.data[i]
    end
    return result
end

function intersect(a::FastBitSet32, b::FastBitSet32)
    nblocks = length(a.data)
    result = FastBitSet32(fill(UInt32(0), nblocks))
    @inbounds for i in 1:nblocks
        result.data[i] = a.data[i] & b.data[i]
    end
    return result
end

function has_intersection(a::FastBitSet32, b::FastBitSet32)
    @inbounds for i in eachindex(a.data)
        if (a.data[i] & b.data[i]) != 0
            return true
        end
    end
    return false
end

function has_intersection(a::BitSet, b::BitSet)
    if length(a ∩ b) > 0
        return true
    end
    return false
end



struct CustomResource
    t::Matrix{Float64} # time cost
    u::Vector{Int} # upper window
    r::Vector{Int} # release dates
    q::Matrix{Int} # demands
    Q::Float64 # capacity
    inc::Vector{FastBitSet32}
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

function isSymmetric()
    return false
end


@static if DEBUG_MODE
    struct CustomState
        T::Float64
        U::Float64
        RD::Float64
        cap::Float64
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
        T::Float64
        U::Float64
        RD::Float64
        cap::Float64
        inc::FastBitSet32
        inVeh::FastBitSet32
    end

    struct StandardState
        q::Float64
        stdWarp::Float64
    end

    struct ForwardLabel
        custom_res::CustomState
        cost::Float64
        std_res::StandardState
        last::Int
    end

    struct BackwardLabel
        custom_res::CustomState
        cost::Float64
        std_res::StandardState
        last::Int
    end
    function myInitStateForward()
        return ForwardLabel(initStateForward()..., StandardState(0.0, 0.0), 0)
    end

    function myInitStateBackward()
        return BackwardLabel(initStateBackward()..., StandardState(Inf, 0.0), 0)
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

    function myInitStateForward()
        return ForwardLabel(initStateForward()..., StandardState(0.0, 0.0), 0)
    end

    function myInitStateBackward()
        return BackwardLabel(initStateBackward()..., StandardState(Inf, 0.0), 0)
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

function initStateForward()
    return (CustomState(0.0, Inf, 0.0, 0.0, FastBitSet32(100), FastBitSet32(100)), 0.0)
end

function initStateBackward()
    return (CustomState(0.0, Inf, 0.0, 0.0, FastBitSet32(100), FastBitSet32(100)), 0.0)
end


function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
    # a = (i,j) i -> j
    # a = (2, 3) => (3, 4)
    T′ = label.custom_res.T + res.t[a...]
    U′ = min(label.custom_res.U, res.u[a[2]] - T′)
    RD′ = max(label.custom_res.RD, res.r[a[2]])
    cap′ = label.custom_res.cap + res.q[1, a[2]]

    inc′ = union(label.custom_res.inc, res.inc[a[2]])#Base.union(label.custom_res.inc, res.inc[a[2]])
    inVeh′ = label.custom_res.inVeh

    if a[2]-1 != 0
        inVeh′ = add(label.custom_res.inVeh, a[2]-1)
    end
    
    if label.cost == Inf
        state = CustomState(T′, U′, RD′, cap′, inc′, inVeh′)
        return (state, Inf)
    end
    if T′ > res.u[a[2]] + 1e-12 || U′ < max(0, RD′) - 1e-12 || cap′ > res.Q + 1e-12 || has(label.custom_res.inc, a[2]-1)
        state = CustomState(T′, U′, RD′, cap′, inc′, inVeh′)
        return (state, Inf)
    end
    state = CustomState(T′, U′, RD′, cap′, inc′, inVeh′)
    return (state, 0.0)
end

function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
    # a = (i, j)
    # 3 - 0
    T′ = 0.0
    U′ = min(label.custom_res.U - res.t[a...], res.u[a[2]])
    RD′ = max(label.custom_res.RD, res.r[a[2]])
    cap′ = label.custom_res.cap + res.q[1, a[1]]

    inc′ = union(label.custom_res.inc, res.inc[a[2]])#Base.union(label.custom_res.inc, res.inc[a[2]])
    inVeh′ = label.custom_res.inVeh

    if a[2]-1 != 0
        inVeh′ = add(label.custom_res.inVeh, a[2]-1)
    end

    if label.cost == Inf
        state = CustomState(T′, U′, RD′, cap′, inc′, inVeh′)
        return (state, Inf)
    end

    if U′ < max(0, max(RD′, res.r[a[2]])) - 1e-6 || cap′ > res.Q + 1e-6 || has(label.custom_res.inc, a[2]-1)
        state = CustomState(T′, U′, RD′, cap′, inc′, inVeh′)
        return (state, Inf)
    end
    state = CustomState(T′, U′, RD′, cap′, inc′, inVeh′)
    return (state, 0.0)
end

function concatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    if (max(forwardLabel.custom_res.RD, backwardLabel.custom_res.RD) <= min(backwardLabel.custom_res.U - forwardLabel.custom_res.T, forwardLabel.custom_res.U)) && 
        (forwardLabel.custom_res.cap + backwardLabel.custom_res.cap <= res.Q) &&
        (!has_intersection(forwardLabel.custom_res.inc, backwardLabel.custom_res.inVeh)) && (!has_intersection(backwardLabel.custom_res.inc, forwardLabel.custom_res.inVeh))
        return (CustomState(0.0, 0.0, 0.0, 0.0, FastBitSet32(1), FastBitSet32(1)), 0.0)
    else
        return (CustomState(0.0, 0.0, 0.0, 0.0, FastBitSet32(1), FastBitSet32(1)), Inf)
    end
end

# STANDARD, user-independent


function extendAlongArc(res::StandardResource, label::ForwardLabel, a::Tuple{Int, Int})
    q_std = max(label.std_res.q + res.d[a...], res.lb[a[2]])
    if q_std > res.ub[a[2]] + 1e-12
        q_std = res.ub[a[2]]
    end
    q_warp = label.std_res.stdWarp
    if label.std_res.q + res.d[a...] > res.ub[a[2]] + 1e-12
        q_warp += label.std_res.q + res.d[a...] - res.ub[a[2]] - 1e-12
    end
    # q_std = max(min(label.std_res.q + res.d[a...], res.ub[a[2]]), res.lb[a[2]])
    # q_warp = label.std_res.stdWarp + max(label.std_res.q + res.d[a...] - res.ub[a[2]] - 1e-12, 0.0)
    return (StandardState(q_std, q_warp))
end

function extendAlongArc(res::StandardResource, label::BackwardLabel, a::Tuple{Int, Int})
    a = (a[2], a[1])

    q_std = min(label.std_res.q - res.d[a...], res.ub[a[1]])
    q_warp = label.std_res.stdWarp
    if q_std < res.lb[a[1]] - 1e-12
        q_warp += res.lb[a[1]] - q_std
    end
    if q_std < res.lb[a[1]] - 1e-12
        q_std = res.lb[a[1]]
    end
    return (StandardState(q_std, q_warp))
end

function concatenationCost(res::StandardResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    q_std = min(forwardLabel.std_res.q, backwardLabel.std_res.q)
    q_warp = max(forwardLabel.std_res.q - backwardLabel.std_res.q, 0) + (forwardLabel.std_res.stdWarp + backwardLabel.std_res.stdWarp)
    return StandardState(q_std, q_warp)
end
