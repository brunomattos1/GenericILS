################ user-defined (GenericILS as a package) ################
using GenericILS
import GenericILS: isSymmetric, isCostResource, initStateForward, initStateBackward,
    extendAlongArc, concatenationCost, AbstractResource, ForwardLabel, BackwardLabel

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

    # copy of the original FastBitSet32
    newbs = FastBitSet32(copy(bs.data))

    # Calcula bloco e bit
    block = (x - 1) ÷ 32 + 1
    pos   = (x - 1) % 32
    mask  = UInt32(1) << pos

    # set the bit in the copy
    newbs.data[block] |= mask

    return newbs
end

function has(bs::FastBitSet32, x::Int)
    if x == 0
        return false
    end
    block = (x - 1) ÷ 32 + 1       # block index (1-based)
    pos   = (x - 1) % 32           # bit position inside the block
    @assert block <= length(bs.data) "Elemento x maior que o tamanho do FastBitSet32"
    mask = UInt32(1) << pos
    return (bs.data[block] & mask) != 0
end

function Base.union(a::FastBitSet32, b::FastBitSet32)
    nblocks = length(a.data)
    result = FastBitSet32(fill(UInt32(0), nblocks))
    @inbounds for i in 1:nblocks
        result.data[i] = a.data[i] | b.data[i]
    end
    return result
end

function Base.intersect(a::FastBitSet32, b::FastBitSet32)
    nblocks = length(a.data)
    result = FastBitSet32(fill(UInt32(0), nblocks))
    @inbounds for i in 1:nblocks
        result.data[i] = a.data[i] & b.data[i]
    end
    return result
end

function hasIntersection(a::FastBitSet32, b::FastBitSet32)
    @inbounds for i in eachindex(a.data)
        if (a.data[i] & b.data[i]) != 0
            return true
        end
    end
    return false
end

function hasIntersection(a::BitSet, b::BitSet)
    if length(a ∩ b) > 0
        return true
    end
    return false
end



struct CustomResource <: AbstractResource
    t::Matrix{Float64} # time cost
    u::Vector{Int}     # upper window (due dates)
    r::Vector{Int}     # release dates
    q::Matrix{Int}     # demands
    Q::Float64         # capacity
    inc::Vector{FastBitSet32} # incompatible customers (indexed by vertex + 1)
end

GenericILS.isSymmetric() = false

GenericILS.isCostResource() = false

struct ForwardState
    T::Float64
    U::Float64
    RD::Float64
    cap::Float64
    inc::FastBitSet32   # customers incompatible with the ones already visited
    inVeh::FastBitSet32 # customers already visited
end

struct BackwardState
    T::Float64
    U::Float64
    RD::Float64
    cap::Float64
    inc::FastBitSet32
    inVeh::FastBitSet32
end

function initStateForward(res::CustomResource)
    return (ForwardState(0.0, Inf, 0.0, 0.0, FastBitSet32(100), FastBitSet32(100)), 0.0)
end

function initStateBackward(res::CustomResource)
    return (BackwardState(0.0, Inf, 0.0, 0.0, FastBitSet32(100), FastBitSet32(100)), 0.0)
end

function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
    T′ = label.state.T + res.t[a...]
    U′ = min(label.state.U, res.u[a[2]] - T′)
    RD′ = max(label.state.RD, res.r[a[2]])
    cap′ = label.state.cap + res.q[1, a[2]]

    inc′ = Base.union(label.state.inc, res.inc[a[2]])
    inVeh′ = label.state.inVeh
    if a[2] - 1 != 0
        inVeh′ = add(label.state.inVeh, a[2] - 1)
    end
    state = ForwardState(T′, U′, RD′, cap′, inc′, inVeh′)

    if label.cost == Inf
        return (state, Inf)
    end
    if T′ > res.u[a[2]] + 1e-12 || U′ < max(0, RD′) - 1e-12 || cap′ > res.Q + 1e-12 || has(label.state.inc, a[2] - 1)
        return (state, Inf)
    end
    return (state, 0.0)
end

function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
    T′ = 0.0
    U′ = min(label.state.U - res.t[a...], res.u[a[2]])
    RD′ = max(label.state.RD, res.r[a[2]])
    cap′ = label.state.cap + res.q[1, a[1]]

    inc′ = Base.union(label.state.inc, res.inc[a[2]])
    inVeh′ = label.state.inVeh
    if a[2] - 1 != 0
        inVeh′ = add(label.state.inVeh, a[2] - 1)
    end
    state = BackwardState(T′, U′, RD′, cap′, inc′, inVeh′)

    if label.cost == Inf
        return (state, Inf)
    end
    if U′ < max(0, max(RD′, res.r[a[2]])) - 1e-6 || cap′ > res.Q + 1e-6 || has(label.state.inc, a[2] - 1)
        return (state, Inf)
    end
    return (state, 0.0)
end

function concatenationCost(res::CustomResource, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel)
    if (max(forwardLabel.state.RD, backwardLabel.state.RD) <= min(backwardLabel.state.U - forwardLabel.state.T, forwardLabel.state.U)) &&
        (forwardLabel.state.cap + backwardLabel.state.cap <= res.Q) &&
        (!hasIntersection(forwardLabel.state.inc, backwardLabel.state.inVeh)) && (!hasIntersection(backwardLabel.state.inc, forwardLabel.state.inVeh))
        return (ForwardState(0.0, 0.0, 0.0, 0.0, FastBitSet32(1), FastBitSet32(1)), 0.0)
    else
        return (ForwardState(0.0, 0.0, 0.0, 0.0, FastBitSet32(1), FastBitSet32(1)), Inf)
    end
end
