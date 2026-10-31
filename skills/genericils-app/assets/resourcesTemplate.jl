################ user-defined (GenericILS as a package) ################
# Template: a capacitated problem (CVRP-like) that runs out of the box.
# Search for "CHANGE" to find the parts you must adapt to your problem.
using GenericILS
import GenericILS: isSymmetric, isCostResource, initStateForward, initStateBackward,
    extendAlongArc, concatenationCost, AbstractResource, ForwardLabel, BackwardLabel

# CHANGE: the data your route-feasibility/cost rules need (matrices are (n+1)x(n+1),
# row/column 1 is the depot, matrix index = vertex id + 1).
struct CustomResource <: AbstractResource
    d::Matrix{Float64}   # d[i, j]: demand "picked up" when entering j (0 when j is the depot)
    Q::Float64           # vehicle capacity
end

# CHANGE: true if the route cost has a part beyond the distance matrix that this resource
# computes (then extendAlongArc must return the accumulated custom cost, see references/api.md).
GenericILS.isCostResource() = false
# CHANGE: true only if travelling i->j is equivalent to j->i for ALL your rules.
GenericILS.isSymmetric() = false

# CHANGE: what a partial route must remember. Forward = prefix from the depot,
# backward = suffix up to the depot.
struct ForwardState
    q::Float64
end

struct BackwardState
    q::Float64
end

initStateForward(res::CustomResource) = (ForwardState(0.0), 0.0)
initStateBackward(res::CustomResource) = (BackwardState(0.0), 0.0)

# Return (newState, cost); cost = Inf flags a violation. Arcs are matrix indices (depot = 1):
#  - forward:  a = (i, j), j is appended and the label includes j
#  - backward: a = (next, cur), cur is prepended; the label at a vertex excludes that vertex
# so forward(x) + backward(x) counts x exactly once (see references/api.md).
function extendAlongArc(res::CustomResource, label::ForwardLabel, a::Tuple{Int, Int})
    q = label.state.q + res.d[1, a[2]]          # forward adds the head j (a[2])
    q > res.Q + 1e-5 ? (ForwardState(q), Inf) : (ForwardState(q), 0.0)
end

function extendAlongArc(res::CustomResource, label::BackwardLabel, a::Tuple{Int, Int})
    q = label.state.q + res.d[1, a[1]]          # backward adds the previous start of the suffix (a[1])
    q > res.Q + 1e-5 ? (BackwardState(q), Inf) : (BackwardState(q), 0.0)
end

# OPTIONAL: data that belongs to a vertex (a service time, a profit, a demand) can be applied in
# `extendToVertex` instead (import it from GenericILS). It runs right after extendAlongArc, on the label it
# returned, with v = the matrix index of the vertex that enters the label (a[2] forward, a[1] backward):
#   extendToVertex(res::CustomResource, label::ForwardLabel, v::Int)  = (label.state, label.cost)
#   extendToVertex(res::CustomResource, label::BackwardLabel, v::Int) = (label.state, label.cost)

# Join a forward prefix with a backward suffix. Must agree with extending the whole route.
function concatenationCost(res::CustomResource, v::Int, fwd::ForwardLabel, bwd::BackwardLabel)
    q = fwd.state.q + bwd.state.q
    q > res.Q + 1e-5 ? (ForwardState(q), Inf) : (ForwardState(q), 0.0)
end
