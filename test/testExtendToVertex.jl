# Resources that put (part of) their rules in the vertex step `extendToVertex`, which runs right after
# `extendAlongArc` on the label it produced.
import GenericILS: extendToVertex

# The capacity of the CVRP, written entirely in the vertex step: the arc does nothing and the vertex adds its demand.
struct VertexCapacity <: AbstractResource
    demand::Vector{Float64}          # by matrix index (id + 1); the depot has none
    Q::Float64
end
struct VCapFwd
    q::Float64
end
struct VCapBwd
    q::Float64
end
initStateForward(::VertexCapacity) = (VCapFwd(0.0), 0.0)
initStateBackward(::VertexCapacity) = (VCapBwd(0.0), 0.0)
extendAlongArc(::VertexCapacity, l::ForwardLabel, ::Tuple{Int,Int}) = (l.state, l.cost)
extendAlongArc(::VertexCapacity, l::BackwardLabel, ::Tuple{Int,Int}) = (l.state, l.cost)
function extendToVertex(res::VertexCapacity, l::ForwardLabel, v::Int)
    q = l.state.q + res.demand[v]
    return (VCapFwd(q), q > res.Q + 1e-5 ? Inf : 0.0)
end
function extendToVertex(res::VertexCapacity, l::BackwardLabel, v::Int)
    q = l.state.q + res.demand[v]
    return (VCapBwd(q), q > res.Q + 1e-5 ? Inf : 0.0)
end
function concatenationCost(res::VertexCapacity, v::Int, f::ForwardLabel, b::BackwardLabel)
    q = f.state.q + b.state.q
    return (VCapFwd(q), q > res.Q + 1e-5 ? Inf : 0.0)
end

# Checks what the vertex step receives: the state after the arc and the vertex that enters the label.
struct VertexRecorder <: AbstractResource end
struct RecFwd
    steps::Int          # vertex steps applied so far
    head::Int           # vertex entered by the last arc
end
struct RecBwd
    steps::Int
    head::Int
end
initStateForward(::VertexRecorder) = (RecFwd(0, 0), 0.0)
initStateBackward(::VertexRecorder) = (RecBwd(0, 0), 0.0)
extendAlongArc(::VertexRecorder, l::ForwardLabel, a::Tuple{Int,Int}) = (RecFwd(l.state.steps, a[2]), l.cost)
extendAlongArc(::VertexRecorder, l::BackwardLabel, a::Tuple{Int,Int}) = (RecBwd(l.state.steps, a[1]), l.cost)
extendToVertex(::VertexRecorder, l::ForwardLabel, v::Int) =
    (RecFwd(l.state.steps + 1, l.state.head), l.state.head == v && l.last == v - 1 ? l.cost : Inf)
extendToVertex(::VertexRecorder, l::BackwardLabel, v::Int) =
    (RecBwd(l.state.steps + 1, l.state.head), l.state.head == v ? l.cost : Inf)
concatenationCost(::VertexRecorder, v::Int, f::ForwardLabel, b::BackwardLabel) =
    (RecFwd(f.state.steps + b.state.steps, 0), f.cost + b.cost)

# A cost per vertex (Inf forbids it). The depot, matrix index 1, is entered only when the route ends.
struct VertexCost <: AbstractResource
    cost::Vector{Float64}
end
struct VCostFwd
    c::Float64
end
struct VCostBwd
    c::Float64
end
initStateForward(::VertexCost) = (VCostFwd(0.0), 0.0)
initStateBackward(::VertexCost) = (VCostBwd(0.0), 0.0)
extendAlongArc(::VertexCost, l::ForwardLabel, ::Tuple{Int,Int}) = (l.state, l.cost)
extendAlongArc(::VertexCost, l::BackwardLabel, ::Tuple{Int,Int}) = (l.state, l.cost)
extendToVertex(res::VertexCost, l::ForwardLabel, v::Int) = (VCostFwd(l.state.c + res.cost[v]), l.cost + res.cost[v])
extendToVertex(res::VertexCost, l::BackwardLabel, v::Int) = (VCostBwd(l.state.c + res.cost[v]), l.cost + res.cost[v])
concatenationCost(::VertexCost, v::Int, f::ForwardLabel, b::BackwardLabel) = (f.state, f.cost + b.cost)

"A solver for a random CVRP whose custom resource is `custom(demands, Q)`."
function capacitySolver(custom; nCust = 14, Q = 30.0, instanceSeed = 3)
    dist, demands, Q = randomInstance(nCust; seed = instanceSeed, Q = Q)
    d = arcDemands(demands)
    n = nCust + 1
    res = Resources(custom(demands, Q),
        StandardResource{1}(d, zeros(n), fill(Q, n)),
        StandardResource{2}(zeros(n, n), zeros(n), fill(Inf, n)))
    solver = Solver(res = res, data = ProblemData(plainVertices(nCust), dist, ceil(Int, sum(demands) / Q) + 3),
        neighborhoods = NEIGHBORHOODS, algorithm = nilsAlgorithm(stop = NILS.ByIterMax(6))(res))
    setSeed!(solver, MersenneTwister(2))
    return solver
end

@testset "extendToVertex" begin
    @testset "a resource that does not define it is not affected" begin
        label = GenericILS.myInitStateForward(CustomResource(zeros(3, 3), 5.0))
        @test extendToVertex(CustomResource(zeros(3, 3), 5.0), label, 2) == (label.state, label.cost)
        back = GenericILS.myInitStateBackward(CustomResource(zeros(3, 3), 5.0))
        @test extendToVertex(CustomResource(zeros(3, 3), 5.0), back, 2) == (back.state, back.cost)
    end

    @testset "the vertex step sees the label after the arc and the vertex that enters it" begin
        rng = MersenneTwister(1)
        res = VertexRecorder()
        for visits in routesOfCustomers(rng, 9, 50)
            f, b = labelsForward(res, visits), labelsBackward(res, visits)
            n = length(visits)
            # one vertex step per arc, none for the source, and never an inconsistent (Inf) vertex or label
            @test [l.state.steps for l in f] == 0:n-1
            @test [l.state.steps for l in b] == 0:n-1
            @test all(l -> l.cost == 0.0, f) && all(l -> l.cost == 0.0, b)
            # every vertex but the source is entered once by a forward label, a backward one and their join
            for x in 1:n
                @test f[x].state.steps + b[n-x+1].state.steps == n - 1
            end
        end
        # the same through the full set of resources of the solver
        solver = capacitySolver((demands, Q) -> VertexRecorder())
        l = GenericILS.myExtendAlongArc(solver.res, GenericILS.myInitStateForward(solver.res.customResource), (1, 4))
        @test l.cost == 0.0 && l.state.steps == 1 && l.last == 3
        l = GenericILS.myExtendAlongArc(solver.res, GenericILS.myInitStateBackward(solver.res.customResource), (1, 4))
        @test l.cost == 0.0 && l.state.steps == 1 && l.last == 3
    end

    @testset "a cost per vertex is paid once, wherever the route is cut" begin
        rng = MersenneTwister(2)
        n = 9
        cost = vcat(7.0, Float64.(rand(rng, 1:20, n)))      # the depot costs when the route ends, never at its start
        res = VertexCost(cost)
        for visits in routesOfCustomers(rng, n, 100)
            total = sum(cost[visits[2:end] .+ 1])
            f, b = labelsForward(res, visits), labelsBackward(res, visits)
            len = length(visits)
            @test f[end].cost ≈ total
            for x in 1:len
                @test GenericILS.myConcatenationCost(res, visits[x], f[x], b[len-x+1]).cost ≈ total
            end
        end
        # Inf from the vertex step forbids the vertex in both directions and in the join
        cost[4] = Inf
        for visits in routesOfCustomers(rng, n, 50)
            f, b = labelsForward(res, visits), labelsBackward(res, visits)
            len = length(visits)
            has = 3 in visits
            @test (f[end].cost == Inf) == has
            @test all(x -> (GenericILS.myConcatenationCost(res, visits[x], f[x], b[len-x+1]).cost == Inf) == has, 1:len)
        end
    end

    @testset "the capacity in the vertex step is the capacity in the arc" begin
        dist, demands, Q = randomInstance(12; seed = 4, Q = 20.0)
        native = CustomResource(arcDemands(demands), Q)
        vertexForm = VertexCapacity(demands, Q)
        rng = MersenneTwister(3)
        compared = 0
        for visits in routesOfCustomers(rng, 12, 200)
            fn, bn = labelsForward(native, visits), labelsBackward(native, visits)
            fv, bv = labelsForward(vertexForm, visits), labelsBackward(vertexForm, visits)
            len = length(visits)
            @test [l.state.q for l in fn] == [l.state.q for l in fv]
            @test [l.state.q for l in bn] == [l.state.q for l in bv]
            @test [l.cost for l in fn] == [l.cost for l in fv]
            @test [l.cost for l in bn] == [l.cost for l in bv]
            @test [l.last for l in fn] == [l.last for l in fv] && [l.last for l in bn] == [l.last for l in bv]
            for x in 1:len
                cn = GenericILS.myConcatenationCost(native, visits[x], fn[x], bn[len-x+1])
                cv = GenericILS.myConcatenationCost(vertexForm, visits[x], fv[x], bv[len-x+1])
                @test cn.cost == cv.cost && cn.state.q == cv.state.q
                compared += 1
            end
        end
        @test compared > 1000

        # and the search finds exactly the same solutions
        a = capacitySolver((demands, Q) -> CustomResource(arcDemands(demands), Q))
        b = capacitySolver((demands, Q) -> VertexCapacity(demands, Q))
        quiet(() -> solve!(a))
        quiet(() -> solve!(b))
        @test a.bestFeasSol.cost < Inf
        @test a.bestFeasSol.cost == b.bestFeasSol.cost
        @test [r.visits for r in a.bestFeasSol.routes] == [r.visits for r in b.bestFeasSol.routes]
        checkConsistency(b, b.bestFeasSol)
    end

    @testset "a cost resource with a profit in the vertex step (isCostResource() == true)" begin
        # Separate process: the resource flags are global, see vertexResourceScript.jl
        script = joinpath(@__DIR__, "vertexResourceScript.jl")
        cmd = `$(Base.julia_cmd()) --project=$(Base.active_project()) $script`
        out = read(cmd, String)
        @test occursin("VERTEX_RESOURCE_OK", out)
    end
end
