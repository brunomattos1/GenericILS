# Helpers of the tests of the C++ resources (testCppResource.jl) and of the adapter (testVrpSolverAdapter.jl).

using Libdl

const CPP_EXAMPLES = joinpath(@__DIR__, "cpp")

"Paths `(cpp, hpp)` of the example `name` in test/cpp."
cppExample(name) = (joinpath(CPP_EXAMPLES, name, "rcsp_custom_res_impl.cpp"), joinpath(CPP_EXAMPLES, name, "rcsp_custom_res_impl.hpp"))

"The capacity resource of the CVRP in the arc step, for the arc demands `d` and capacity `Q`."
cppCapacityArc(d, Q) = loadCppResource(cppExample("cvrp")...; nv = size(d, 1), arcParams = (i, j) -> (d = d[i, j],), constParams = (Q = Q,))

"The capacity resource of the CVRP in the vertex step, for the demands (by matrix index) and capacity `Q`."
cppCapacityVertex(demands, Q) = loadCppResource(cppExample("cvrpVertex")...; nv = length(demands), vertexParams = v -> (d = demands[v],), constParams = (Q = Q,))

"The cost resource of `distance`: the arc distances plus `vertexCost[v]` for every vertex that is entered."
cppDistance(dist, vertexCost) = loadCppResource(cppExample("distance")...; nv = size(dist, 1),
    arcParams = (i, j) -> (w = dist[i, j],), vertexParams = v -> (c = vertexCost[v],))

import GenericILS: extendToVertex

# ---- the color examples: a route may not have two customers of the same color, and has a capacity ----

"Colors 1..`ncolors` for the customers of an instance with `n` vertices (the depot has none)."
colorsOf(n, ncolors; seed = 1) = vcat(0, rand(MersenneTwister(seed), 1:ncolors, n - 1))

"The resource of the color examples, with the state of a hand-written Julia resource: the colors used are a bit mask."
struct ColorsRef <: AbstractResource
    demand::Vector{Float64}
    color::Vector{Int}
    Q::Float64
end
struct CRFwd
    q::Float64
    used::UInt64
end
struct CRBwd
    q::Float64
    used::UInt64
end
initStateForward(::ColorsRef) = (CRFwd(0.0, UInt64(0)), 0.0)
initStateBackward(::ColorsRef) = (CRBwd(0.0, UInt64(0)), 0.0)
extendAlongArc(::ColorsRef, l::ForwardLabel, ::Tuple{Int,Int}) = (l.state, l.cost)
extendAlongArc(::ColorsRef, l::BackwardLabel, ::Tuple{Int,Int}) = (l.state, l.cost)
function colorsEnter(res::ColorsRef, q, used, v)
    q += res.demand[v]
    bad = q > res.Q + 1e-5
    c = res.color[v]
    if c > 0
        bad |= (used >> c) & 1 == 1
        used |= UInt64(1) << c
    end
    return q, used, bad ? Inf : 0.0
end
function extendToVertex(res::ColorsRef, l::ForwardLabel, v::Int)
    q, used, c = colorsEnter(res, l.state.q, l.state.used, v)
    return (CRFwd(q, used), l.cost + c)
end
function extendToVertex(res::ColorsRef, l::BackwardLabel, v::Int)
    q, used, c = colorsEnter(res, l.state.q, l.state.used, v)
    return (CRBwd(q, used), l.cost + c)
end
function concatenationCost(res::ColorsRef, v::Int, f::ForwardLabel, b::BackwardLabel)
    bad = f.state.q + b.state.q > res.Q + 1e-5 || f.state.used & b.state.used != 0
    return (f.state, f.cost + b.cost + (bad ? Inf : 0.0))
end

"The color example `dir` (`colorsVector`: the used colors in a std::vector; `colorsBitset`: in a std::bitset)."
cppColors(dir, demands, colors, Q; kwargs...) = loadCppResource(cppExample(dir)...; nv = length(demands),
    vertexParams = v -> (d = demands[v], color = colors[v]), constParams = (Q = Q, ncolors = maximum(colors)), kwargs...)

"Arc parameters of the `cvrp` example as a Julia value with the layout of the C++ struct, as the MetaSolver passes them."
struct CvrpArcLayout
    d::Float64
end

"""
Check that `other` gives the same label costs, vertices and concatenations as `ref` on random routes, whatever the
states are. Returns how many concatenations were compared and how many of them are infeasible.
"""
function checkSameCosts(ref, other, rng, nCust; routes = 150)
    compared = infeasible = 0
    for visits in routesOfCustomers(rng, nCust, routes)
        fr, br = labelsForward(ref, visits), labelsBackward(ref, visits)
        fo, bo = labelsForward(other, visits), labelsBackward(other, visits)
        len = length(visits)
        @test [l.cost for l in fr] == [l.cost for l in fo]
        @test [l.cost for l in br] == [l.cost for l in bo]
        @test [l.last for l in fr] == [l.last for l in fo] && [l.last for l in br] == [l.last for l in bo]
        for x in 1:len
            cr = GenericILS.myConcatenationCost(ref, visits[x], fr[x], br[len-x+1])
            co = GenericILS.myConcatenationCost(other, visits[x], fo[x], bo[len-x+1])
            @test cr.cost == co.cost
            compared += 1
            infeasible += cr.cost == Inf
        end
    end
    return compared, infeasible
end

"Solve the same problem with each custom resource builder (`(demands, Q) -> resource`); the solutions must be identical."
function checkSameSolutions(builders...)
    solvers = [capacitySolver(b) for b in builders]
    for solver in solvers
        quiet(() -> solve!(solver))
    end
    first = solvers[1].bestFeasSol
    @test first.cost < Inf
    for solver in solvers[2:end]
        @test solver.bestFeasSol.cost == first.cost
        @test [r.visits for r in solver.bestFeasSol.routes] == [r.visits for r in first.routes]
        # the cost recomputed from the visits, as the states are not comparable
        @test makeSolution(solver, [r.visits for r in solver.bestFeasSol.routes]).cost ≈ solver.bestFeasSol.cost
    end
    return solvers
end

"Check that `other` gives the same labels and concatenations as `native` on random routes; returns how many were compared."
function checkSameLabels(native, other, rng, nCust; routes = 150, states = true)
    compared = 0
    for visits in routesOfCustomers(rng, nCust, routes)
        fn, bn = labelsForward(native, visits), labelsBackward(native, visits)
        fo, bo = labelsForward(other, visits), labelsBackward(other, visits)
        len = length(visits)
        if states
            @test [l.state.q for l in fn] == [l.state.q for l in fo]
            @test [l.state.q for l in bn] == [l.state.q for l in bo]
        end
        @test [l.cost for l in fn] == [l.cost for l in fo]
        @test [l.cost for l in bn] == [l.cost for l in bo]
        @test [l.last for l in fn] == [l.last for l in fo] && [l.last for l in bn] == [l.last for l in bo]
        for x in 1:len
            cn = GenericILS.myConcatenationCost(native, visits[x], fn[x], bn[len-x+1])
            co = GenericILS.myConcatenationCost(other, visits[x], fo[x], bo[len-x+1])
            @test cn.cost == co.cost
            compared += 1
        end
    end
    return compared
end
