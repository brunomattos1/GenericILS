# Custom resources written in C++ (MetaSolver format) used through loadCppResource. The examples are in test/cpp:
#   cvrp          the capacity of the CVRP in the arc step (demand as an arc parameter)
#   cvrpVertex    the same CVRP written in the vertex step (demand as a vertex parameter)
#   distance      a cost resource: the distance of the arcs and a cost for every vertex that is entered
# They are compared with hand-written Julia resources. The tests are skipped when the machine has no C++ compiler.

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

if GenericILS.CppResources.cppCompiler() === nothing
    @warn "no C++ compiler found (install g++ or clang++, or set CXX): the C++ resource tests are skipped"
else
@testset "C++ resources" begin
    @testset "reading the structs of the header" begin
        fields = GenericILS.CppResources.hppStructFields
        header = """
        struct A {
            double S, T;          // two fields in one declaration
            int k = 3;            /* with an initializer */
            A() { S = T = 0.0; }
            A(double s, double t) { S = s; T = t; }
            double f() const { return S; }
            bool flag;
        };
        struct Empty { };
        """
        @test fields(header, "A") == [:S => Float64, :T => Float64, :k => Cint, :flag => Bool]
        @test isempty(fields(header, "Empty"))
        @test_throws ErrorException fields("struct P { double* x; };", "P")
        @test_throws ErrorException fields("struct P { std::string s; };", "P")
        @test_throws ErrorException fields(header, "Missing")
    end

    @testset "the capacity in the arc step is the capacity of the hand-written resource" begin
        dist, demands, Q = randomInstance(12; seed = 4, Q = 20.0)
        d = arcDemands(demands)
        native = CustomResource(d, Q)
        cpp = cppCapacityArc(d, Q)
        @test cpp isa CppResource
        @test !isCostResource(cpp) && !isSymmetric(cpp)
        @test checkSameLabels(native, cpp, MersenneTwister(1), 12) > 1000
        # the states of the header are generated as Julia structs with its field names
        @test fieldnames(typeof(GenericILS.myInitStateForward(cpp).state)) == (:q,)
        # loading it again reuses the library and the generated types
        again = cppCapacityArc(d, Q)
        @test typeof(again) == typeof(cpp)
        cppFile, hppFile = cppExample("cvrp")
        lib = GenericILS.CppResources.compileCppResource(cppFile, hppFile)
        stamp = mtime(lib)
        @test GenericILS.CppResources.compileCppResource(cppFile, hppFile) == lib && mtime(lib) == stamp

        # and the search finds exactly the same solutions
        a = capacitySolver((demands, Q) -> CustomResource(arcDemands(demands), Q))
        b = capacitySolver((demands, Q) -> cppCapacityArc(arcDemands(demands), Q))
        quiet(() -> solve!(a))
        quiet(() -> solve!(b))
        @test a.bestFeasSol.cost < Inf
        @test a.bestFeasSol.cost == b.bestFeasSol.cost
        @test [r.visits for r in a.bestFeasSol.routes] == [r.visits for r in b.bestFeasSol.routes]
        checkConsistency(b, b.bestFeasSol)
    end

    @testset "the capacity in the vertex step: vertex parameters and extendToVertex in C++" begin
        dist, demands, Q = randomInstance(12; seed = 4, Q = 20.0)
        native = CustomResource(arcDemands(demands), Q)
        cpp = cppCapacityVertex(demands, Q)
        @test checkSameLabels(native, cpp, MersenneTwister(2), 12) > 1000

        a = capacitySolver((demands, Q) -> CustomResource(arcDemands(demands), Q))
        b = capacitySolver((demands, Q) -> cppCapacityVertex(demands, Q))
        quiet(() -> solve!(a))
        quiet(() -> solve!(b))
        @test a.bestFeasSol.cost == b.bestFeasSol.cost
        @test [r.visits for r in a.bestFeasSol.routes] == [r.visits for r in b.bestFeasSol.routes]
    end

    @testset "a cost resource: the flags come from the C++ and the cost is the route cost" begin
        rng = MersenneTwister(5)
        n = 10
        dist, demands, Q = randomInstance(n; seed = 5, Q = 30.0)
        vertexCost = vcat(2.0, Float64.(rand(rng, 1:6, n)))            # the depot is paid when the route ends
        cpp = cppDistance(dist, vertexCost)
        @test isCostResource(cpp) && !isSymmetric(cpp)
        @test !isCostResource()                                        # no global flag: the resource decides

        routeCost(visits) = sum(dist[visits[k]+1, visits[k+1]+1] for k in 1:length(visits)-1) + sum(vertexCost[visits[2:end] .+ 1])
        for visits in routesOfCustomers(rng, n, 100)
            f, b = labelsForward(cpp, visits), labelsBackward(cpp, visits)
            len = length(visits)
            @test f[end].cost ≈ routeCost(visits)
            for x in 1:len
                @test GenericILS.myConcatenationCost(cpp, visits[x], f[x], b[len-x+1]).cost ≈ routeCost(visits)
            end
        end

        # the whole search: the cost of the solver is the cost the resource defines
        d = arcDemands(demands)
        res = Resources(cpp,
            StandardResource{1}(d, zeros(n + 1), fill(Q, n + 1)),
            StandardResource{2}(zeros(n + 1, n + 1), zeros(n + 1), fill(Inf, n + 1)))
        solver = Solver(res = res, data = ProblemData(plainVertices(n), zeros(n + 1, n + 1), ceil(Int, sum(demands) / Q) + 3),
            neighborhoods = NEIGHBORHOODS, algorithm = nilsAlgorithm(stop = NILS.ByIterMax(6))(res))
        setSeed!(solver, MersenneTwister(2))
        @test !canPruneByDist(solver)
        quiet(() -> solve!(solver))
        best = solver.bestFeasSol
        routes = [r.visits for r in best.routes if length(r.visits) > 2]
        @test best.cost < Inf
        @test sort(vcat([v[2:end-1] for v in routes]...)) == 1:n
        @test all(v -> sum(demands[v[2:end-1] .+ 1]) <= Q + 1e-6, routes)
        @test best.cost ≈ sum(routeCost(v) for v in routes)
        @test best.totalLabelCost ≈ best.cost          # all the cost is in the resource (zero cost matrix)
        checkConsistency(solver, best)
    end

    @testset "states that own heap memory: a std::vector in the state" begin
        rng = MersenneTwister(6)
        dist, demands, Q = randomInstance(12; seed = 4, Q = 25.0)
        colors = colorsOf(length(demands), 4)
        ref = ColorsRef(demands, colors, Q)
        vec = cppColors("colorsVector", demands, colors, Q)
        @test typeof(GenericILS.myInitStateForward(vec).state) == GenericILS.CppResources.CppState
        compared, infeasible = checkSameCosts(ref, vec, rng, 12)
        @test compared > 1000 && 0 < infeasible < compared

        # the search finds the same solution as with the Julia resource
        builder(demands, Q) = ColorsRef(demands, colorsOf(length(demands), 4), Q)
        cppBuilder(dir) = (demands, Q) -> cppColors(dir, demands, colorsOf(length(demands), 4), Q)
        checkSameSolutions(builder, cppBuilder("colorsVector"))
    end

    @testset "plain state that the header parser does not read: a std::bitset in the state" begin
        rng = MersenneTwister(7)
        dist, demands, Q = randomInstance(12; seed = 4, Q = 25.0)
        colors = colorsOf(length(demands), 4)
        bits = cppColors("colorsBitset", demands, colors, Q)
        @test typeof(GenericILS.myInitStateForward(bits).state) <: GenericILS.CppResources.OpaqueState
        compared, infeasible = checkSameCosts(ColorsRef(demands, colors, Q), bits, rng, 12)
        @test compared > 1000 && 0 < infeasible < compared

        # the same example with its states forced into the C++ heap
        owned = cppColors("colorsBitset", demands, colors, Q; ownedStates = true)
        @test typeof(GenericILS.myInitStateForward(owned).state) == GenericILS.CppResources.CppState
        @test checkSameCosts(ColorsRef(demands, colors, Q), owned, MersenneTwister(7), 12) == (compared, infeasible)

        builder(demands, Q) = ColorsRef(demands, colorsOf(length(demands), 4), Q)
        cppBuilder(dir; kwargs...) = (demands, Q) -> cppColors(dir, demands, colorsOf(length(demands), 4), Q; kwargs...)
        checkSameSolutions(builder, cppBuilder("colorsBitset"), cppBuilder("colorsBitset"; ownedStates = true))
    end

    @testset "the examples with plain states also work with the states in the C++ heap" begin
        dist, demands, Q = randomInstance(12; seed = 4, Q = 20.0)
        d = arcDemands(demands)
        owned = loadCppResource(cppExample("cvrp")...; nv = size(d, 1), arcParams = (i, j) -> (d = d[i, j],), constParams = (Q = Q,),
            ownedStates = true)
        @test typeof(GenericILS.myInitStateForward(owned).state) == GenericILS.CppResources.CppState
        @test checkSameLabels(CustomResource(d, Q), owned, MersenneTwister(8), 12; states = false) > 1000

        # a resource with cost: the cost of the label is the same
        vertexCost = vcat(2.0, Float64.(rand(MersenneTwister(9), 1:6, 12)))
        plain = cppDistance(dist, vertexCost)
        heap = loadCppResource(cppExample("distance")...; nv = size(dist, 1), arcParams = (i, j) -> (w = dist[i, j],),
            vertexParams = v -> (c = vertexCost[v],), ownedStates = true)
        compared, infeasible = checkSameCosts(plain, heap, MersenneTwister(9), 12)
        @test compared > 1000 && infeasible == 0
    end

    @testset "the states in the C++ heap are freed with their labels" begin
        live = Libdl.dlsym(Libdl.dlopen(GenericILS.CppResources.compileCppResource(cppExample("colorsVector")...)), :colors_liveStates)
        liveStates() = ccall(live, Clong, ())
        "Collect garbage until at most `limit` states are alive."
        function collectUntil(limit)
            for _ in 1:40
                liveStates() <= limit && break
                GC.gc()
                yield()
            end
            return liveStates() <= limit
        end
        dist, demands, Q = randomInstance(12; seed = 4, Q = 25.0)
        res = cppColors("colorsVector", demands, colorsOf(length(demands), 4), Q)

        # the objects are made in functions, so that nothing of the test keeps them reachable
        @noinline function makeLabels()
            labels = labelsForward(res, [0, 1, 2, 3, 4, 5, 0])
            copied = deepcopy(labels)
            return length(labels), all(a.state === b.state for (a, b) in zip(labels, copied)), liveStates()
        end
        @noinline function solveOnce()
            solver = capacitySolver((demands, Q) -> cppColors("colorsVector", demands, colorsOf(length(demands), 4), Q))
            quiet(() -> solve!(solver))
            return solver.bestFeasSol.cost < Inf, liveStates()
        end

        collectUntil(0)
        base = liveStates()
        count, shared, during = makeLabels()
        @test during >= base + count                       # one C++ state per label
        @test shared                                       # deepcopy shares the state: no second owner of the same pointer
        @test collectUntil(base)

        found, during = solveOnce()
        @test found
        @test during > base                                # a search makes many states
        @test collectUntil(base)                           # and none is left when it is dropped
        @test GenericILS.CppResources.LIVE_STATES[] >= 0
    end

    @testset "parameters: plain data, as in the MetaSolver" begin
        d = arcDemands(randomInstance(5; seed = 1)[2])
        cppFile, hppFile = cppExample("cvrp")
        # an isbits Julia value with the layout of the C++ struct instead of a NamedTuple
        viaStruct = loadCppResource(cppFile, hppFile; nv = size(d, 1), arcParams = (i, j) -> CvrpArcLayout(d[i, j]), constParams = (Q = 10.0,))
        viaNamed = loadCppResource(cppFile, hppFile; nv = size(d, 1), arcParams = (i, j) -> (d = d[i, j],), constParams = (Q = 10.0,))
        @test checkSameLabels(viaNamed, viaStruct, MersenneTwister(1), 5) > 100
        @test_throws ArgumentError loadCppResource(cppFile, hppFile; nv = 6, arcParams = (i, j) -> 1.0f0, constParams = (Q = 1.0,))   # wrong size

        # a vector in a parameter struct cannot cross by value
        mktempdir() do dir
            header = replace(read(hppFile, String), "\r\n" => "\n")
            withVector = replace(header, "struct CustomResVertexParameters\n{\n};" => "struct CustomResVertexParameters\n{\n    std::vector<double> v;\n};")
            @test withVector != header
            write(joinpath(dir, "rcsp_custom_res_impl.hpp"), withVector)
            cp(cppFile, joinpath(dir, "rcsp_custom_res_impl.cpp"))
            @test_throws ArgumentError loadCppResource(joinpath(dir, "rcsp_custom_res_impl.cpp"), joinpath(dir, "rcsp_custom_res_impl.hpp");
                nv = 6, arcParams = (i, j) -> (d = 0.0,), vertexParams = v -> (;), constParams = (Q = 1.0,))
        end
    end

    @testset "freeing the C++ object" begin
        d = arcDemands(randomInstance(5; seed = 1)[2])
        res = cppCapacityArc(d, 10.0)
        label = GenericILS.myInitStateForward(res)
        @test res.owner.handle != C_NULL
        freeCppResource!(res)
        @test res.owner.handle == C_NULL
        freeCppResource!(res)                                          # harmless
        @test_throws ArgumentError GenericILS.myInitStateForward(res)
        @test_throws ArgumentError GenericILS.myExtendAlongArc(res, label, (1, 2))
    end

    @testset "the garbage collector releases the C++ object" begin
        d = arcDemands(randomInstance(5; seed = 1)[2])
        @noinline function makeAndDrop(k)
            for _ in 1:k
                GenericILS.myInitStateForward(cppCapacityArc(d, 10.0))
            end
        end
        before = GenericILS.CppResources.RELEASED[]
        makeAndDrop(6)
        for _ in 1:30
            GenericILS.CppResources.RELEASED[] - before >= 6 && break
            GC.gc()
            yield()
        end
        @test GenericILS.CppResources.RELEASED[] - before >= 6
    end

    @testset "errors" begin
        d = arcDemands(randomInstance(5; seed = 1)[2])
        cppFile, hppFile = cppExample("cvrp")
        @test_throws ArgumentError loadCppResource(cppFile, hppFile; nv = 6, constParams = (Q = 1.0,))              # no arc parameters
        @test_throws ArgumentError loadCppResource(cppFile, hppFile; nv = 6, arcParams = (i, j) -> (d = 0.0,))      # no constant Q
        @test_throws ArgumentError loadCppResource(cppFile, hppFile; nv = 6, arcParams = (i, j) -> (x = 0.0,), constParams = (Q = 1.0,))
        @test_throws ArgumentError loadCppResource(joinpath(CPP_EXAMPLES, "nowhere.cpp"), hppFile; nv = 6, constParams = (Q = 1.0,))
        @test_throws ErrorException loadCppResource(cppFile, hppFile; nv = 6, arcParams = (i, j) -> (d = 0.0,),
            constParams = (Q = 1.0,), compiler = "no-such-compiler-for-genericils")
        mktempdir() do dir                                                # a file that does not compile
            broken = joinpath(dir, "rcsp_custom_res_impl.cpp")
            write(broken, read(cppFile, String) * "\nthis is not C++;\n")
            cp(hppFile, joinpath(dir, "rcsp_custom_res_impl.hpp"))
            redirect_stderr(devnull) do
                @test_throws ErrorException loadCppResource(broken, joinpath(dir, "rcsp_custom_res_impl.hpp");
                    nv = 6, arcParams = (i, j) -> (d = 0.0,), constParams = (Q = 1.0,))
            end
        end
    end
end
end
