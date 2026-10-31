using Random

# Vertices of the tests of optional visits. Groups (id: members, `*` marks the optional ones, quota, sameRoute):
#   1: 1 2 3     quota 2           any routes (a choice between regular vertices)
#   2: 4 5*      quota 1, sameRoute  an optional extra that must go along with its group
#   3: 6*        quota 0           a plain optional customer
#   4: 7         quota 1, sameRoute  nothing to keep together
#   5: 8 9*      quota 1           any routes
optionalVertices() = [
    Vertex(1, Group(1, 2, false)), Vertex(2, Group(1, 2, false)), Vertex(3, Group(1, 2, false)),
    Vertex(4, Group(2, 1, true)), Vertex(5, Group(2, 1, true); isOptional = true),
    optionalVertex(6, 3),
    Vertex(7, Group(4, 1, true)),
    Vertex(8, Group(5, 1, false)), Vertex(9, Group(5, 1, false); isOptional = true),
]

optionalData(maxRoutes = 4) = ProblemData(optionalVertices(), zeros(10, 10), maxRoutes)

@testset "Optional vertices" begin
    @testset "Vertex, optionalVertex and ProblemData" begin
        g = Group(1, 1, false)
        @test !Vertex(1, g).isOptional
        @test Vertex(1, g; isOptional = true).isOptional
        @test Vertex(1, g, true).isOptional
        # vertices are still identified by their id
        @test Vertex(3, g) == Vertex(3, g; isOptional = true)
        @test length(Set([Vertex(1, g), Vertex(1, g; isOptional = true)])) == 1

        v = optionalVertex(5)
        @test v.isOptional && v.group == Group(5, 0, false)
        @test optionalVertex(5, 2).group == Group(2, 0, false)
        @test optionalVertex(5, 2; sameRoute = true).group == Group(2, 0, true)

        data = optionalData()
        @test data.isOptional == [false, false, false, false, true, true, false, false, true]
        @test data.optionalIds == [5, 6, 9]
        @test data.nOptional == [0, 1, 1, 0, 1]
        @test data.members == [[1, 2, 3], [4, 5], [6], [7], [8, 9]]
        @test data.regularMembers == [[1, 2, 3], [4], Int[], [7], [8]]
        @test data.hasOptional
        @test !ProblemData(plainVertices(3), zeros(4, 4), 2).hasOptional
        @test isempty(ProblemData().optionalIds)

        # a group has to stay in one route when it is sameRoute and can have several visits
        @test data.locked == [false, true, false, false, false]
        @test data.hasLockedGroups
        @test GenericILS.mustStayInRoute(Group(1, 1, true), 1)         # one needed plus an optional one
        @test !GenericILS.mustStayInRoute(Group(1, 1, true), 0)
        @test !GenericILS.mustStayInRoute(Group(1, 0, true), 1)        # at most one visit
        @test !GenericILS.mustStayInRoute(Group(1, 2, false), 1)
        @test !ProblemData([optionalVertex(1, 1; sameRoute = true)], zeros(2, 2), 1).hasLockedGroups

        # there is something to choose among regular vertices only in the first group
        @test data.hasExchangeableGroups
        @test !ProblemData([optionalVertex(1), Vertex(2, Group(2, 1, false))], zeros(3, 3), 1).hasExchangeableGroups
    end

    @testset "invalid groups with optional vertices are rejected" begin
        build(vs) = ProblemData(vs, zeros(5, 5), 2)
        # the quota counts regular vertices only: an optional one alone needs no visit
        @test_throws ArgumentError build([Vertex(1, Group(1, 1, false); isOptional = true)])
        # a group with regular vertices needs at least one visit, and no more than its regular vertices
        @test_throws ArgumentError build([Vertex(1, Group(1, 0, false)), optionalVertex(2, 1)])
        @test_throws ArgumentError build([Vertex(1, Group(1, 2, false)), Vertex(2, Group(1, 2, false); isOptional = true)])
        @test build([Vertex(1, Group(1, 1, false)), Vertex(2, Group(1, 1, false); isOptional = true)]) isa ProblemData
        # all optional: no quota
        @test build([optionalVertex(1, 1), Vertex(2, Group(1, 0, false); isOptional = true)]) isa ProblemData
    end

    @testset "groupViolations does not count the optional vertices in the quota" begin
        data = optionalData()
        ok = [[0, 1, 2, 4, 7, 8, 0]]                                   # quotas met; optional vertices are not visited
        @test groupViolations(data, ok) == String[]
        # visiting optional vertices changes nothing about the quotas
        @test groupViolations(data, [[0, 1, 2, 4, 5, 6, 7, 8, 9, 0]]) == String[]
        @test groupViolations(data, [[0, 1, 3, 4, 5, 7, 8, 0], [0, 6, 9, 0]]) == String[]
        # the optional vertex of a group that stays in one route must go along with it
        @test any(contains("group 2 is split"), groupViolations(data, [[0, 1, 2, 4, 7, 8, 0], [0, 5, 0]]))
        # the quota of the regular vertices is still enforced, however many optional ones are visited
        @test any(contains("group 1 has 1"), groupViolations(data, [[0, 1, 4, 7, 8, 5, 6, 9, 0]]))
        @test any(contains("group 5 has 0"), groupViolations(data, [[0, 1, 2, 4, 7, 9, 0]]))
        @test any(contains("group 2 has 0"), groupViolations(data, [[0, 1, 2, 5, 7, 8, 0]]))
        # an optional vertex is still visited at most once
        @test any(contains("more than once"), groupViolations(data, [[0, 1, 2, 4, 7, 8, 6, 0], [0, 6, 0]]))
    end

    @testset "canLeaveRoute counts the visits a group has in the route" begin
        data = optionalData()
        # group 2 = {4, 5*} stays in one route: leaving the route needs all the visits it has there
        @test !GenericILS.canLeaveRoute(data, [0, 4, 5, 0], 2, 1)      # 4 would leave 5 behind
        @test !GenericILS.canLeaveRoute(data, [0, 4, 5, 0], 3, 1)      # and 5 would leave 4 behind
        @test  GenericILS.canLeaveRoute(data, [0, 4, 5, 0], 2, 2)      # both together
        @test  GenericILS.canLeaveRoute(data, [0, 4, 0], 2, 1)         # the optional vertex is not visited: 4 is alone
        @test !GenericILS.canLeaveRoute(data, [0, 5, 4, 0], 2, 1)
        @test  GenericILS.canLeaveRoute(data, [0, 1, 4, 7, 5, 0], 2, 1)    # vertex 1: group 1 may be anywhere
        @test  GenericILS.canLeaveRoute(data, [0, 1, 4, 7, 5, 0], 3, 3)    # 4, 7, 5: the group goes whole, 7 is free
        @test !GenericILS.canLeaveRoute(data, [0, 1, 4, 7, 5, 0], 3, 2)    # 4, 7 leaves 5
        @test  GenericILS.canLeaveRoute(data, [0, 6, 9, 0], 2, 1)          # optional vertices of groups that do not stay together
        # a group of regular vertices only keeps comparing with its quota
        regular = ProblemData(groupedVertices([(1:3, 3, true)]), zeros(4, 4), 1)
        @test !GenericILS.canLeaveRoute(regular, [0, 1, 2, 3, 0], 2, 2)
        @test  GenericILS.canLeaveRoute(regular, [0, 1, 2, 3, 0], 2, 3)
    end

    @testset "safeCuts! follows the visits a group has in the route" begin
        data = optionalData()
        safe, seen = Bool[], zeros(Int, length(data.groups))
        GenericILS.safeCuts!(safe, seen, data, [0, 4, 7, 5, 0])
        @test safe == [true, false, false, true, true]                 # 4 .. 5 are cut-free
        @test all(iszero, seen)
        GenericILS.safeCuts!(safe, seen, data, [0, 4, 7, 0])           # 5 is not visited: 4 is alone
        @test safe == [true, true, true, true]
        @test all(iszero, seen)
        GenericILS.safeCuts!(safe, seen, data, [0, 5, 1, 4, 2, 0])
        @test safe == [true, false, false, true, true, true]
        @test all(iszero, seen)
        # no group stays together: every cut is fine
        free = ProblemData(plainVertices(3), zeros(4, 4), 1)
        GenericILS.safeCuts!(safe, zeros(Int, 3), free, [0, 1, 2, 3, 0])
        @test all(safe)
    end

    # Reward for visiting a vertex, as a discount on every arc that enters it (the same thing a resource that
    # returns `-profit` would do): the objective of a route is its distance minus the profits of its vertices.
    function reward!(solver, profits)
        for (id, p) in profits
            solver.data.costMatrix[:, id + 1] .-= p
        end
        return solver
    end

    @testset "construction: optional vertices are inserted only when they pay off" begin
        for seed in 1:8
            # nothing rewards the optional vertices: they stay out and the quotas are met
            solver, = buildCVRP(nilsAlgorithm(); nCust = 9, vertices = optionalVertices(), instanceSeed = seed)
            setSeed!(solver, MersenneTwister(seed))
            sol = constructSol!(solver)
            @test groupViolations(solver.data, sol) == String[]
            @test length(customersOf(sol)) == 5
            @test !any(v -> solver.data.isOptional[v], customersOf(sol))
            checkConsistency(solver, sol)

            # a profit that beats any detour brings the optional vertices in, wherever the group has to be
            solver, = buildCVRP(nilsAlgorithm(); nCust = 9, vertices = optionalVertices(), instanceSeed = seed)
            reward!(solver, [5 => 1000.0, 6 => 1000.0, 9 => 1000.0])
            setSeed!(solver, MersenneTwister(seed))
            sol = constructSol!(solver)
            @test length(customersOf(sol)) == 8                          # 5 regular and the 3 rewarded ones
            @test all(v -> v in customersOf(sol), [5, 6, 9])
            @test groupViolations(solver.data, sol) == String[]       # 5 went along with 4
            checkConsistency(solver, sol)

            # only the rewarded one is inserted
            solver, = buildCVRP(nilsAlgorithm(); nCust = 9, vertices = optionalVertices(), instanceSeed = seed)
            reward!(solver, [6 => 1000.0])
            setSeed!(solver, MersenneTwister(seed))
            sol = constructSol!(solver)
            @test 6 in customersOf(sol) && !(5 in customersOf(sol)) && !(9 in customersOf(sol))
            @test length(customersOf(sol)) == 6
            @test groupViolations(solver.data, sol) == String[]
        end
    end

    @testset "construction: an optional vertex never takes the place of a regular one" begin
        # Every arc costs 100, except those of vertex 5 (optional), which cost 55: inserting it between two stops
        # adds 10, while inserting any regular vertex adds 100. It is the cheapest candidate, yet it does not pay off.
        for seed in 1:4
            solver, = buildCVRP(nilsAlgorithm(); nCust = 9, vertices = optionalVertices(), instanceSeed = seed)
            solver.data.costMatrix .= 100.0
            solver.data.costMatrix[6, :] .= 55.0
            solver.data.costMatrix[:, 6] .= 55.0
            for i in 1:10
                solver.data.costMatrix[i, i] = 0.0
            end
            # two routes only: the seeds do not cover the regular vertices, the insertion phase has to place the rest
            setData!(solver, ProblemData(solver.data.vertices, solver.data.costMatrix, 2))
            setSeed!(solver, MersenneTwister(seed))
            sol = constructSol!(solver)
            @test 4 in customersOf(sol)
            @test !(5 in customersOf(sol))
            @test length(customersOf(sol)) == 5
            @test groupViolations(solver.data, sol) == String[]
        end
    end

    @testset "construction: all vertices optional, as in an orienteering problem" begin
        vertices = [optionalVertex(i) for i in 1:8]
        # no profit: every route is left empty
        solver, = buildCVRP(nilsAlgorithm(); nCust = 8, vertices = vertices)
        sol = constructSol!(solver)
        @test length(sol.routes) == min(8, solver.data.maxNbRoutes)
        @test all(rt -> rt.visits == [0, 0], sol.routes)
        @test groupViolations(solver.data, sol) == String[]
        checkConsistency(solver, sol)
        # with profit, the routes fill up
        solver, = buildCVRP(nilsAlgorithm(); nCust = 8, Q = 1000.0, vertices = vertices)
        reward!(solver, [i => 1000.0 for i in 1:8])
        sol = constructSol!(solver)
        @test customersOf(sol) == collect(1:8)
        @test groupViolations(solver.data, sol) == String[]
        checkConsistency(solver, sol)
        # a capacity that fits three of them: the penalty keeps the profit from overloading the routes
        solver, demands, Q = buildCVRP(nilsAlgorithm(); nCust = 8, Q = 12.0, vertices = vertices)
        reward!(solver, [i => 15.0 for i in 1:8])
        sol = constructSol!(solver)
        @test length(customersOf(sol)) < 8
        @test sol.totalInfeas == 0
    end

    @testset "construction: a group of optional vertices that stays together" begin
        vertices = vcat([optionalVertex(1, 1; sameRoute = true), Vertex(2, Group(1, 0, true); isOptional = true)],
                        [Vertex(i, Group(i - 1, 1, false)) for i in 3:6])
        for seed in 1:6
            solver, = buildCVRP(nilsAlgorithm(); nCust = 6, Q = 1000.0, vertices = vertices, instanceSeed = seed)
            reward!(solver, [1 => 500.0, 2 => 500.0])
            setSeed!(solver, MersenneTwister(seed))
            sol = constructSol!(solver)
            @test 1 in customersOf(sol) && 2 in customersOf(sol)
            r1 = findfirst(rt -> 1 in rt.visits, sol.routes)
            @test 2 in sol.routes[r1].visits                           # the second one joined the route of the first
            @test groupViolations(solver.data, sol) == String[]
        end
    end

    @testset "construction: empty routes when there are fewer regular vertices than routes" begin
        vertices = vcat([Vertex(1, Group(1, 1, false)), Vertex(2, Group(2, 1, false))], [optionalVertex(i) for i in 3:6])
        solver, = buildCVRP(nilsAlgorithm(); nCust = 6, Q = 1000.0, vertices = vertices)
        reward!(solver, [i => 500.0 for i in 3:6])
        sol = constructSol!(solver)
        @test length(sol.routes) == min(2 + 4, solver.data.maxNbRoutes)
        @test customersOf(sol) == collect(1:6)
        @test groupViolations(solver.data, sol) == String[]
        checkConsistency(solver, sol)
    end

    @testset "construction of a problem without optional vertices is unchanged" begin
        for (nCust, seed) in ((12, 1), (20, 4))
            solver, = buildCVRP(nilsAlgorithm(); nCust = nCust, instanceSeed = seed)
            sol = constructSol!(solver)
            @test customersOf(sol) == collect(1:nCust)
            @test !solver.data.hasOptional
        end
    end

    @testset "evaluating an insertion adds no allocation of its own" begin
        solver, = buildCVRP(nilsAlgorithm(); nCust = 12)
        sol = constructSol!(solver)
        c = sol.routes[2].visits[2]
        ins = GenericILS.Insertion(1, c, 2)
        primitive() = (block = solver.bufferRoute; resize!(block, 1); block[1] = c;
                       GenericILS.infeasArcsInsertionK(solver, sol, 1, block, 2))
        evaluation() = GenericILS.evalBestInsertion(solver, sol, ins)
        primitive(); evaluation()                                       # compile
        # it used to build a one-element vector per call on top of what the label evaluation allocates
        @test (@allocated evaluation()) <= (@allocated primitive())
    end

    # ------------------------------------------------------------------------------------------------
    # Searches
    # ------------------------------------------------------------------------------------------------
    optionalNeighborhoods = (NEIGHBORHOODS..., IntraGroupExchange(), AddOptional(), RemoveOptional())

    "A time-window instance over `optionalVertices()` whose optional vertices are rewarded by `profit` each."
    function optionalSolver(seed; profit = 40.0, neighborhoods = optionalNeighborhoods)
        solver, = buildTW(nilsAlgorithm(); nCust = 9, Q = 30.0, horizon = 40.0, instanceSeed = seed,
                          vertices = optionalVertices(), neighborhoods = neighborhoods)
        reward!(solver, [id => profit for id in solver.data.optionalIds])
        setSeed!(solver, MersenneTwister(seed))
        return solver
    end

    "Customers on a line (depot at 0, customer `i` at `pos[i+1]`) with `profit[i]` for entering the optional ones."
    function lineOptionalSolver(pos, vertices, profit = Dict{Int, Float64}(); neighborhoods = optionalNeighborhoods)
        solver, = buildCVRP(nilsAlgorithm(); nCust = length(pos) - 1, Q = 1000.0, vertices = vertices, neighborhoods = neighborhoods)
        solver.data.costMatrix .= [abs(a - b) for a in pos, b in pos]
        reward!(solver, collect(profit))
        return solver
    end

    @testset "IntraGroupExchange only exchanges regular vertices" begin
        for seed in 1:6
            solver = optionalSolver(seed)
            sol = constructSol!(solver)
            optionals = sort([v for v in customersOf(sol) if solver.data.isOptional[v]])
            visited = length(customersOf(sol))
            for _ in 1:30
                search!(IntraGroupExchange(), solver, sol) || break
            end
            @test sort([v for v in customersOf(sol) if solver.data.isOptional[v]]) == optionals
            @test length(customersOf(sol)) == visited
            @test groupViolations(solver.data, sol) == String[]
            checkConsistency(solver, sol)
        end
        # nothing to choose among: the neighborhood leaves the solution alone
        vertices = [Vertex(1, Group(1, 1, false)), Vertex(2, Group(1, 1, false); isOptional = true), Vertex(3, Group(2, 1, false))]
        solver, = buildCVRP(nilsAlgorithm(); nCust = 3, vertices = vertices, neighborhoods = optionalNeighborhoods)
        @test !solver.data.hasExchangeableGroups
        sol = constructSol!(solver)
        @test !search!(IntraGroupExchange(), solver, sol)
    end

    @testset "IntraGroupExchange leaves a visited optional vertex to the neighborhoods that handle it" begin
        # group {1, 2, 3*} needs one regular visit: 1 is visited, 2 is not. The optional vertex 3 is far and its profit (100)
        # is below the 160 it costs, so replacing it by 2 would shorten the route; that is a job for RemoveOptional.
        pos = [0.0, 10.0, 11.0, 90.0, 12.0]
        vertices = [Vertex(1, Group(1, 1, false)), Vertex(2, Group(1, 1, false)), Vertex(3, Group(1, 1, false); isOptional = true),
                    Vertex(4, Group(2, 1, false))]
        solver = lineOptionalSolver(pos, vertices, Dict(3 => 100.0))
        sol = makeSolution(solver, [[0, 1, 3, 4, 0]])
        @test !search!(IntraGroupExchange(), solver, sol)
        @test sol.routes[1].visits == [0, 1, 3, 4, 0]
        @test search!(RemoveOptional(), solver, sol)
        @test sol.routes[1].visits == [0, 1, 4, 0]
    end

    @testset "the moves that change the visits mark their route as modified" begin
        # the other neighborhoods cache what they evaluated for a route until it is marked as modified
        pos = [0.0, 10.0, 20.0, 15.0]
        vertices = [Vertex(1, Group(1, 1, false)), Vertex(2, Group(2, 1, false)), optionalVertex(3)]
        solver = lineOptionalSolver(pos, vertices, Dict(3 => 5.0))
        sol = makeSolution(solver, [[0, 1, 2, 0], [0, 0]])
        sol.timeStamp = 7
        GenericILS.apply!(AddOptional(), solver, sol, GenericILS.BestOptional(0.0, sol.dist, 1, 3, 3))
        @test sol.routes[1].visits == [0, 1, 3, 2, 0]
        @test sol.routes[1].lastModif == 7 && sol.routes[2].lastModif == 0
        sol.timeStamp = 9
        GenericILS.apply!(RemoveOptional(), solver, sol, GenericILS.BestOptional(0.0, sol.dist, 1, 3, 3))
        @test sol.routes[1].visits == [0, 1, 2, 0]
        @test sol.routes[1].lastModif == 9 && sol.routes[2].lastModif == 0
    end

    @testset "AddOptional inserts an optional vertex where it pays off most" begin
        pos = [0.0, 10.0, 20.0, 15.0, 100.0]       # customers 1, 2 regular; 3 on the way (15), 4 far (100)
        vertices = [Vertex(1, Group(1, 1, false)), Vertex(2, Group(2, 1, false)), optionalVertex(3), optionalVertex(4)]
        # 3 is on the way between 1 and 2: it costs nothing to visit, so any profit pays
        solver = lineOptionalSolver(pos, vertices, Dict(3 => 5.0))
        sol = makeSolution(solver, [[0, 1, 2, 0]])
        @test search!(AddOptional(), solver, sol)
        @test sol.routes[1].visits == [0, 1, 3, 2, 0]
        checkConsistency(solver, sol)
        @test !search!(AddOptional(), solver, sol)                      # a local optimum
        # without profit there is no reason to visit it
        solver = lineOptionalSolver(pos, vertices)
        sol = makeSolution(solver, [[0, 1, 2, 0]])
        @test !search!(AddOptional(), solver, sol)
        @test sol.routes[1].visits == [0, 1, 2, 0]
        # vertex 4 is far: the detour is 160 (20 -> 100 -> 0 instead of 20 -> 0), so 150 is not enough but 170 is
        solver = lineOptionalSolver(pos, vertices, Dict(4 => 150.0))
        sol = makeSolution(solver, [[0, 1, 2, 0]])
        @test !search!(AddOptional(), solver, sol)
        solver = lineOptionalSolver(pos, vertices, Dict(4 => 170.0))
        sol = makeSolution(solver, [[0, 1, 2, 0]])
        @test search!(AddOptional(), solver, sol)
        @test sol.routes[1].visits in ([0, 1, 4, 2, 0], [0, 1, 2, 4, 0])     # both positions cost the same
        @test sol.cost ≈ 200.0 - 170.0                                  # the route is 200 long
        checkConsistency(solver, sol)
    end

    @testset "AddOptional picks the route and the position" begin
        # two routes on the two sides of the depot; the optional vertex sits next to the second one
        pos = [0.0, 30.0, -30.0, -29.0]
        vertices = [Vertex(1, Group(1, 1, false)), Vertex(2, Group(2, 1, false)), optionalVertex(3)]
        solver = lineOptionalSolver(pos, vertices, Dict(3 => 10.0))
        sol = makeSolution(solver, [[0, 1, 0], [0, 2, 0]])
        @test search!(AddOptional(), solver, sol)
        @test 3 in sol.routes[2].visits && !(3 in sol.routes[1].visits)
        @test sol.routes[2].visits == [0, 3, 2, 0]
        @test sol.routes[1].visits == [0, 1, 0]
        checkConsistency(solver, sol)
    end

    @testset "AddOptional keeps an optional vertex with the group that stays in one route" begin
        # group {2, 3*} stays in one route; 3 (at -30) sits right next to route 2 (vertex 4 at -29), but its group
        # (vertex 2 at 40) is in route 1
        pos = [0.0, 30.0, 40.0, -30.0, -29.0]
        vertices = [Vertex(1, Group(1, 1, false)), Vertex(2, Group(2, 1, true)), Vertex(3, Group(2, 1, true); isOptional = true),
                    Vertex(4, Group(3, 1, false))]
        solver = lineOptionalSolver(pos, vertices, Dict(3 => 1000.0))
        sol = makeSolution(solver, [[0, 1, 2, 0], [0, 4, 0]])           # route 1: vertices 1 and 2; route 2: vertex 4
        @test solver.data.locked[2]
        @test search!(AddOptional(), solver, sol)
        @test 3 in sol.routes[1].visits                                    # with its group, although route 2 is closer
        @test !(3 in sol.routes[2].visits)
        @test groupViolations(solver.data, sol) == String[]
        checkConsistency(solver, sol)
    end

    @testset "RemoveOptional drops the optional vertices that do not pay off" begin
        pos = [0.0, 10.0, 20.0, 80.0]               # 3 is optional and far: it costs 2 * (80 - 20) = 120 on the way back
        vertices = [Vertex(1, Group(1, 1, false)), Vertex(2, Group(2, 1, false)), optionalVertex(3)]
        solver = lineOptionalSolver(pos, vertices, Dict(3 => 100.0))
        sol = makeSolution(solver, [[0, 1, 2, 3, 0]])
        @test sol.cost ≈ 160.0 - 100.0
        @test search!(RemoveOptional(), solver, sol)
        @test sol.routes[1].visits == [0, 1, 2, 0]
        @test sol.cost ≈ 40.0
        checkConsistency(solver, sol)
        @test !search!(RemoveOptional(), solver, sol)
        # with enough profit it stays
        solver = lineOptionalSolver(pos, vertices, Dict(3 => 130.0))
        sol = makeSolution(solver, [[0, 1, 2, 3, 0]])
        @test !search!(RemoveOptional(), solver, sol)
        @test sol.routes[1].visits == [0, 1, 2, 3, 0]
        # regular vertices are never removed, whatever they cost
        solver = lineOptionalSolver(pos, [Vertex(1, Group(1, 1, false)), Vertex(2, Group(2, 1, false)), Vertex(3, Group(3, 1, false))])
        sol = makeSolution(solver, [[0, 1, 2, 3, 0]])
        @test !search!(RemoveOptional(), solver, sol)
        @test sol.routes[1].visits == [0, 1, 2, 3, 0]
    end

    "A random insertion of an unvisited optional vertex or removal of a visited one, with the routes it must produce."
    function randomOptionalMove(rng, solver, sol, add::Bool)
        data = solver.data
        visited = GenericILS.visitedVertices(data, sol)
        if add
            cands = [u for u in data.optionalIds if !visited[u]]
            isempty(cands) && return nothing
            u = rand(rng, cands)
            g = data.groupOf[u]
            groupRoute = GenericILS.routeOfGroups(data, sol)
            r = data.locked[g] && groupRoute[g] != 0 ? groupRoute[g] : rand(rng, eachindex(sol.routes))
            v = sol.routes[r].visits
            j = rand(rng, 2:length(v))
            dist = GenericILS.bestInsertionCost(sol.dist, data.costMatrix, v, u, j)
            return GenericILS.BestOptional(0.0, dist, r, j, u), Dict(r => vcat(v[1:j-1], u, v[j:end]))
        end
        cands = [(r, i) for r in eachindex(sol.routes) for i in 2:length(sol.routes[r].visits)-1 if data.isOptional[sol.routes[r].visits[i]]]
        isempty(cands) && return nothing
        r, i = rand(rng, cands)
        v = sol.routes[r].visits
        dist = GenericILS.removeOptionalCost(sol.dist, data.costMatrix, v, i)
        return GenericILS.BestOptional(0.0, dist, r, i, v[i]), Dict(r => vcat(v[1:i-1], v[i+1:end]))
    end

    @testset "apply! of $(typeof(neigh)) matches the move on plain vectors" for (neigh, add) in ((AddOptional(), true), (RemoveOptional(), false))
        applied = 0
        for seed in 1:8
            solver = optionalSolver(seed)
            rng = MersenneTwister(seed)
            for trial in 1:12
                sol = constructSol!(solver)
                if add                                                  # make room: drop some optional vertices first
                    for _ in 1:2
                        m = randomOptionalMove(rng, solver, sol, false)
                        m === nothing || GenericILS.apply!(RemoveOptional(), solver, sol, m[1])
                    end
                end
                m = randomOptionalMove(rng, solver, sol, add)
                m === nothing && continue
                move, expected = m
                visitedBefore = length(customersOf(sol))
                GenericILS.apply!(neigh, solver, sol, move)
                for (r, visits) in expected
                    @test sol.routes[r].visits == visits
                end
                @test length(customersOf(sol)) == visitedBefore + (add ? 1 : -1)
                @test groupViolations(solver.data, sol) == String[]
                @test sol.dist ≈ manualCost(sol, solver.data.costMatrix)
                checkConsistency(solver, sol)
                applied += 1
            end
        end
        @test applied > 30
    end

    @testset "the predicted objective of an insertion or removal equals the applied one on feasible routes" begin
        checked = 0
        for seed in 1:8
            solver = optionalSolver(seed)
            rng = MersenneTwister(seed)
            for trial in 1:12, add in (true, false)
                sol = constructSol!(solver)
                m = randomOptionalMove(rng, solver, sol, add)
                m === nothing && continue
                move, = m
                predicted = add ? GenericILS.evalAddOptional(solver, sol, move.route, move.idx, move.customer, move.dist) :
                                  GenericILS.evalRemoveOptional(solver, sol, move.route, move.idx, move.dist)
                GenericILS.apply!(add ? AddOptional() : RemoveOptional(), solver, sol, move)
                if sol.routes[move.route].infeas == 0
                    @test predicted ≈ sol.cost
                    checked += 1
                else
                    @test predicted >= sol.cost - 1e-6               # the count of violations is an upper bound
                end
            end
        end
        @test checked > 30
    end

    @testset "$(typeof(neigh)) keeps the solution valid, consistent and improves it" for neigh in (AddOptional(), RemoveOptional())
        improvedAny = false
        for seed in 1:8
            solver = optionalSolver(seed; profit = 25.0 + 5seed)
            rng = MersenneTwister(seed)
            sol = constructSol!(solver)
            for _ in 1:3                                                # perturb: drop some visited optional vertices...
                m = randomOptionalMove(rng, solver, sol, false)
                m === nothing || GenericILS.apply!(RemoveOptional(), solver, sol, m[1])
            end
            for _ in 1:2                                                # ...and add some back at random
                m = randomOptionalMove(rng, solver, sol, true)
                m === nothing || GenericILS.apply!(AddOptional(), solver, sol, m[1])
            end
            regular = sort([v for v in customersOf(sol) if !solver.data.isOptional[v]])
            for _ in 1:40
                cost0 = sol.cost
                flag = search!(neigh, solver, sol)
                @test sol.cost <= cost0 + 1e-6
                flag && @test sol.cost < cost0 - 1e-6
                flag && (improvedAny = true)
                @test sort([v for v in customersOf(sol) if !solver.data.isOptional[v]]) == regular
                @test groupViolations(solver.data, sol) == String[]
                flag || break
            end
            checkConsistency(solver, sol)
            cost1 = sol.cost
            @test !search!(neigh, solver, sol)
            @test sol.cost ≈ cost1
        end
        @test improvedAny
    end

    @testset "RVND with every neighborhood keeps optional visits valid, and the cache hides nothing" begin
        improved = 0
        for seed in 1:10
            solver = optionalSolver(seed; profit = 20.0 + 6seed)
            sol = constructSol!(solver)
            cost0 = sol.cost
            quiet(() -> GenericILS.NILS.RVND!(solver, sol))
            @test sol.cost <= cost0 + 1e-6
            improved += sol.cost < cost0 - 1e-6
            # which regular vertices of the first group are visited may change, but not how many
            @test count(v -> !solver.data.isOptional[v], customersOf(sol)) == 5
            @test groupViolations(solver.data, sol) == String[]
            checkConsistency(solver, sol)
            fill!(sol.lastEval, 0)                                       # a fresh evaluation must agree
            @test !any(search!(n, solver, sol) for n in solver.neighborhoods)
        end
        @test improved >= 3
    end

    # ------------------------------------------------------------------------------------------------
    # Perturbation
    # ------------------------------------------------------------------------------------------------
    "Same instance as `optionalSolver` with the algorithm of `algorithmFor`."
    function optionalPerturbSolver(seed, algorithmFor)
        solver, = buildTW(algorithmFor; nCust = 9, Q = 30.0, horizon = 40.0, instanceSeed = seed,
                          vertices = optionalVertices(), neighborhoods = optionalNeighborhoods)
        reward!(solver, [id => 40.0 for id in solver.data.optionalIds])
        setSeed!(solver, MersenneTwister(seed))
        return solver
    end

    @testset "randomToggleOptional! flips the visit of one optional vertex" begin
        # nothing to flip without optional vertices
        solver, = buildCVRP(nilsAlgorithm(); nCust = 8)
        sol = constructSol!(solver)
        routes = [copy(rt.visits) for rt in sol.routes]
        @test !GenericILS.randomToggleOptional!(solver, sol)
        @test [rt.visits for rt in sol.routes] == routes

        added = removed = 0
        for seed in 1:6
            solver = optionalPerturbSolver(seed, nilsAlgorithm())
            sol = constructSol!(solver)
            regular = sort([v for v in customersOf(sol) if !solver.data.isOptional[v]])
            for _ in 1:60
                before = length(customersOf(sol))
                @test GenericILS.randomToggleOptional!(solver, sol)
                after = length(customersOf(sol))
                @test abs(after - before) == 1
                added += after > before
                removed += after < before
                @test sort([v for v in customersOf(sol) if !solver.data.isOptional[v]]) == regular
                @test groupViolations(solver.data, sol) == String[]
            end
            checkConsistency(solver, sol)
        end
        @test added > 20 && removed > 20
    end

    @testset "randomToggleOptional! inserts an optional vertex in the route of its group" begin
        # group {4, 5*} stays in one route; nothing rewards 5 here, so it starts out
        for seed in 1:6
            solver, = buildCVRP(nilsAlgorithm(); nCust = 9, vertices = optionalVertices(), instanceSeed = seed)
            setSeed!(solver, MersenneTwister(seed))
            sol = constructSol!(solver)
            @test !(5 in customersOf(sol))
            for _ in 1:80
                GenericILS.randomToggleOptional!(solver, sol)
                @test groupViolations(solver.data, sol) == String[]
            end
            checkConsistency(solver, sol)
        end
    end

    @testset "the perturbations of the algorithms flip an optional visit too" for (name, algorithmFor, perturbs) in (
            ("NILS", nilsAlgorithm(), (GenericILS.NILS.innerPerturb!, GenericILS.NILS.outerPerturb!)),
            ("ILS", ilsAlgorithm(), (GenericILS.ILS.perturb!,)))
        for perturb! in perturbs, seed in 1:4
            solver = optionalPerturbSolver(seed, algorithmFor)
            sol = constructSol!(solver)
            for _ in 1:40
                before = length(customersOf(sol))
                perturb!(solver, sol)
                @test abs(length(customersOf(sol)) - before) == 1       # shifts and swaps keep the count, the flip changes it
                @test groupViolations(solver.data, sol) == String[]
            end
            checkConsistency(solver, sol)
        end
        # without optional vertices the perturbation keeps the number of visits
        solver, = buildCVRP(algorithmFor; nCust = 10)
        sol = constructSol!(solver)
        for perturb! in perturbs
            perturb!(solver, sol)
            @test customersOf(sol) == collect(1:10)
        end
    end

    # ------------------------------------------------------------------------------------------------
    # Set Partitioning
    # ------------------------------------------------------------------------------------------------
    @testset "Set Partitioning selects exactly the best valid combination with optional vertices" begin
        together = vcat([optionalVertex(1, 1; sameRoute = true), Vertex(2, Group(1, 0, true); isOptional = true)],
                        [Vertex(i, Group(i - 1, 1, false)) for i in 3:8])
        specs = [("regular and optional", optionalVertices(), 9),
                 ("all optional", [optionalVertex(i) for i in 1:8], 8),
                 ("optional vertices that stay together", together, 8)]
        feasible = infeasible = 0
        for (name, vertices, n) in specs, maxRoutes in (nothing, 2), keep in (1.0, 0.5), seed in 1:4
            solver, = buildCVRP(nilsAlgorithm(); nCust = n, vertices = vertices, instanceSeed = seed)
            reward!(solver, [id => 25.0 + 5 * (id % 4) for id in solver.data.optionalIds])
            setSeed!(solver, MersenneTwister(seed))
            maxRoutes === nothing || setData!(solver, ProblemData(solver.data.vertices, solver.data.costMatrix, maxRoutes))
            pool = mixedPool(solver, MersenneTwister(seed), n; keep = keep)
            length(pool) <= 13 || (pool = pool[1:13])
            expected = bruteForceOptimum(solver, pool)

            sp, λ, ok = partitioningModel(solver, pool)
            ok && optimize!(sp)
            if expected == Inf
                @test !ok || termination_status(sp) == MOI.INFEASIBLE
                infeasible += 1
            else
                @test ok
                @test termination_status(sp) == MOI.OPTIMAL
                @test objective_value(sp) ≈ expected atol = 1e-6
                chosen = [pool[r] for r in eachindex(pool) if value(λ[r]) > 0.5]
                @test groupViolations(solver.data, chosen) == String[]
                feasible += 1
            end
        end
        @test feasible > 30 && infeasible > 5
    end

    @testset "Set Partitioning never selects two routes of a group of optional vertices that stays together" begin
        vertices = vcat([optionalVertex(1, 1; sameRoute = true), Vertex(2, Group(1, 0, true); isOptional = true)],
                        [Vertex(3, Group(2, 1, false))])
        solver, = buildCVRP(nilsAlgorithm(); nCust = 3, Q = 1000.0, vertices = vertices)
        reward!(solver, [1 => 1000.0, 2 => 1000.0])
        # both together is the best; without that route only one of the two can be taken, although both pay
        pool = [[0, 1, 0], [0, 2, 0], [0, 3, 0], [0, 1, 2, 0]]
        sp, λ, ok = partitioningModel(solver, pool)
        @test ok
        optimize!(sp)
        chosen = [pool[r] for r in eachindex(pool) if value(λ[r]) > 0.5]
        @test groupViolations(solver.data, chosen) == String[]
        @test [0, 1, 2, 0] in chosen
        sp, λ, ok = partitioningModel(solver, pool[1:3])
        optimize!(sp)
        chosen = [pool[r] for r in 1:3 if value(λ[r]) > 0.5]
        @test groupViolations(solver.data, chosen) == String[]
        @test count(r -> r in ([0, 1, 0], [0, 2, 0]), chosen) == 1
        @test [0, 3, 0] in chosen
    end

    @testset "Set Partitioning visits an optional vertex at most once" begin
        solver, = buildCVRP(nilsAlgorithm(); nCust = 2, Q = 1000.0, vertices = [optionalVertex(1), optionalVertex(2)])
        reward!(solver, [1 => 500.0, 2 => 500.0])
        pool = [[0, 1, 2, 0], [0, 1, 0], [0, 2, 0]]
        sp, λ, ok = partitioningModel(solver, pool)
        optimize!(sp)
        chosen = [pool[r] for r in eachindex(pool) if value(λ[r]) > 0.5]
        @test groupViolations(solver.data, chosen) == String[]
        @test sort(vcat([r[2:end-1] for r in chosen]...)) == [1, 2]
        # nothing pays: the empty solution is chosen, no route is forced
        solver, = buildCVRP(nilsAlgorithm(); nCust = 2, Q = 1000.0, vertices = [optionalVertex(1), optionalVertex(2)])
        sp, λ, ok = partitioningModel(solver, pool)
        @test ok
        optimize!(sp)
        @test objective_value(sp) ≈ 0.0 atol = 1e-9
        @test all(r -> value(λ[r]) < 0.5, eachindex(pool))
    end

    @testset "$name finishes with a valid solution that collects the optional vertices that pay" for (name, algorithmFor) in
            (("NILS", nilsAlgorithm()), ("ILS", ilsAlgorithm()))
        for seed in 1:3
            solver, = buildCVRP(algorithmFor; nCust = 9, Q = 100.0, vertices = optionalVertices(), instanceSeed = seed,
                                neighborhoods = optionalNeighborhoods)
            reward!(solver, [id => 600.0 for id in solver.data.optionalIds])
            setSeed!(solver, MersenneTwister(seed))
            quiet(() -> solve!(solver))
            best = solver.bestFeasSol
            @test best.cost < Inf
            @test groupViolations(solver.data, best) == String[]
            @test all(v -> v in vcat([rt.visits for rt in best.routes]...), solver.data.optionalIds)
            @test best.dist ≈ manualCost(best, solver.data.costMatrix)
            @test best.dist ≈ best.cost
            @test solver.statistics.bestFeasCost <= solver.statistics.bestFeasCostBefSP + 1e-6
        end
        # profit too small to matter: no optional vertex is visited
        solver, = buildCVRP(algorithmFor; nCust = 9, Q = 100.0, vertices = optionalVertices(), neighborhoods = optionalNeighborhoods)
        quiet(() -> solve!(solver))
        @test !any(v -> solver.data.isOptional[v], vcat([rt.visits[2:end-1] for rt in solver.bestFeasSol.routes]...))
        @test groupViolations(solver.data, solver.bestFeasSol) == String[]
    end
end
