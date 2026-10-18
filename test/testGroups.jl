"Vertices from `(ids, numVisits, sameRoute)` specs; group `g` is the `g`-th spec."
groupedVertices(spec) = sort!([Vertex(id, Group(g, N, same)) for (g, (ids, N, same)) in enumerate(spec) for id in ids];
                              by = v -> v.id)

# 12 customers in each of the shapes the groups can take
const GVRP_SPEC    = [(1:3, 1, false), (4:6, 1, false), (7:9, 1, false), (10:12, 1, false)]
const CLUSTER_SPEC = [(1:4, 4, true), (5:8, 4, true), (9:12, 4, true)]
const MIXED_SPEC   = [(1:2, 1, false), (3:6, 2, true), (7:10, 2, false), (11:11, 1, false), (12:12, 1, false)]

"Customers visited by `sol`, sorted."
visitedBy(sol) = customersOf(sol)

@testset "Visit groups" begin
    @testset "Group, Vertex and ProblemData" begin
        @test Group(2, 3, true) == Group(2, 3, true)
        @test Vertex(5, Group(2, 3, true)).group.numVisits == 3

        data = ProblemData(groupedVertices(MIXED_SPEC), zeros(13, 13), 3)
        @test [g.groupId for g in data.groups] == 1:5
        @test data.groups[2] == Group(2, 2, true)
        @test data.members == [[1, 2], [3, 4, 5, 6], [7, 8, 9, 10], [11], [12]]
        @test data.groupOf == [1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 5]

        # a classic VRP is one group per customer, each needing one visit
        plain = ProblemData(plainVertices(4), zeros(5, 5), 2)
        @test all(g -> g.numVisits == 1 && !g.sameRoute, plain.groups)
        @test plain.members == [[1], [2], [3], [4]]
        @test plain.groupOf == 1:4

        empty = ProblemData()
        @test isempty(empty.groups) && isempty(empty.members) && isempty(empty.groupOf)

        # invalid data is rejected when the problem is built
        build(vs) = ProblemData(vs, zeros(13, 13), 3)
        @test_throws ArgumentError build([Vertex(1, Group(1, 1, false)), Vertex(1, Group(2, 1, false))])  # repeated id
        @test_throws ArgumentError build([Vertex(0, Group(1, 1, false))])                                  # id 0 is the depot
        @test_throws ArgumentError build([Vertex(1, Group(1, 1, false)), Vertex(2, Group(3, 1, false))])   # group 2 missing
        @test_throws ArgumentError build([Vertex(1, Group(0, 1, false))])                                  # group ids start at 1
        @test_throws ArgumentError build([Vertex(1, Group(1, 2, false))])                                  # more visits than members
        @test_throws ArgumentError build([Vertex(1, Group(1, 0, false))])                                  # no visits
        @test_throws ArgumentError build([Vertex(1, Group(1, 1, false)), Vertex(2, Group(1, 1, true))])    # members disagree
    end

    @testset "groupViolations" begin
        data = ProblemData(groupedVertices(MIXED_SPEC), zeros(13, 13), 5)
        ok = [[0, 1, 3, 4, 0], [0, 7, 9, 11, 0], [0, 12, 0]]
        @test isempty(groupViolations(data, ok))
        @test isempty(groupViolations(data, [[0, 2, 6, 5, 0], [0, 8, 10, 0], [0, 11, 12, 0]]))

        @test any(contains("group 1 has 0"), groupViolations(data, [[0, 3, 4, 0], [0, 7, 9, 11, 12, 0]]))  # nothing from group 1
        @test any(contains("group 3 has 3"), groupViolations(data, [[0, 1, 3, 4, 0], [0, 7, 8, 9, 11, 12, 0]]))  # one visit too many
        @test any(contains("more than once"), groupViolations(data, [[0, 1, 3, 4, 0], [0, 1, 7, 9, 11, 12, 0]]))
        @test any(contains("group 2 is split"), groupViolations(data, [[0, 1, 3, 0], [0, 4, 7, 9, 11, 12, 0]]))
        @test any(contains("unknown vertex 13"), groupViolations(data, [[0, 1, 3, 4, 13, 0]]))
        # members of a group that may use several routes can be spread freely
        @test isempty(groupViolations(data, [[0, 1, 3, 4, 7, 0], [0, 9, 11, 12, 0]]))
        # works on solutions too
        solver, = buildCVRP(nilsAlgorithm(); vertices = groupedVertices(MIXED_SPEC))
        @test isempty(groupViolations(solver.data, makeSolution(solver, ok)))
        @test !isempty(groupViolations(solver.data, createSolution(solver, [[0, 1, 0]])))
    end

    @testset "constructSol! visits each group numVisits times" begin
        for (name, spec, expected) in [("GVRP", GVRP_SPEC, 4), ("clusters", CLUSTER_SPEC, 12), ("mixed", MIXED_SPEC, 7)]
            for seed in 1:10
                solver, = buildCVRP(nilsAlgorithm(); vertices = groupedVertices(spec), instanceSeed = seed)
                setSeed!(solver, MersenneTwister(seed))
                sol = constructSol!(solver)
                @test groupViolations(solver.data, sol) == String[]
                @test length(visitedBy(sol)) == expected
                @test all(r.visits[1] == 0 && r.visits[end] == 0 for r in sol.routes)
                @test size(sol.lastEval) == (length(solver.neighborhoods), length(sol.routes), length(sol.routes))
                checkConsistency(solver, sol)
            end
        end
    end

    @testset "sameRoute groups are never split, even under tight capacity" begin
        # three clusters of 4 customers, capacity for about one cluster per route: the construction must keep each
        # cluster together and pay the capacity penalty instead of splitting it
        for seed in 1:10
            solver, = buildCVRP(ilsAlgorithm(); vertices = groupedVertices(CLUSTER_SPEC), Q = 12.0, instanceSeed = seed)
            setSeed!(solver, MersenneTwister(seed))
            sol = constructSol!(solver)
            @test groupViolations(solver.data, sol) == String[]
            @test length(visitedBy(sol)) == 12
        end
    end

    @testset "number of seed routes follows the groups" begin
        # one cluster can only fill one route, however many routes are allowed
        solver, = buildCVRP(nilsAlgorithm(); vertices = groupedVertices([(1:12, 12, true)]))
        sol = constructSol!(solver)
        @test length(sol.routes) == 1
        @test sort(sol.routes[1].visits[2:end-1]) == collect(1:12)
        # extra routes (`r`) never exceed what the groups can seed
        @test length(constructSol!(solver; r = 5).routes) == 1

        # a group that needs 3 visits in several routes can seed 3 routes
        solver, = buildCVRP(nilsAlgorithm(); vertices = groupedVertices([(1:12, 3, false)]))
        @test length(constructSol!(solver).routes) == 3
        sol = constructSol!(solver)
        @test groupViolations(solver.data, sol) == String[]
    end

    @testset "construction of a plain VRP is unchanged by the groups" begin
        # every customer alone in a group needing one visit is the classic problem
        solver, demands, Q = buildCVRP(nilsAlgorithm(); nCust = 20, Q = 25.0, instanceSeed = 4)
        sol = constructSol!(solver)
        @test customersOf(sol) == collect(1:20)
        @test groupViolations(solver.data, sol) == String[]
        checkCVRP(sol, demands, Q, 20)
    end

    @testset "the search refuses groups it cannot keep together yet" begin
        for make in (nilsAlgorithm(), ilsAlgorithm())
            solver, = buildCVRP(make; vertices = groupedVertices(CLUSTER_SPEC))
            @test_throws ErrorException quiet(() -> solve!(solver))
            solver, = buildCVRP(make; vertices = groupedVertices(MIXED_SPEC))
            @test_throws ErrorException quiet(() -> solve!(solver))
        end
        @test GenericILS.supportsGroups(ProblemData(groupedVertices(GVRP_SPEC), zeros(13, 13), 3).groups)
        # one visit in a single route is no constraint at all
        @test GenericILS.supportsGroups([Group(1, 1, true)])
    end

    @testset "solving a problem with optional customers keeps the visit counts" begin
        # the moves of the search only relocate visits, so they keep every group at numVisits visits
        for make in (nilsAlgorithm(), ilsAlgorithm())
            solver, = buildCVRP(make; vertices = groupedVertices(GVRP_SPEC))
            quiet(() -> solve!(solver))
            best = solver.bestFeasSol
            if best.cost < Inf
                @test groupViolations(solver.data, best) == String[]
            end
        end
    end
end
