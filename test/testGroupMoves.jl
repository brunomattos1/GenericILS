using Random

# 11 visits: three groups that need all (or several) of their visits in one route, and two plain customers
const LOCKED_SPEC = [(1:4, 4, true), (5:7, 3, true), (8:11, 2, true), (12:13, 1, false), (14:16, 1, false)]
const MOVE_NEIGHBORHOODS = (NEIGHBORHOODS..., IntraGroupExchange())

"A solver over customers on a line (depot at 0, customer `i` at `pos[i + 1]`, distance = |x - y|), no capacity limit."
function lineGroupSolver(pos, vertices; neighborhoods = NEIGHBORHOODS)
    solver, = buildCVRP(nilsAlgorithm(); nCust = length(pos) - 1, Q = 1000.0, vertices = vertices, neighborhoods = neighborhoods)
    solver.data.costMatrix .= [abs(a - b) for a in pos, b in pos]
    return solver
end

routeOf(sol, v) = findfirst(rt -> v in rt.visits, sol.routes)

lockedSolver(seed) = begin
    solver, = buildTW(nilsAlgorithm(); nCust = 16, Q = 30.0, horizon = 25.0, instanceSeed = seed + 1,
                      vertices = groupedVertices(LOCKED_SPEC), neighborhoods = MOVE_NEIGHBORHOODS)
    setSeed!(solver, MersenneTwister(seed))
    solver
end

@testset "Moves between routes keep the groups together" begin
    @testset "canLeaveRoute and safeCuts!" begin
        data = ProblemData(groupedVertices([(1:3, 3, true), (4:5, 2, true), (6:6, 1, false), (7:8, 1, false)]), zeros(9, 9), 3)
        route = [0, 1, 2, 3, 6, 4, 5, 0]
        @test !GenericILS.canLeaveRoute(data, route, 2, 1)       # 1 alone
        @test !GenericILS.canLeaveRoute(data, route, 2, 2)       # 1, 2 but not 3
        @test  GenericILS.canLeaveRoute(data, route, 2, 3)       # all of group 1
        @test !GenericILS.canLeaveRoute(data, route, 3, 3)       # 2, 3, 6: group 1 incomplete
        @test  GenericILS.canLeaveRoute(data, route, 5, 1)       # plain customer 6
        @test !GenericILS.canLeaveRoute(data, route, 5, 2)       # 6, 4: group 2 incomplete
        @test  GenericILS.canLeaveRoute(data, route, 6, 2)       # all of group 2
        @test  GenericILS.canLeaveRoute(data, route, 2, 6)       # everything but the depots

        safe, seen = Bool[], zeros(Int, length(data.groups))
        GenericILS.safeCuts!(safe, seen, data, route)
        @test safe == [true, false, false, true, true, false, true, true]
        @test all(iszero, seen)
        # a route without groups that need to stay together can be cut anywhere
        GenericILS.safeCuts!(safe, seen, data, [0, 6, 7, 8, 0])
        @test all(safe)
    end

    @testset "a visit of a group that stays in one route never leaves it alone" begin
        # group {1, 2} needs both visits in one route. 2 sits next to customer 3 of route 2, so moving or swapping
        # it there would pay off, but it would split the group.
        pos = [0.0, 10.0, 100.0, 99.0]
        vertices = vcat(groupedVertices([(1:2, 2, true)]), [Vertex(3, Group(2, 1, false))])
        for neigh in (InterSwap{1,1}(), InterShift{1}(), InterShift{2}(), InterSwap{2,1}(), InterSwap{2,2}(), TwoOptStar())
            solver = lineGroupSolver(pos, vertices)
            sol = makeSolution(solver, [[0, 1, 2, 0], [0, 3, 0]])
            while search!(neigh, solver, sol)
                @test groupViolations(solver.data, sol) == String[]
            end
            @test routeOf(sol, 1) == routeOf(sol, 2)
        end
        # the swap of 2 and 3 is the only one that would pay off: with the rule there is no move at all
        solver = lineGroupSolver(pos, vertices)
        sol = makeSolution(solver, [[0, 1, 2, 0], [0, 3, 0]])
        @test !search!(InterSwap{1,1}(), solver, sol)
        @test [rt.visits for rt in sol.routes] == [[0, 1, 2, 0], [0, 3, 0]]
    end

    @testset "a whole group can move as a block" begin
        # InterShift{2}: group {1, 2} (at 100, 101) moves from the route of customer 3 to the route of customer 4
        pos = [0.0, 100.0, 101.0, 5.0, 99.0]
        vertices = vcat(groupedVertices([(1:2, 2, true)]), [Vertex(3, Group(2, 1, false)), Vertex(4, Group(3, 1, false))])
        solver = lineGroupSolver(pos, vertices)
        sol = makeSolution(solver, [[0, 3, 1, 2, 0], [0, 4, 0]])
        @test search!(InterShift{2}(), solver, sol)
        @test groupViolations(solver.data, sol) == String[]
        @test routeOf(sol, 1) == routeOf(sol, 2) == routeOf(sol, 4) != routeOf(sol, 3)
        checkConsistency(solver, sol)

        # TwoOptStar: the tail [1, 2] goes along with the exchange of tails
        pos = [0.0, -100.0, -101.0, 100.0, 101.0]
        vertices = vcat(groupedVertices([(1:2, 2, true)]), [Vertex(3, Group(2, 1, false)), Vertex(4, Group(3, 1, false))])
        solver = lineGroupSolver(pos, vertices)
        sol = makeSolution(solver, [[0, 3, 1, 2, 0], [0, 4, 0]])
        @test search!(TwoOptStar(), solver, sol)
        @test groupViolations(solver.data, sol) == String[]
        @test routeOf(sol, 1) == routeOf(sol, 2) != routeOf(sol, 3)
        @test routeOf(sol, 3) == routeOf(sol, 4)
        checkConsistency(solver, sol)
    end

    @testset "$(typeof(neigh)) keeps the groups and improves consistently" for neigh in MOVE_NEIGHBORHOODS
        for seed in 1:6
            solver = lockedSolver(seed)
            sol = constructSol!(solver)
            visited = length(customersOf(sol))
            for _ in 1:40
                cost0 = sol.cost
                flag = search!(neigh, solver, sol)
                @test sol.cost <= cost0 + 1e-6
                flag && @test sol.cost < cost0 - 1e-6
                @test length(customersOf(sol)) == visited
                @test groupViolations(solver.data, sol) == String[]
                flag || break
            end
            checkConsistency(solver, sol)
            cost1 = sol.cost
            @test !search!(neigh, solver, sol)
            @test sol.cost ≈ cost1
        end
    end

    @testset "RVND with every neighborhood keeps the groups" begin
        improved = 0
        for seed in 1:8
            solver = lockedSolver(seed)
            sol = constructSol!(solver)
            cost0 = sol.cost
            quiet(() -> GenericILS.NILS.RVND!(solver, sol))
            @test sol.cost <= cost0 + 1e-6
            improved += sol.cost < cost0 - 1e-6
            @test groupViolations(solver.data, sol) == String[]
            checkConsistency(solver, sol)
            fill!(sol.lastEval, 0)
            @test !any(search!(n, solver, sol) for n in solver.neighborhoods)
        end
        @test improved >= 4
    end

    @testset "perturbations keep the groups ($mod)" for mod in (GenericILS.NILS, GenericILS.ILS)
        moved = 0
        for seed in 1:6
            solver = lockedSolver(seed)
            sol = constructSol!(solver)
            visited = length(customersOf(sol))
            for _ in 1:100
                moved += mod.randomInterShift10!(solver, sol) + mod.randomInterSwap11!(solver, sol)
                @test groupViolations(solver.data, sol) == String[]
            end
            @test length(customersOf(sol)) == visited
            checkConsistency(solver, sol)
        end
        @test moved > 100

        # when nothing can leave its route the perturbation does nothing
        solver, = buildCVRP(nilsAlgorithm(); nCust = 12, vertices = groupedVertices([(1:6, 6, true), (7:12, 6, true)]))
        sol = makeSolution(solver, [vcat(0, 1:6, 0), vcat(0, 7:12, 0)])
        before = [copy(rt.visits) for rt in sol.routes]
        @test !mod.randomInterShift10!(solver, sol)
        @test !mod.randomInterSwap11!(solver, sol)
        @test [rt.visits for rt in sol.routes] == before
    end
end
