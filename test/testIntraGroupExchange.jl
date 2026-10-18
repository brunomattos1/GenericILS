using Random

# 16 customers; 8 visits: three of the first group (any routes), two of a group that stays in one route,
# two of a third group and one of a last group that is alone with `sameRoute` (no constraint at all).
const EXCHANGE_SPEC = [(1:5, 3, false), (6:10, 2, true), (11:13, 2, false), (14:16, 1, true)]
const EXCHANGE_NEIGHBORHOODS = (NEIGHBORHOODS..., IntraGroupExchange())

function exchangeSolver(; nCust = 16, spec = EXCHANGE_SPEC, Q = 30.0, horizon = 25.0, instanceSeed = 3, seed = 1)
    solver, = buildTW(nilsAlgorithm(); nCust = nCust, Q = Q, horizon = horizon, instanceSeed = instanceSeed,
                      vertices = groupedVertices(spec), neighborhoods = EXCHANGE_NEIGHBORHOODS)
    setSeed!(solver, MersenneTwister(seed))
    return solver
end

"A random `(move, expectedRoutes)` of IntraGroupExchange on `sol`, or `nothing` when no member can be exchanged."
function randomExchange(rng, solver, sol)
    data = solver.data
    visited = GenericILS.visitedVertices(data, sol)
    cands = [(r, i, u) for r in eachindex(sol.routes) for i in 2:length(sol.routes[r].visits)-1
             for u in data.members[data.groupOf[sol.routes[r].visits[i]]] if !visited[u]]
    isempty(cands) && return nothing
    r1, i, u = rand(rng, cands)
    r2 = GenericILS.mustStayInRoute(data.groups[data.groupOf[u]]) ? r1 : rand(rng, eachindex(sol.routes))
    v1, v2 = sol.routes[r1].visits, sol.routes[r2].visits
    if r1 == r2
        j = i
        dist = GenericILS.intraGroupReplaceCost(sol.dist, data.costMatrix, v1, i, u)
        expected = Dict(r1 => vcat(v1[1:i-1], u, v1[i+1:end]))
    else
        j = rand(rng, 2:length(v2))
        dist = GenericILS.intraGroupExchangeCost(sol.dist, data.costMatrix, v1, v2, i, j, u)
        expected = Dict(r1 => vcat(v1[1:i-1], v1[i+1:end]), r2 => vcat(v2[1:j-1], u, v2[j:end]))
    end
    return GenericILS.BestExchange(0.0, dist, r1, r2, i, j, u), expected
end

@testset "IntraGroupExchange" begin
    @testset "apply! matches the exchange on plain vectors" begin
        applied = inPlace = betweenRoutes = 0
        for seed in 1:8
            solver = exchangeSolver(seed = seed, instanceSeed = seed + 2)
            rng = MersenneTwister(seed)
            for trial in 1:15
                sol = constructSol!(solver)
                m = randomExchange(rng, solver, sol)
                m === nothing && continue
                move, expected = m
                visitedBefore = length(customersOf(sol))
                GenericILS.apply!(IntraGroupExchange(), solver, sol, move)
                for (r, visits) in expected
                    @test sol.routes[r].visits == visits
                end
                @test length(customersOf(sol)) == visitedBefore
                @test groupViolations(solver.data, sol) == String[]
                @test sol.dist ≈ manualCost(sol, solver.data.costMatrix)
                checkConsistency(solver, sol)
                applied += 1
                move.firstRoute == move.secondRoute ? (inPlace += 1) : (betweenRoutes += 1)
            end
        end
        @test applied > 40 && inPlace > 5 && betweenRoutes > 5
    end

    @testset "the predicted objective equals the applied one on feasible routes" begin
        checked = 0
        for seed in 1:8
            solver = exchangeSolver(seed = seed, instanceSeed = seed + 2)
            rng = MersenneTwister(seed)
            for trial in 1:15
                sol = constructSol!(solver)
                m = randomExchange(rng, solver, sol)
                m === nothing && continue
                move, = m
                predicted = GenericILS.evalIntraGroupExchange(solver, sol, move.firstRoute, move.secondRoute,
                    move.firstIdx, move.secondIdx, move.customer, move.dist)
                GenericILS.apply!(IntraGroupExchange(), solver, sol, move)
                # the infeasibility count is only an upper bound; everything else must be exact
                if sol.routes[move.firstRoute].infeas == 0 && sol.routes[move.secondRoute].infeas == 0
                    @test predicted ≈ sol.cost
                    checked += 1
                else
                    @test predicted >= sol.cost - 1e-6
                end
            end
        end
        @test checked > 20
    end

    @testset "search! keeps the groups, never worsens and ends in a local optimum" begin
        improvedAny = false
        for seed in 1:8
            solver = exchangeSolver(seed = seed, instanceSeed = seed + 2)
            sol = constructSol!(solver)
            visited = length(customersOf(sol))
            for _ in 1:50
                cost0 = sol.cost
                flag = search!(IntraGroupExchange(), solver, sol)
                @test sol.cost <= cost0 + 1e-6
                flag && @test sol.cost < cost0 - 1e-6
                flag && (improvedAny = true)
                @test length(customersOf(sol)) == visited
                @test groupViolations(solver.data, sol) == String[]
                flag || break
            end
            checkConsistency(solver, sol)
            cost1 = sol.cost
            @test !search!(IntraGroupExchange(), solver, sol)
            @test sol.cost ≈ cost1
            # the cache of evaluated route pairs must not hide improvements: a fresh evaluation finds none either
            fill!(sol.lastEval, 0)
            @test !search!(IntraGroupExchange(), solver, sol)
            @test sol.cost ≈ cost1
        end
        @test improvedAny
    end

    # An exchange changes which members of a group are unvisited, and with it the moves available to every route
    # that holds a member of the group, even routes the exchange did not touch. A search that trusts the cache of
    # evaluated route pairs must still find exactly what a fresh one finds.
    @testset "the cache of evaluated pairs does not hide moves after other exchanges" begin
        # group {1, 2, 3, 4} needs two visits (members at 100, 10.2, -300, -500 on a line), plus a customer x at 10.
        # 1 and 2 are visited and the unvisited members are farther away: no exchange improves anything.
        pos = [0.0, 100.0, 10.2, -300.0, -500.0, 10.0]
        dist = [abs(a - b) for a in pos, b in pos]
        vertices = vcat(groupedVertices([(1:4, 2, false)]), [Vertex(5, Group(2, 1, false))])
        for seed in 1:12
            solver, = buildCVRP(nilsAlgorithm(); nCust = 5, Q = 1000.0, vertices = vertices, neighborhoods = EXCHANGE_NEIGHBORHOODS)
            solver.data.costMatrix .= dist
            sol = makeSolution(solver, [[0, 1, 0], [0, 5, 0], [0, 2, 0]])
            @test !search!(IntraGroupExchange(), solver, sol)               # explores every pair and caches it
            # an exchange in route 3 (2 leaves, -300 enters) leaves routes 1 and 2 untouched, but 2 is now unvisited,
            # so route 1 can swap its far member for it
            dist3 = GenericILS.intraGroupReplaceCost(sol.dist, solver.data.costMatrix, sol.routes[3].visits, 2, 3)
            GenericILS.apply!(IntraGroupExchange(), solver, sol, GenericILS.BestExchange(0.0, dist3, 3, 3, 2, 2, 3))
            @test sol.routes[3].visits == [0, 3, 0]

            cached = newSolution(solver); copySolution!(cached, sol)
            fresh  = newSolution(solver); copySolution!(fresh, sol)
            fill!(fresh.lastEval, 0)
            setSeed!(solver, MersenneTwister(seed))
            flagCached = search!(IntraGroupExchange(), solver, cached)
            setSeed!(solver, MersenneTwister(seed))
            flagFresh = search!(IntraGroupExchange(), solver, fresh)
            @test flagCached && flagFresh
            @test [rt.visits for rt in cached.routes] == [rt.visits for rt in fresh.routes]
            @test cached.cost ≈ fresh.cost
        end
    end

    @testset "it works together with the other neighborhoods" begin
        for seed in 1:5
            solver = exchangeSolver(seed = seed, instanceSeed = seed + 2)
            sol = constructSol!(solver)
            visited = length(customersOf(sol))
            cost0 = sol.cost
            quiet(() -> GenericILS.NILS.RVND!(solver, sol))
            @test sol.cost <= cost0 + 1e-6
            @test length(customersOf(sol)) == visited
            @test groupViolations(solver.data, sol) == String[]
            checkConsistency(solver, sol)
            # RVND ends only when no neighborhood improves, and a fresh evaluation must agree
            fill!(sol.lastEval, 0)
            @test !any(search!(n, solver, sol) for n in solver.neighborhoods)
        end
    end

    @testset "nothing to exchange when every member of every group is visited" begin
        solver = exchangeSolver(spec = [(1:8, 8, false), (9:16, 8, false)])
        sol = constructSol!(solver)
        routes = [copy(rt.visits) for rt in sol.routes]
        @test !search!(IntraGroupExchange(), solver, sol)
        @test [rt.visits for rt in sol.routes] == routes
    end

    @testset "a group that stays in one route is exchanged in place" begin
        # group {1, 2, 3, 4} needs two visits in one route: 1 and 2 (far from the depot) are visited and 3 and 4
        # are right next to it. Route 2 also runs next to 3 and 4, but they must never leave route 1.
        pos = [0.0, 30.0, 31.0, 1.0, 2.0, 1.5, 2.5]      # depot, customers 1..6 on a line
        dist = [abs(a - b) for a in pos, b in pos]
        vertices = vcat(groupedVertices([(1:4, 2, true)]), [Vertex(5, Group(2, 1, false)), Vertex(6, Group(3, 1, false))])
        solver, = buildCVRP(nilsAlgorithm(); nCust = 6, Q = 1000.0, vertices = vertices, neighborhoods = EXCHANGE_NEIGHBORHOODS)
        solver.data.costMatrix .= dist
        sol = makeSolution(solver, [[0, 1, 2, 0], [0, 5, 6, 0]])
        while search!(IntraGroupExchange(), solver, sol)
            @test groupViolations(solver.data, sol) == String[]
        end
        @test sort(sol.routes[1].visits[2:end-1]) == [3, 4]
        @test sort(sol.routes[2].visits[2:end-1]) == [5, 6]
        checkConsistency(solver, sol)
    end

    @testset "an unconstrained group moves its member to the best route" begin
        # group {1, 2, 3} needs one visit; 1 is far away and the other routes pass right next to 2 and 3
        pos = [0.0, 40.0, 10.0, 11.0, 10.5, 12.0]       # depot, customers 1..5 on a line
        dist = [abs(a - b) for a in pos, b in pos]
        vertices = vcat(groupedVertices([(1:3, 1, false)]), [Vertex(4, Group(2, 1, false)), Vertex(5, Group(3, 1, false))])
        solver, = buildCVRP(nilsAlgorithm(); nCust = 5, Q = 1000.0, vertices = vertices, neighborhoods = EXCHANGE_NEIGHBORHOODS)
        solver.data.costMatrix .= dist
        sol = makeSolution(solver, [[0, 1, 0], [0, 4, 5, 0]])
        @test search!(IntraGroupExchange(), solver, sol)
        @test groupViolations(solver.data, sol) == String[]
        @test 1 ∉ customersOf(sol)
        @test sol.routes[1].visits == [0, 0]
        @test sol.dist ≈ 24.0
    end

    @testset "solving with the new neighborhood keeps the visit counts" begin
        for make in (nilsAlgorithm(), ilsAlgorithm())
            solver, = buildCVRP(make; vertices = groupedVertices(GVRP_SPEC), neighborhoods = EXCHANGE_NEIGHBORHOODS)
            quiet(() -> solve!(solver))
            best = solver.bestFeasSol
            @test best.cost < Inf
            @test groupViolations(solver.data, best) == String[]
        end
    end

    @testset "a solver without the neighborhood in its tuple refuses it" begin
        solver, = buildCVRP(nilsAlgorithm(); vertices = groupedVertices(GVRP_SPEC))
        sol = constructSol!(solver)
        @test_throws ErrorException search!(IntraGroupExchange(), solver, sol)
    end
end
