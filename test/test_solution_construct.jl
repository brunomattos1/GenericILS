@testset "Solutions and construction" begin
    @testset "Route and Solution constructors" begin
        solver, = buildCVRP(nilsAlgorithm())
        rt = new_route(solver, [0, 1, 0])
        @test rt.visits == [0, 1, 0]
        @test (rt.infeas, rt.warpStd1, rt.warpStd2, rt.lastModif) == (0, 0.0, 0.0, 0)
        @test isempty(rt.forwardLabels) && isempty(rt.backwardLabels)
        @test rt.feasibleF == -1 && rt.lastFeasibleB == -1

        sol = new_solution(solver)
        @test isempty(sol.routes)
        @test (sol.dist, sol.cost, sol.totalInfeas, sol.timeStamp) == (0.0, 0.0, 0, 0)
    end

    @testset "copy_solution! is a deep copy" begin
        solver, = buildCVRP(nilsAlgorithm())
        src = makeSolution(solver, [[0, 1, 2, 0], [0, 3, 0]])
        src.timeStamp = 7
        src.lastEval[1, 1, 2] = 5
        dest = new_solution(solver)
        copy_solution!(dest, src)

        @test dest.dist == src.dist && dest.cost == src.cost && dest.timeStamp == 7
        @test [r.visits for r in dest.routes] == [r.visits for r in src.routes]
        @test dest.lastEval == src.lastEval
        for (a, b) in zip(dest.routes, src.routes)
            @test a.forwardLabels == b.forwardLabels
            @test a.backwardLabels == b.backwardLabels
            @test (a.infeas, a.feasibleF, a.feasibleB, a.lastModif) == (b.infeas, b.feasibleF, b.feasibleB, b.lastModif)
        end

        # mutating the source afterwards must not leak into the copy
        push!(src.routes[1].visits, 99)
        src.lastEval[1, 1, 2] = 123
        src.cost = -1.0
        @test dest.routes[1].visits == [0, 1, 2, 0]
        @test dest.lastEval[1, 1, 2] == 5
        @test dest.cost != -1.0

        # copying onto a solution with a different number of routes and a different lastEval shape
        small = makeSolution(solver, [[0, 4, 0]])
        copy_solution!(small, dest)
        @test length(small.routes) == 2 && size(small.lastEval) == size(dest.lastEval)
        copy_solution!(dest, makeSolution(solver, [[0, 5, 0]]))
        @test length(dest.routes) == 1
    end

    @testset "UserSolution" begin
        solver, = buildCVRP(nilsAlgorithm())
        sol = createSolution(solver, [[0, 1, 0]])
        @test sol isa GenericILS.UserSolution
        @test getRoutes(sol)[1].visits == [0, 1, 0]
        @test sol.dist ≈ 2 * solver.data.costMatrix[1, 2]
    end

    @testset "constructSol! builds a complete, consistent solution" begin
        for (make, nCust, Q, seed) in [(nilsAlgorithm(), 12, 30.0, 1), (ilsAlgorithm(), 20, 25.0, 4),
                                       (nilsAlgorithm(), 6, 100.0, 9)]
            solver, demands, Q = buildCVRP(make; nCust = nCust, Q = Q, instanceSeed = seed)
            sol = constructSol!(solver)
            @test customersOf(sol) == collect(1:nCust)
            @test all(r.visits[1] == 0 && r.visits[end] == 0 for r in sol.routes)
            @test size(sol.lastEval) == (length(solver.neighborhoods), length(sol.routes), length(sol.routes))
            @test all(iszero, sol.lastEval)
            checkConsistency(solver, sol)
            # capacity is respected whenever the penalties can be paid, which greedy insertion does here
            @test sol.totalInfeas == 0
            checkCVRP(sol, demands, Q, nCust)
        end
    end

    @testset "insertion cost and evaluation" begin
        solver, = buildCVRP(nilsAlgorithm())
        C = solver.data.costMatrix
        sol = makeSolution(solver, [[0, 1, 2, 0], [0, 3, 0]])
        v = sol.routes[1].visits
        for pos in 2:length(v), c in (3, 4)
            expected = sol.dist - C[v[pos-1]+1, v[pos]+1] + C[v[pos-1]+1, c+1] + C[c+1, v[pos]+1]
            @test GenericILS.bestInsertionCost(sol.dist, C, v, c, pos) ≈ expected
            dist, cost, infeas, w1, w2 = GenericILS.evalBestInsertion(solver, sol, GenericILS.Insertion(1, c, pos))
            @test dist ≈ expected
            trial = vcat(v[1:pos-1], c, v[pos:end])
            stats = routeStats(solver, trial)
            @test (infeas, w1, w2) == (stats[1], stats[3], stats[4])
        end
    end
end
