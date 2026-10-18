@testset "Solver API" begin
    solver, demands, Q = buildCVRP(nilsAlgorithm())
    quiet(() -> solve!(solver))

    @testset "getters and setters" begin
        @test getSeed(solver) isa MersenneTwister
        s = MersenneTwister(7)
        @test getSeed(setSeed!(solver, s)) === s

        algo = getAlgorithm(solver)
        @test setAlgorithm!(solver, algo) === solver
        @test getAlgorithm(solver) === algo

        pm = getPenaltyManager(solver)
        @test setPenaltyManager!(solver, pm) === solver

        @test getRes(solver) === solver.res
        @test setRes!(solver, solver.res) === solver
        @test getData(solver) === solver.data
        @test setData!(solver, solver.data) === solver
        @test getNeighborhoods(solver) == NEIGHBORHOODS
        @test setNeighborhoods!(solver, NEIGHBORHOODS) === solver

        @test setMIPSolver!(solver, HiGHS.Optimizer) === solver
        @test getMIPSolver(solver) === HiGHS.Optimizer
        @test getStatistics(solver) === solver.statistics
        @test getCostMatrix(solver) === solver.data.costMatrix
        @test getCurrSol(solver) === solver.currSol
    end

    @testset "solutions and routes" begin
        sol = bestSol(solver)
        @test getCost(sol) == sol.cost
        @test getDistance(sol) == sol.dist
        @test getRoutes(sol) === sol.routes
        all = getBestRoutes(solver)
        @test all === solver.bestFeasSol.routes
        @test getBestRoutes(solver, 1) == [all[1]]
        @test getBestRoutes(solver, [1]) == [all[1]]
        @test length(getBestRoutes(solver, 1:length(all))) == length(all)
    end

    @testset "Vertex, ProblemData and Statistics" begin
        v(id) = Vertex(id, Group(id, 1, false))
        @test v(3) == v(3)
        @test v(3) != v(4)
        @test hash(v(3)) == hash(v(3))
        @test length(Set([v(1), v(1), v(2)])) == 2
        # vertices are identified by their id alone
        @test Vertex(3, Group(1, 1, false)) == Vertex(3, Group(2, 2, true))
        @test isempty(ProblemData().vertices)
        @test GenericILS.Statistics().bestFeasCost == Inf
    end

    @testset "users can build and inspect solutions" begin
        routes = [[0, 1, 2, 0], [0, 3, 4, 5, 6, 0], [0, 7, 8, 9, 10, 11, 12, 0]]
        sol = createSolution(solver, routes)
        @test manualCost(sol, solver.data.costMatrix) ≈ sol.dist
        expected = sum(solver.data.costMatrix[r[i]+1, r[i+1]+1] for r in routes for i in 1:length(r)-1)
        @test sol.dist ≈ expected
        fixed = createSolution(solver, routes; dist = 1.0, cost = 2.0)
        @test (fixed.dist, fixed.cost) == (1.0, 2.0)
        @test objectiveValue(solver, solver.bestFeasSol) >= solver.bestFeasSol.dist
        @test canPruneByDist(solver)

        out = capture() do
            printLabels(solver, sol)
            printConcatenations(solver, sol)
        end
        @test occursin("FORWARD LABELS", out) && occursin("BACKWARD LABELS", out)
        @test occursin("Concatenating", out)
    end

    @testset "algorithm contract" begin
        @test_throws ErrorException run!(DummyAlgorithm(), solver)
        @test_throws ErrorException GenericILS.totalTime(DummyAlgorithm())
    end

    @testset "printInfo reports the search state" begin
        out = capture(() -> printInfo(solver))
        @test occursin("|", out)
    end
end
