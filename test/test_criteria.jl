@testset "Accept and stop criteria" begin
    @testset "NILS: $name" for (name, accept, stop) in [
        ("AcceptBest + ByIterMax", NILS.AcceptBest(), NILS.ByIterMax(4)),
        ("Metropolis + ByTemperature", NILS.Metropolis(50.0, 0.5), NILS.ByTemperature(1.0)),
        ("MetropolisTimed + ByTemperature", NILS.MetropolisTimed(50.0, 50.0, 1.5, 2.0), NILS.ByTemperature(1e-3)),
        ("RandomWalk + ByTime", NILS.RandomWalk(), NILS.ByTime(0.5)),
    ]
        solver, demands, Q = buildCVRP(nilsAlgorithm(accept = accept, stop = stop))
        quiet(() -> solve!(solver))
        checkCVRP(bestSol(solver), demands, Q, 12)
    end

    @testset "ILS: $name" for (name, accept, stop) in [
        ("AcceptBest + ByIterMax", ILS.AcceptBest(), ILS.ByIterMax(4)),
        ("Metropolis + ByTemperature", ILS.Metropolis(50.0, 0.5), ILS.ByTemperature(1.0)),
        ("MetropolisTimed + ByTemperature", ILS.MetropolisTimed(50.0, 50.0, 1.5, 2.0), ILS.ByTemperature(1e-3)),
        ("RandomWalk + ByTime", ILS.RandomWalk(), ILS.ByTime(0.5)),
    ]
        solver, demands, Q = buildCVRP(ilsAlgorithm(accept = accept, stop = stop))
        quiet(() -> solve!(solver))
        checkCVRP(bestSol(solver), demands, Q, 12)
    end

    @testset "stop criteria decide on iterations, time and temperature" begin
        solver, _, _ = buildCVRP(nilsAlgorithm(accept = NILS.Metropolis(10.0, 0.9)))
        algo = solver.algorithm
        algo.iter = 5
        algo.startTime = time() - 10.0
        algo.acceptCriteria.temperature = 0.5
        @test NILS.stop(NILS.ByIterMax(5), solver)
        @test !NILS.stop(NILS.ByIterMax(6), solver)
        @test NILS.stop(NILS.ByTime(5.0), solver)
        @test !NILS.stop(NILS.ByTime(50.0), solver)
        @test NILS.stop(NILS.ByTemperature(1.0), solver)
        @test !NILS.stop(NILS.ByTemperature(0.1), solver)

        solver, _, _ = buildCVRP(ilsAlgorithm(accept = ILS.Metropolis(10.0, 0.9)))
        algo = solver.algorithm
        algo.iter = 5
        algo.startTime = time() - 10.0
        algo.acceptCriteria.temperature = 0.5
        @test ILS.stop(ILS.ByIterMax(5), solver)
        @test !ILS.stop(ILS.ByIterMax(6), solver)
        @test ILS.stop(ILS.ByTime(5.0), solver)
        @test ILS.stop(ILS.ByTemperature(1.0), solver)
        @test !ILS.stop(ILS.ByTemperature(0.1), solver)
    end

    @testset "acceptSol compares costs with a tolerance" begin
        solver, _, _ = buildCVRP(nilsAlgorithm())
        better, worse = new_solution(solver), new_solution(solver)
        better.cost, worse.cost = 10.0, 11.0
        @test NILS.acceptSol(solver, better, worse)
        @test !NILS.acceptSol(solver, worse, better)
        @test !NILS.acceptSol(solver, better, better)
        @test ILS.acceptSol(solver, better, worse)
        @test !ILS.acceptSol(solver, worse, better)
    end

    @testset "$M MetropolisTimedIter cools down and eventually freezes" for M in (NILS, ILS)
        solver, _, _ = buildCVRP(M === NILS ? nilsAlgorithm() : ilsAlgorithm())
        crit = M.MetropolisTimedIter(1.0, 1.0, 10.0, 0.5, 0)
        cur, best = new_solution(solver), new_solution(solver)
        cur.cost, best.cost = 5.0, 10.0          # improvement: always taken
        M.accept!(crit, solver, cur, best)
        @test best.cost == 5.0
        cur.cost = 100.0                          # much worse, temperature ~0: rejected
        M.accept!(crit, solver, cur, best)
        @test cur.cost == best.cost
        for _ in 1:200
            M.accept!(crit, solver, cur, best)
        end
        @test crit.temperature == 0.0
    end
end
