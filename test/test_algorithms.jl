@testset "Algorithms" begin
    @testset "NILSAlgorithm accessors" begin
        algo = nilsAlgorithm()(buildCVRP(nilsAlgorithm())[1].res)
        @test NILS.getInnerIterMax(algo) == 3
        @test NILS.setInnerIterMax!(algo, 7) === algo
        @test NILS.getInnerIterMax(algo) == 7

        div = NILS.Diversification(outerShift = 3, outerSwap = 2, innerShift = 1, innerSwap = 0)
        @test NILS.setDiversification!(algo, div) === algo
        @test NILS.getDiversification(algo) === div
        @test NILS.Diversification() == NILS.Diversification(2, 0, 2, 0)

        ac, sc = NILS.AcceptBest(), NILS.ByIterMax(9)
        @test NILS.setAcceptCriteria!(algo, ac) === algo
        @test NILS.getAcceptCriteria(algo) === ac
        @test NILS.setStopCriteria!(algo, sc) === algo
        @test NILS.getStopCriteria(algo) === sc

        @test NILS.setTimeLimits!(algo, 12.0, 34.0) === algo
        @test NILS.getTimeLimits(algo) == (ils = 12.0, sp = 34.0)
    end

    @testset "ILSAlgorithm accessors" begin
        algo = ilsAlgorithm()(buildCVRP(nilsAlgorithm())[1].res)
        div = ILS.Diversification(shift = 3, swap = 2)
        @test ILS.setDiversification!(algo, div) === algo
        @test ILS.getDiversification(algo) === div
        @test ILS.Diversification() == ILS.Diversification(2, 0)

        ac, sc = ILS.AcceptBest(), ILS.ByIterMax(9)
        @test ILS.setAcceptCriteria!(algo, ac) === algo
        @test ILS.getAcceptCriteria(algo) === ac
        @test ILS.setStopCriteria!(algo, sc) === algo
        @test ILS.getStopCriteria(algo) === sc

        @test ILS.setTimeLimits!(algo, 12.0, 34.0) === algo
        @test ILS.getTimeLimits(algo) == (ils = 12.0, sp = 34.0)
    end

    @testset "time limits and pool strategy through the solver" begin
        solver, demands, Q = buildCVRP(nilsAlgorithm(stop = NILS.ByTime(60.0)))
        NILS.setTimeLimitILS(solver, 0.0)         # forces the search phase to end right away
        NILS.setTimeLimitSP(solver, 30.0)
        NILS.aggressivePool(solver, true)
        @test solver.algorithm.timeLimitILS == 0.0
        @test solver.algorithm.timeLimitSP == 30.0
        @test solver.algorithm.aggressivePool
        quiet(() -> solve!(solver))
        checkCVRP(bestSol(solver), demands, Q, 12)

        solver, demands, Q = buildCVRP(ilsAlgorithm(stop = ILS.ByTime(60.0)))
        ILS.setTimeLimitILS(solver, 0.0)
        ILS.setTimeLimitSP(solver, 30.0)
        ILS.aggressivePool(solver, true)
        quiet(() -> solve!(solver))
        checkCVRP(bestSol(solver), demands, Q, 12)
    end

    @testset "aggressive pool and perturbations on a larger instance" begin
        for (make, mod) in ((nilsAlgorithm, NILS), (ilsAlgorithm, ILS))
            solver, demands, Q = buildCVRP(make(stop = mod.ByIterMax(8), aggressivePool = true);
                nCust = 25, Q = 40.0, instanceSeed = 5)
            quiet(() -> solve!(solver))
            checkCVRP(bestSol(solver), demands, Q, 25)
            @test solver.statistics.bestFeasCost <= solver.statistics.bestFeasCostBefSP + 1e-6
        end
    end

    @testset "restricted neighborhoods" begin
        for nbh in [(TwoOptStar(),), (IntraShift(), InterShift{1}()), (InterSwap{1,1}(), InterSwap{2,2}())]
            solver, demands, Q = buildCVRP(nilsAlgorithm(); neighborhoods = nbh, nCust = 15, instanceSeed = 4)
            quiet(() -> solve!(solver))
            checkCVRP(bestSol(solver), demands, Q, 15)
        end
    end

    @testset "target-rate penalty manager drives a full run" begin
        pm = TargetRatePenaltyManager(
            penaltyCustom = 100.0, penaltyCustomIncrease = 0.4, penaltyCustomDecrease = 0.2,
            penaltyStandard1 = 100.0, penaltyStandard1Increase = 0.4, penaltyStandard1Decrease = 0.2,
            penaltyStandard2 = 0.0, penaltyStandard2Increase = 0.0, penaltyStandard2Decrease = 0.0,
            targetFeasRate = 0.7, feasRateTolerance = 0.05, updatePeriod = 3)
        solver, demands, Q = buildCVRP(nilsAlgorithm(stop = NILS.ByIterMax(12)); penaltyManager = pm,
            nCust = 20, Q = 25.0, instanceSeed = 6)
        @test solver.penaltyManager isa TargetRatePenaltyManager
        quiet(() -> solve!(solver))
        @test isfinite(solver.bestFeasSol.cost)
        checkCVRP(bestSol(solver), demands, Q, 20)
    end

    @testset "time windows in a standard resource" begin
        for (make, mod) in ((nilsAlgorithm, NILS), (ilsAlgorithm, ILS))
            solver, demands, Q, t, lb, ub = buildTW(make(stop = mod.ByIterMax(10)))
            quiet(() -> solve!(solver))
            sol = bestSol(solver)
            @test isfinite(sol.cost)
            checkCVRP(sol, demands, Q, 10)
            checkTW(sol, t, lb, ub)
        end
    end
end
