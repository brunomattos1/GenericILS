@testset "Penalty managers" begin
    solver, _, _ = buildCVRP(nilsAlgorithm())
    function newSol(; infeas = 0, w1 = 0.0, w2 = 0.0)
        s = new_solution(solver)
        s.totalInfeas = infeas
        s.totalWarpStd1 = w1
        s.totalWarpStd2 = w2
        return s
    end

    @testset "StandardPenaltyManager" begin
        pm = StandardPenaltyManager(
            penaltyCustom = 1.0, penaltyCustomIncrease = 0.5, penaltyCustomDecrease = 0.5,
            penaltyStandard1 = 1.0, penaltyStandard1Increase = 0.5, penaltyStandard1Decrease = 0.5,
            penaltyStandard2 = 1.0, penaltyStandard2Increase = 0.5, penaltyStandard2Decrease = 0.5)
        # infeasible in every lane: all penalties go up
        GenericILS.updatePenalty(pm, newSol(infeas = 3, w1 = 2.0, w2 = 1.0))
        @test pm.penaltyCustom ≈ 1.5
        @test pm.penaltyStandard1 ≈ 1.5
        @test pm.penaltyStandard2 ≈ 1.5
        # feasible in every lane: all penalties go down
        GenericILS.updatePenalty(pm, newSol())
        @test pm.penaltyCustom ≈ 0.75
        @test pm.penaltyStandard1 ≈ 0.75
        @test pm.penaltyStandard2 ≈ 0.75
        # bounds: never below 0.1, never above 10000
        for _ in 1:50
            GenericILS.updatePenalty(pm, newSol())
        end
        @test pm.penaltyCustom ≈ 0.1
        for _ in 1:100
            GenericILS.updatePenalty(pm, newSol(infeas = 1, w1 = 1.0, w2 = 1.0))
        end
        @test pm.penaltyCustom ≈ 10000.0
        @test pm.penaltyStandard1 ≈ 10000.0
        @test pm.penaltyStandard2 ≈ 10000.0
    end

    @testset "TargetRatePenaltyManager" begin
        pm = TargetRatePenaltyManager(
            penaltyCustom = 1.0, penaltyCustomIncrease = 1.0, penaltyCustomDecrease = 1.0,
            penaltyStandard1 = 1.0, penaltyStandard1Increase = 1.0, penaltyStandard1Decrease = 1.0,
            penaltyStandard2 = 1.0, penaltyStandard2Increase = 1.0, penaltyStandard2Decrease = 1.0,
            targetFeasRate = 0.5, feasRateTolerance = 0.1, updatePeriod = 4)
        # the window is not full yet: nothing changes
        for _ in 1:3
            GenericILS.updatePenalty(pm, newSol(infeas = 1, w1 = 1.0, w2 = 1.0))
        end
        @test pm.penaltyCustom == 1.0
        # 4 infeasible solutions: feasibility rate 0 < target - tol -> penalties double
        GenericILS.updatePenalty(pm, newSol(infeas = 1, w1 = 1.0, w2 = 1.0))
        @test pm.penaltyCustom ≈ 2.0
        @test pm.penaltyStandard1 ≈ 2.0
        @test pm.penaltyStandard2 ≈ 2.0
        @test pm.windowCount == 0
        # 4 feasible solutions: rate 1 > target + tol -> penalties are divided by (1 + decrease)
        for _ in 1:4
            GenericILS.updatePenalty(pm, newSol())
        end
        @test pm.penaltyCustom ≈ 1.0
        @test pm.penaltyStandard1 ≈ 1.0
        @test pm.penaltyStandard2 ≈ 1.0
        # rate inside the tolerance band: unchanged
        for k in 1:4
            GenericILS.updatePenalty(pm, k <= 2 ? newSol() : newSol(infeas = 1, w1 = 1.0, w2 = 1.0))
        end
        @test pm.penaltyCustom ≈ 1.0
    end
end
