@testset "Labels" begin
    solver, demands, Q = buildCVRP(nilsAlgorithm(); nCust = 12, Q = 20.0)
    cum(visits) = cumsum(demands[visits .+ 1])        # forward load at each position

    @testset "forward and backward labels of a feasible route" begin
        visits = [0, 1, 2, 3, 0]
        @test sum(demands[visits .+ 1]) <= Q
        sol = makeSolution(solver, [visits])
        rt = sol.routes[1]
        n = length(visits)
        @test length(rt.forwardLabels) == n && length(rt.backwardLabels) == n
        # forward(x) includes x; backward(x) holds what comes AFTER x and not x
        @test [l.state.q for l in rt.forwardLabels] ≈ cum(visits)
        @test [l.last for l in rt.forwardLabels] == visits
        suffix(k) = sum(demands[visits[k+1:end] .+ 1])
        @test [l.state.q for l in rt.backwardLabels] ≈ [suffix(n - k + 1) for k in 1:n]
        @test [l.last for l in rt.backwardLabels] == reverse(visits)
        # joining both halves at any vertex gives the whole route
        for x in 1:n
            joined = GenericILS.myConcatenationCost(solver.res, visits[x], rt.forwardLabels[x], rt.backwardLabels[n - x + 1])
            @test joined.cost == 0.0
            @test joined.state.q ≈ sum(demands[visits .+ 1])
        end
        @test (rt.feasibleF, rt.feasibleB) == (n - 1, n - 1)
        @test (rt.lastFeasibleF, rt.lastFeasibleB) == (n, n)
    end

    @testset "feasibility markers of an overloaded route" begin
        visits = vcat(0, collect(1:8), 0)
        @test sum(demands[visits .+ 1]) > Q
        sol = makeSolution(solver, [visits])
        rt = sol.routes[1]
        n = length(visits)
        firstBad = findfirst(>(Q + 1e-5), cum(visits))
        @test [l.cost == Inf for l in rt.forwardLabels] == (cum(visits) .> Q + 1e-5)
        @test rt.lastFeasibleF == firstBad - 1
        @test rt.feasibleF == firstBad - 2
        @test rt.feasibleB < n - 1
        @test rt.infeas == n - max(rt.feasibleF, rt.feasibleB) - 1
        @test rt.infeas > 0
        @test sol.totalInfeas == rt.infeas
    end

    @testset "computeLabels variants agree" begin
        routes = [[0, 1, 2, 0], [0, 3, 4, 5, 6, 7, 0], [0, 0]]
        full = makeSolution(solver, routes)
        partial = new_solution(solver)
        partial.routes = [new_route(solver, copy(v)) for v in routes]
        computeLabels(solver, partial, 1, 2, 3)               # explicit route list
        for (a, b) in zip(full.routes, partial.routes)
            @test a.forwardLabels == b.forwardLabels
            @test a.backwardLabels == b.backwardLabels
            @test (a.feasibleF, a.feasibleB, a.lastFeasibleF, a.lastFeasibleB) ==
                  (b.feasibleF, b.feasibleB, b.lastFeasibleF, b.lastFeasibleB)
        end
        user = createSolution(solver, routes)                  # UserSolution
        for (a, b) in zip(full.routes, user.routes)
            @test a.forwardLabels == b.forwardLabels
            @test a.backwardLabels == b.backwardLabels
        end
        # relabeling a single modified route leaves the others untouched
        other = full.routes[2].forwardLabels
        insert!(full.routes[1].visits, 3, 8)               # [0, 1, 8, 2, 0]
        computeLabels(solver, full, 1)
        @test full.routes[2].forwardLabels === other
        @test [l.last for l in full.routes[1].forwardLabels] == [0, 1, 8, 2, 0]
    end

    @testset "computeRouteLabelCost" begin
        @test GenericILS.computeRouteLabelCost(solver, [0, 1, 2, 0]) == 0.0
        @test GenericILS.computeRouteLabelCost(solver, vcat(0, collect(1:8), 0)) == Inf
    end

    @testset "StandardResource: bounded accumulator with warp (time-window semantics)" begin
        # vertices 0, 1, 2 -> indices 1, 2, 3; travel d, windows [lb, ub]
        d = [0.0 3.0 9.0; 3.0 0.0 4.0; 9.0 4.0 0.0]
        lb = [0.0, 5.0, 0.0]
        ub = [100.0, 10.0, 12.0]
        std = StandardResource{1}(d, lb, ub)
        zeroState = GenericILS.StandardState(0.0, 0.0)
        fwd(q, warp) = ForwardLabel{ForwardState}(ForwardState(0.0), 0.0,
            GenericILS.StandardState(q, warp), zeroState, 0)
        bwd(q, warp) = BackwardLabel{BackwardState}(BackwardState(0.0), 0.0,
            GenericILS.StandardState(q, warp), zeroState, 0)

        # arriving before the window opens: wait (q jumps to lb), no warp
        s = extendAlongArc(std, fwd(0.0, 0.0), (1, 2))
        @test (s.q, s.stdWarp) == (5.0, 0.0)
        # arriving inside the window
        s = extendAlongArc(std, fwd(6.0, 0.0), (1, 2))
        @test (s.q, s.stdWarp) == (9.0, 0.0)
        # arriving late: clipped to ub and the excess is accumulated as warp
        s = extendAlongArc(std, fwd(9.0, 0.5), (2, 3))
        @test (s.q, s.stdWarp) == (12.0, 0.5 + 1.0)

        # backward labels carry the latest feasible start; the arc is passed as (next, cur)
        s = extendAlongArc(std, bwd(12.0, 0.0), (3, 2))     # prepend vertex 1 before vertex 2
        @test (s.q, s.stdWarp) == (8.0, 0.0)                 # 12 - d[2,3]
        s = extendAlongArc(std, bwd(7.0, 0.0), (3, 2))       # would have to leave at 3, before lb = 5: warp of 2
        @test (s.q, s.stdWarp) == (5.0, 2.0)

        # concatenation: fine when the forward arrival is not later than the backward deadline
        c = GenericILS.concatenationCost(std, 1, fwd(6.0, 0.25), bwd(8.0, 0.5))
        @test (c.q, c.stdWarp) == (6.0, 0.75)
        c = GenericILS.concatenationCost(std, 1, fwd(9.0, 0.0), bwd(8.0, 0.5))
        @test (c.q, c.stdWarp) == (8.0, 1.5)

        # the initial labels
        @test GenericILS.myInitStateForward(solver.res.customResource).std1State == zeroState
        @test GenericILS.myInitStateBackward(solver.res.customResource).std1State.q == Inf
    end
end
