# The functions in Infeasibility.jl predict, from the labels of the *current* route, what a move
# would produce: (number of infeasible positions, label cost, std1 warp, std2 warp). The ground
# truth is to apply the move on the visit list and compute everything from scratch.
#
# What is guaranteed (and checked here):
#   * the std1/std2 warps are exact;
#   * the route is predicted feasible (0 infeasible positions) exactly when it really is, and then
#     the label cost is exact too;
#   * for infeasible routes the count of infeasible positions is a conservative estimate: it is
#     never smaller than the real one (it only trusts the labels of the untouched prefix/suffix and
#     the bridging arc), and the label cost is not meaningful.
using Random

function checkPrediction(got, want)
    @test isapprox(got[3], want[3]; atol = 1e-9)
    @test isapprox(got[4], want[4]; atol = 1e-9)
    @test got[1] >= want[1]
    @test (got[1] == 0) == (want[1] == 0)
    want[1] == 0 && @test got[2] == want[2]
    return want[1] > 0
end

@testset "Infeasibility evaluation" begin
    # tight capacity and tight windows so that many candidate routes are infeasible
    configs = [(nCust = 12, Q = 18.0, horizon = 25.0, seed = 3),
               (nCust = 14, Q = 30.0, horizon = 15.0, seed = 7),
               (nCust = 10, Q = 12.0, horizon = 60.0, seed = 11)]

    @testset "removing k consecutive customers" begin
        checked = infeasible = 0
        for cfg in configs
            solver, = buildTW(nilsAlgorithm(); nCust = cfg.nCust, Q = cfg.Q, horizon = cfg.horizon, instanceSeed = cfg.seed)
            rng = MersenneTwister(cfg.seed)
            for trial in 1:6
                sol = makeSolution(solver, randomRoutes(rng, cfg.nCust, 3))
                for r in eachindex(sol.routes), k in 1:3
                    v = sol.routes[r].visits
                    for pos in 2:length(v)-k
                        got = GenericILS.infeasArcsRemovalK(solver, sol, r, pos, k)
                        want = routeStats(solver, vcat(v[1:pos-1], v[pos+k:end]))
                        infeasible += checkPrediction(got, want)
                        checked += 1
                    end
                end
            end
        end
        @test checked > 100
        @test infeasible > 10          # the fixture really exercises infeasible routes
    end

    @testset "inserting customers" begin
        checked = infeasible = 0
        for cfg in configs
            solver, = buildTW(nilsAlgorithm(); nCust = cfg.nCust, Q = cfg.Q, horizon = cfg.horizon, instanceSeed = cfg.seed)
            rng = MersenneTwister(cfg.seed + 100)
            for trial in 1:6
                sol = makeSolution(solver, randomRoutes(rng, cfg.nCust, 3))
                for r in eachindex(sol.routes), k in 1:3
                    v = sol.routes[r].visits
                    others = [c for c in 1:cfg.nCust if c ∉ v]
                    length(others) < k && continue
                    customers = shuffle(rng, others)[1:k]
                    for pos in 2:length(v)
                        got = GenericILS.infeasArcsInsertionK(solver, sol, r, customers, pos)
                        want = routeStats(solver, vcat(v[1:pos-1], customers, v[pos:end]))
                        infeasible += checkPrediction(got, want)
                        checked += 1
                    end
                end
            end
        end
        @test checked > 100
        @test infeasible > 10
    end

    @testset "replacing a block" begin
        checked = infeasible = 0
        for cfg in configs
            solver, = buildTW(nilsAlgorithm(); nCust = cfg.nCust, Q = cfg.Q, horizon = cfg.horizon, instanceSeed = cfg.seed)
            rng = MersenneTwister(cfg.seed + 200)
            for trial in 1:5
                sol = makeSolution(solver, randomRoutes(rng, cfg.nCust, 3))
                for r in eachindex(sol.routes), kRemove in 1:2, kInsert in 1:3
                    v = sol.routes[r].visits
                    others = [c for c in 1:cfg.nCust if c ∉ v]
                    length(others) < kInsert && continue
                    new = shuffle(rng, others)[1:kInsert]
                    for pos in 2:length(v)-kRemove
                        got = GenericILS.infeasArcsReplaceBlockK(solver, sol, r, pos, kRemove, new)
                        want = routeStats(solver, vcat(v[1:pos-1], new, v[pos+kRemove:end]))
                        infeasible += checkPrediction(got, want)
                        checked += 1
                    end
                end
            end
        end
        @test checked > 100
        @test infeasible > 10
    end

    @testset "2-opt*: head of one route with the tail of another" begin
        checked = infeasible = 0
        for cfg in configs
            solver, = buildTW(nilsAlgorithm(); nCust = cfg.nCust, Q = cfg.Q, horizon = cfg.horizon, instanceSeed = cfg.seed)
            rng = MersenneTwister(cfg.seed + 300)
            for trial in 1:6
                sol = makeSolution(solver, randomRoutes(rng, cfg.nCust, 3))
                for rH in eachindex(sol.routes), rT in eachindex(sol.routes)
                    rH == rT && continue
                    head, tail = sol.routes[rH].visits, sol.routes[rT].visits
                    for posH in 1:length(head)-1, posT in 1:length(tail)-1
                        got = GenericILS.infeasArcs2optStar(solver, sol, rH, rT, posH, posT)
                        want = routeStats(solver, vcat(head[1:posH], tail[posT+1:end]))
                        infeasible += checkPrediction(got, want)
                        checked += 1
                    end
                end
            end
        end
        @test checked > 100
        @test infeasible > 10
    end
end
