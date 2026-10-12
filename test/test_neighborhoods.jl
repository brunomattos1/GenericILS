using Random

# --- expected result of each move, computed on plain vectors --------------------------------

# Each generator returns `(move, expectedVisits)` for a random valid move, or `nothing`.
function randomMove(::TwoOptStar, rng, sol)
    rs = [r for r in eachindex(sol.routes) if length(sol.routes[r].visits) >= 3]
    length(rs) < 2 && return nothing
    r1, r2 = shuffle(rng, rs)[1:2]
    v1, v2 = sol.routes[r1].visits, sol.routes[r2].visits
    i, j = rand(rng, 1:length(v1)-2), rand(rng, 1:length(v2)-2)
    dist = GenericILS.twoOptStarCost(sol.dist, costMatrixRef[], v1, v2, i, j)
    return (GenericILS.BestMove(0.0, dist, r1, r2, i, j),
            Dict(r1 => vcat(v1[1:i], v2[j+1:end]), r2 => vcat(v2[1:j], v1[i+1:end])))
end

function randomMove(::IntraShift, rng, sol)
    rs = [r for r in eachindex(sol.routes) if length(sol.routes[r].visits) >= 4]
    isempty(rs) && return nothing
    r = rand(rng, rs)
    v = sol.routes[r].visits
    i = rand(rng, 2:length(v)-1)
    j = rand(rng, [x for x in 2:length(v)-1 if x != i])
    dist = GenericILS.intraShift10Cost(sol.dist, costMatrixRef[], v, i, j)
    # i < j: customer i lands right AFTER the old v[j] (j == i+1 is a swap); i > j: right BEFORE v[j]
    expected = copy(v)
    c = v[i]
    deleteat!(expected, i)
    insert!(expected, j, c)
    return (GenericILS.BestMove(0.0, dist, r, 0, i, j), Dict(r => expected))
end

function randomMove(::InterShift{k}, rng, sol) where {k}
    r1s = [r for r in eachindex(sol.routes) if length(sol.routes[r].visits) > k + 1]
    isempty(r1s) && return nothing
    r1 = rand(rng, r1s)
    others = [r for r in eachindex(sol.routes) if r != r1]
    isempty(others) && return nothing
    r2 = rand(rng, others)
    v1, v2 = sol.routes[r1].visits, sol.routes[r2].visits
    i, j = rand(rng, 2:length(v1)-k), rand(rng, 2:length(v2))
    dist = GenericILS.interShiftCost(sol.dist, costMatrixRef[], v1, v2, i, j, k)
    block = v1[i:i+k-1]
    return (GenericILS.BestMove(0.0, dist, r1, r2, i, j),
            Dict(r1 => vcat(v1[1:i-1], v1[i+k:end]), r2 => vcat(v2[1:j-1], block, v2[j:end])))
end

function randomMove(::InterSwap{k1,k2}, rng, sol) where {k1,k2}
    r1s = [r for r in eachindex(sol.routes) if length(sol.routes[r].visits) > k1 + 1]
    r2s = [r for r in eachindex(sol.routes) if length(sol.routes[r].visits) > k2 + 1]
    pairs = [(a, b) for a in r1s for b in r2s if a != b]
    isempty(pairs) && return nothing
    r1, r2 = rand(rng, pairs)
    v1, v2 = sol.routes[r1].visits, sol.routes[r2].visits
    i, j = rand(rng, 2:length(v1)-k1), rand(rng, 2:length(v2)-k2)
    dist = GenericILS.interSwapCost(sol.dist, costMatrixRef[], v1, v2, i, j, k1, k2)
    return (GenericILS.BestMove(0.0, dist, r1, r2, i, j),
            Dict(r1 => vcat(v1[1:i-1], v2[j:j+k2-1], v1[i+k1:end]),
                 r2 => vcat(v2[1:j-1], v1[i:i+k1-1], v2[j+k2:end])))
end

# the cost matrix of the solver under test (set by each testset)
const costMatrixRef = Ref{Matrix{Float64}}()

@testset "Neighborhoods" begin
    instances = [(nCust = 12, Q = 18.0, horizon = 25.0, seed = 3),
                 (nCust = 14, Q = 30.0, horizon = 15.0, seed = 7),
                 (nCust = 10, Q = 12.0, horizon = 60.0, seed = 11)]

    @testset "apply! of $(typeof(neigh)) matches the move on plain vectors" for neigh in NEIGHBORHOODS
        applied = 0
        for cfg in instances
            solver, = buildTW(nilsAlgorithm(); nCust = cfg.nCust, Q = cfg.Q, horizon = cfg.horizon, instanceSeed = cfg.seed)
            costMatrixRef[] = solver.data.costMatrix
            rng = MersenneTwister(cfg.seed)
            for trial in 1:25
                sol = makeSolution(solver, randomRoutes(rng, cfg.nCust, 4))
                m = randomMove(neigh, rng, sol)
                m === nothing && continue
                move, expected = m
                before = customersOf(sol)
                GenericILS.apply!(neigh, solver, sol, move)
                for (r, visits) in expected
                    @test sol.routes[r].visits == visits
                end
                @test customersOf(sol) == before
                checkConsistency(solver, sol)       # dist, infeasibility, warps, labels, cost
                applied += 1
            end
        end
        @test applied > 20
    end

    @testset "search! of $(typeof(neigh)) keeps solutions consistent and never worsens them" for neigh in NEIGHBORHOODS
        improvedAny = false
        for cfg in instances
            solver, = buildTW(nilsAlgorithm(); nCust = cfg.nCust, Q = cfg.Q, horizon = cfg.horizon, instanceSeed = cfg.seed)
            rng = MersenneTwister(cfg.seed + 1)
            for trial in 1:6
                sol = makeSolution(solver, randomRoutes(rng, cfg.nCust, 4))
                before = customersOf(sol)
                for _ in 1:30
                    cost0 = sol.cost
                    flag = search!(neigh, solver, sol)
                    @test sol.cost <= cost0 + 1e-6
                    flag && @test sol.cost < cost0 - 1e-6
                    flag && (improvedAny = true)
                    @test customersOf(sol) == before
                    flag || break
                end
                checkConsistency(solver, sol)
                # a local optimum stays a local optimum
                cost1 = sol.cost
                @test !search!(neigh, solver, sol)
                @test sol.cost ≈ cost1
            end
        end
        @test improvedAny
    end

    # The label-based violation estimate used by IntraShift must describe exactly the move that
    # dist/apply! perform: i < j -> right after v[j]; i > j -> right before v[j].
    @testset "IntraShift violation estimate equals the applied move" begin
        agree(a, b) = isapprox(a[3], b[3]; atol = 1e-9) && isapprox(a[4], b[4]; atol = 1e-9) && ((a[1] == 0) == (b[1] == 0))
        checked = 0
        for cfg in instances
            solver, = buildTW(nilsAlgorithm(); nCust = cfg.nCust, Q = cfg.Q, horizon = cfg.horizon, instanceSeed = cfg.seed)
            rng = MersenneTwister(cfg.seed)
            for trial in 1:6
                sol = makeSolution(solver, randomRoutes(rng, cfg.nCust, 3))
                for r in eachindex(sol.routes), pruned in (false, true)
                    v = sol.routes[r].visits
                    L = length(v)
                    L < 4 && continue
                    # `pruned`: only every third move is evaluated, like when distance pruning skips
                    # moves; the label chain must still be advanced for all of them
                    evaluate(n) = !pruned || n % 3 == 0
                    n = 0
                    for i in 2:L-1, j in i+1:L-1
                        GenericILS.advanceIntraShift10!(solver, sol, r, i, j)
                        evaluate(n += 1) || continue
                        moved = copy(v); c = moved[i]; deleteat!(moved, i); insert!(moved, j, c)
                        @test agree(GenericILS.computeViolIntraShift10(solver, sol, r, i, j), routeStats(solver, moved))
                        checked += 1
                    end
                    for i in L-1:-1:2, j in i-1:-1:2
                        GenericILS.advanceIntraShift10!(solver, sol, r, i, j)
                        evaluate(n += 1) || continue
                        moved = copy(v); c = moved[i]; deleteat!(moved, i); insert!(moved, j, c)
                        @test agree(GenericILS.computeViolIntraShift10(solver, sol, r, i, j), routeStats(solver, moved))
                        checked += 1
                    end
                end
            end
        end
        @test checked > 500
    end

    @testset "IntraShift can move a customer to the end of its route" begin
        # directed costs: the only cheap arcs are 0->1->2->3->0, so from [0, 3, 1, 2, 0] the unique
        # way to the optimum is sending customer 3 to the end, right after customer 2
        dist = fill(10.0, 4, 4)
        for k in 1:4
            dist[k, k] = 0.0
        end
        dist[1, 2] = dist[2, 3] = dist[3, 4] = dist[4, 1] = 1.0
        solver = solverFromDist(dist)
        sol = makeSolution(solver, [[0, 3, 1, 2, 0]])
        @test sol.dist ≈ 31.0
        @test search!(IntraShift(), solver, sol)
        @test sol.routes[1].visits == [0, 1, 2, 3, 0]
        @test sol.dist ≈ 4.0
    end

    @testset "hand-made improving moves are found" begin
        # customers on a line, depot at 0: cost of a route = total distance walked
        @testset "IntraShift puts a customer back in order" begin
            solver = lineSolver([1.0, 2.0, 3.0])
            sol = makeSolution(solver, [[0, 2, 1, 3, 0]])
            @test sol.dist ≈ 8.0
            @test search!(IntraShift(), solver, sol)
            @test sol.routes[1].visits == [0, 1, 2, 3, 0]
            @test sol.dist ≈ 6.0
            @test sol.cost ≈ 6.0
        end

        @testset "InterShift moves a customer to the route that serves its neighborhood" begin
            # Q = 2 forbids the tie-breaking alternative of putting everything in one route
            solver = lineSolver([10.0, -10.0, 10.5]; Q = 2.0)
            sol = makeSolution(solver, [[0, 1, 0], [0, 2, 3, 0]])
            @test sol.dist ≈ 20.0 + 41.0
            @test search!(InterShift{1}(), solver, sol)
            # customers 1 and 3 end up together (in either order: same length on a line), 2 alone
            @test sort([sort(r.visits) for r in sol.routes]) == sort([[0, 0, 1, 3], [0, 0, 2]])
            @test sol.dist ≈ 41.0
        end

        @testset "InterSwap exchanges two customers between routes" begin
            solver = lineSolver([10.0, -10.0, -10.5, 10.5])
            sol = makeSolution(solver, [[0, 1, 2, 0], [0, 3, 4, 0]])
            @test sol.dist ≈ 40.0 + 42.0
            @test search!(InterSwap{1,1}(), solver, sol)
            @test sol.dist ≈ 42.0
            @test customersOf(sol) == [1, 2, 3, 4]
        end

        @testset "TwoOptStar exchanges the tails of two routes" begin
            solver = lineSolver([10.0, -10.0, -9.0, 11.0])
            sol = makeSolution(solver, [[0, 1, 2, 0], [0, 3, 4, 0]])
            @test sol.dist ≈ 80.0
            @test search!(TwoOptStar(), solver, sol)
            @test sol.dist ≈ 42.0
            @test sort([sol.routes[1].visits, sol.routes[2].visits]) == sort([[0, 1, 4, 0], [0, 3, 2, 0]])
        end

        @testset "nothing to improve: search! returns false and leaves the solution alone" begin
            solver = lineSolver([1.0, 2.0, 3.0])
            sol = makeSolution(solver, [[0, 1, 2, 3, 0]])
            for neigh in NEIGHBORHOODS
                @test !search!(neigh, solver, sol)
                @test sol.routes[1].visits == [0, 1, 2, 3, 0]
            end
        end

        @testset "capacity stops an improving move that would be infeasible" begin
            # Q = 2: customer 3 would be closer to route 1, but route 1 is already full
            solver = lineSolver([10.0, -10.0, 10.5, 10.2]; Q = 2.0)
            sol = makeSolution(solver, [[0, 1, 4, 0], [0, 2, 3, 0]])
            solver.penaltyManager.penaltyCustom = 1e6
            solver.penaltyManager.penaltyStandard1 = 1e6
            for neigh in (InterShift{1}(), InterShift{2}(), TwoOptStar(), IntraShift())
                search!(neigh, solver, sol)
            end
            checkConsistency(solver, sol)
            @test all(length(r.visits) - 2 <= 2 for r in sol.routes)
            @test sol.totalInfeas == 0
            @test sol.dist <= 21.2 + 41.0 + 1e-9        # never worse than the starting point
        end
    end
end
