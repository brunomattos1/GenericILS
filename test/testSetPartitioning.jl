using Random
using JuMP

"Model of the Set Partitioning over `pool` for `solver`, as the algorithms build it. Returns `(model, λ, ok)`."
function partitioningModel(solver, pool)
    sp = Model(HiGHS.Optimizer)
    set_silent(sp)
    @variable(sp, λ[r = 1:length(pool)], Bin)
    @objective(sp, Min, sum(GenericILS.NILS.routeCost(solver, pool[r]) * λ[r] for r in eachindex(pool)))
    ok = GenericILS.addPartitioningConstraints!(sp, solver, pool, λ)
    return sp, λ, ok
end

"Best cost over all subsets of `pool` that are valid solutions (`groupViolations`) with few enough routes, or `Inf`."
function bruteForceOptimum(solver, pool)
    best = Inf
    cost = [GenericILS.NILS.routeCost(solver, r) for r in pool]
    for mask in 1:(2^length(pool) - 1)
        chosen = [r for r in eachindex(pool) if (mask >> (r - 1)) & 1 == 1]
        length(chosen) <= solver.data.maxNbRoutes || continue
        isempty(groupViolations(solver.data, pool[chosen])) || continue
        best = min(best, sum(cost[chosen]))
    end
    return best
end

"A pool: (some of) the routes of two constructed solutions plus random routes that need not respect any group."
function mixedPool(solver, rng, nCust; junk = 5, keep = 1.0)
    pool = Vector{Vector{Int}}()
    for _ in 1:2
        sol = constructSol!(solver)
        append!(pool, [copy(rt.visits) for rt in sol.routes if length(rt.visits) > 2 && rand(rng) < keep])
    end
    for _ in 1:junk
        push!(pool, vcat(0, shuffle(rng, 1:nCust)[1:rand(rng, 1:4)], 0))
    end
    return unique!(pool)
end

# 12 customers: groups that need some of their members, one of them in a single route
const SMALL_EXCHANGE_SPEC = [(1:4, 2, false), (5:8, 2, true), (9:10, 1, false), (11:12, 1, true)]

@testset "Set Partitioning over groups" begin
    @testset "the model selects exactly the best valid combination of pooled routes" begin
        feasible = infeasible = 0
        for (name, spec) in [("GVRP", GVRP_SPEC), ("clusters", CLUSTER_SPEC), ("mixed", MIXED_SPEC),
                             ("exchange", SMALL_EXCHANGE_SPEC), ("plain", [([i], 1, false) for i in 1:12])],
            maxRoutes in (nothing, 2), keep in (1.0, 0.5), seed in 1:5
            solver, = buildCVRP(nilsAlgorithm(); nCust = 12, vertices = groupedVertices(spec), instanceSeed = seed)
            setSeed!(solver, MersenneTwister(seed))
            maxRoutes === nothing || setData!(solver, ProblemData(solver.data.vertices, solver.data.costMatrix, maxRoutes))
            pool = mixedPool(solver, MersenneTwister(seed), 12; keep = keep)
            length(pool) <= 13 || (pool = pool[1:13])
            expected = bruteForceOptimum(solver, pool)

            sp, λ, ok = partitioningModel(solver, pool)
            if ok
                optimize!(sp)
            end
            if expected == Inf
                @test !ok || termination_status(sp) == MOI.INFEASIBLE
                infeasible += 1
            else
                @test ok
                @test termination_status(sp) == MOI.OPTIMAL
                @test objective_value(sp) ≈ expected atol = 1e-6
                chosen = [pool[r] for r in eachindex(pool) if value(λ[r]) > 0.5]
                @test groupViolations(solver.data, chosen) == String[]
                feasible += 1
            end
        end
        @test feasible > 20 && infeasible > 5
    end

    @testset "routes that split a group that stays in one route are never chosen" begin
        # group {1, 2, 3} needs 2 visits in one route; routes [1] and [2] would cover it only by splitting it
        vertices = vcat(groupedVertices([(1:3, 2, true)]), [Vertex(4, Group(2, 1, false))])
        solver, = buildCVRP(nilsAlgorithm(); nCust = 4, vertices = vertices)
        pool = [[0, 1, 0], [0, 2, 0], [0, 1, 2, 0], [0, 4, 0], [0, 4, 3, 0]]
        sp, λ, ok = partitioningModel(solver, pool)
        @test ok
        optimize!(sp)
        chosen = [pool[r] for r in eachindex(pool) if value(λ[r]) > 0.5]
        @test groupViolations(solver.data, chosen) == String[]
        @test [0, 1, 0] ∉ chosen && [0, 2, 0] ∉ chosen
        @test is_fixed(λ[1]) && is_fixed(λ[2]) && !is_fixed(λ[3])
        # without a route that serves the group completely there is no solution
        _, _, ok = partitioningModel(solver, [[0, 1, 0], [0, 2, 0], [0, 4, 0]])
        @test !ok
        # nor when no route has any member of a group
        _, _, ok = partitioningModel(solver, [[0, 4, 0]])
        @test !ok
    end

    @testset "a classic VRP gets the same model as before" begin
        solver, = buildCVRP(nilsAlgorithm(); nCust = 12)
        pool = mixedPool(solver, MersenneTwister(1), 12; junk = 0)
        sp, _, ok = partitioningModel(solver, pool)
        @test ok
        # one equality per customer and the limit on the number of routes: nothing else
        @test num_constraints(sp; count_variable_in_set_constraints = false) == 12 + 1
        @test num_constraints(sp, AffExpr, MOI.EqualTo{Float64}) == 12
    end

    @testset "$algoName finishes with a valid solution that is never worse ($specName)" for (algoName, algoMake) in
            (("NILS", nilsAlgorithm()), ("ILS", ilsAlgorithm())),
            (specName, spec) in (("GVRP", GVRP_SPEC), ("clusters", CLUSTER_SPEC), ("mixed", MIXED_SPEC), ("exchange", EXCHANGE_SPEC))
        nCust = maximum(maximum(ids) for (ids, _, _) in spec)
        for seed in 1:3
            solver, = buildCVRP(algoMake; nCust = nCust, Q = 100.0, vertices = groupedVertices(spec), instanceSeed = seed,
                                neighborhoods = (NEIGHBORHOODS..., IntraGroupExchange()))
            setSeed!(solver, MersenneTwister(seed))
            quiet(() -> solve!(solver))
            best = solver.bestFeasSol
            @test best.cost < Inf
            @test groupViolations(solver.data, best) == String[]
            # the distance of the solution found by the MIP is its real distance (it used to be left at 0)
            @test best.dist ≈ manualCost(best, solver.data.costMatrix)
            @test best.dist ≈ best.cost
            @test solver.statistics.bestFeasCost <= solver.statistics.bestFeasCostBefSP + 1e-6
        end
    end
end
