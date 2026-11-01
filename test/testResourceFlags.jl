# isCostResource / isSymmetric defined for the resource type instead of for the whole process. The helpers define the
# global flags (false) for the CVRP resource; PerTypeCost is a cost resource in the same process.

# CVRP where the whole route cost lives in the resource (like HFVRP/MDVRP): the cost matrix of the problem is zero.
struct PerTypeCost <: AbstractResource
    dist::Matrix{Float64}
end
struct PTFwd
    D::Float64
end
struct PTBwd
    D::Float64
end
GenericILS.isCostResource(::PerTypeCost) = true
GenericILS.isSymmetric(::PerTypeCost) = false
initStateForward(::PerTypeCost) = (PTFwd(0.0), 0.0)
initStateBackward(::PerTypeCost) = (PTBwd(0.0), 0.0)
extendAlongArc(res::PerTypeCost, l::ForwardLabel, a::Tuple{Int,Int}) =
    (PTFwd(l.state.D + res.dist[a[1], a[2]]), l.state.D + res.dist[a[1], a[2]])
extendAlongArc(res::PerTypeCost, l::BackwardLabel, a::Tuple{Int,Int}) =
    (PTBwd(l.state.D + res.dist[a[2], a[1]]), l.state.D + res.dist[a[2], a[1]])
concatenationCost(res::PerTypeCost, v::Int, f::ForwardLabel, b::BackwardLabel) =
    (PTFwd(f.state.D + b.state.D), f.state.D + b.state.D)

"A solver for a random CVRP; `withCost` puts the distance in `PerTypeCost` instead of in the cost matrix."
function flagsSolver(withCost::Bool; nCust = 14, Q = 30.0, instanceSeed = 3, algorithm = nilsAlgorithm(stop = NILS.ByIterMax(6)))
    dist, demands, Q = randomInstance(nCust; seed = instanceSeed, Q = Q)
    d = arcDemands(demands)
    n = nCust + 1
    custom = withCost ? PerTypeCost(dist) : CustomResource(d, Q)
    res = Resources(custom,
        StandardResource{1}(d, zeros(n), fill(Q, n)),
        StandardResource{2}(zeros(n, n), zeros(n), fill(Inf, n)))
    solver = Solver(res = res,
        data = ProblemData(plainVertices(nCust), withCost ? zeros(n, n) : dist, ceil(Int, sum(demands) / Q) + 3),
        neighborhoods = NEIGHBORHOODS, algorithm = algorithm(res))
    setSeed!(solver, MersenneTwister(2))
    return solver, dist, demands, Q
end

@testset "Resource flags by type" begin
    @testset "a resource without its own flags uses the global ones" begin
        plain = CustomResource(zeros(3, 3), 5.0)
        @test isCostResource(plain) == isCostResource() == false
        @test isSymmetric(plain) == isSymmetric() == false
    end

    @testset "a resource type decides for itself" begin
        @test isCostResource(PerTypeCost(zeros(3, 3)))
        @test !isSymmetric(PerTypeCost(zeros(3, 3)))
        # the global flag still says false for the other resources of the process
        @test !isCostResource()
        a, = flagsSolver(false)
        b, = flagsSolver(true)
        @test canPruneByDist(a)
        @test !canPruneByDist(b)
    end

    @testset "problems with and without a resource cost in the same process" begin
        first = let (solver, dist, demands, Q) = flagsSolver(false)
            quiet(() -> solve!(solver))
            solver.bestFeasSol.cost
        end

        solver, dist, demands, Q = flagsSolver(true)
        quiet(() -> solve!(solver))
        best = solver.bestFeasSol
        routes = [r.visits for r in best.routes if length(r.visits) > 2]
        @test best.cost < Inf
        @test sort(vcat([v[2:end-1] for v in routes]...)) == 1:14
        @test all(v -> sum(demands[v[2:end-1] .+ 1]) <= Q + 1e-6, routes)
        # the cost of the solver is the distance, which only the resource knows
        @test best.cost ≈ sum(sum(dist[v[k]+1, v[k+1]+1] for k in 1:length(v)-1) for v in routes)
        @test objectiveValue(solver, best) ≈ best.cost
        checkConsistency(solver, best)

        # the same with the other algorithm, which has its own Set Partitioning
        ils, = flagsSolver(true; algorithm = ilsAlgorithm())
        quiet(() -> solve!(ils))
        ilsRoutes = [r.visits for r in ils.bestFeasSol.routes if length(r.visits) > 2]
        @test ils.bestFeasSol.cost < Inf
        @test ils.bestFeasSol.cost ≈ sum(sum(dist[v[k]+1, v[k+1]+1] for k in 1:length(v)-1) for v in ilsRoutes)
        @test ils.bestFeasSol.totalLabelCost ≈ ils.bestFeasSol.cost
        checkConsistency(ils, ils.bestFeasSol)

        # solving the other kind of problem before or after does not change the result of the plain one
        again, = flagsSolver(false)
        quiet(() -> solve!(again))
        @test again.bestFeasSol.cost == first
    end

    @testset "the solution of the Set Partitioning keeps the label costs of a cost resource" begin
        for (algorithm, module_) in ((nilsAlgorithm(), GenericILS.NILS), (ilsAlgorithm(), GenericILS.ILS))
            solver, dist, demands, Q = flagsSolver(true; algorithm = algorithm)
            # a feasible solution made greedily is the only route pool, and the incumbent is worse than any solution
            routes, current, load = Vector{Int}[], [0], 0.0
            for c in 1:14
                if load + demands[c+1] > Q
                    push!(routes, vcat(current, 0))
                    current, load = [0], 0.0
                end
                push!(current, c)
                load += demands[c+1]
            end
            push!(routes, vcat(current, 0))
            module_.registerRoutePool!(solver, makeSolution(solver, routes))
            solver.bestFeasSol = newSolution(solver)
            solver.bestFeasSol.cost = Inf
            module_.setPartitioning(solver, Inf)
            best = solver.bestFeasSol
            @test best.cost < Inf
            @test best.totalLabelCost ≈ best.cost                      # the cost matrix is zero: it is all label cost
            checkConsistency(solver, best)
        end
    end
end
