using GenericILS
using Random
include("resources_pkg.jl")
include("data.jl")

using CPLEX

const TIME_FACTOR = 10.0

function printSol(sol::Solution)
    for r = 1:length(sol.routes)
        print("route $r: ")
        for v in sol.routes[r].visits
            print("$v ")
        end
        println()
    end
    println("Cost: $(round(sol.cost, digits=2))")
end

function buildDemandMatrix(data::DataHVRP)
    n  = nbCustomers(data)
    d_ = zeros(Float64, n+1, n+1)
    for i = 1:n+1, j = 1:n+1
        if i != j && j != 1
            d_[i, j] = demand(data, j-1)
        end
    end
    return d_
end

function buildDistMatrix(data::DataHVRP)
    n    = length(data.G′.V′)
    dist = zeros(Float64, n, n)
    for i = 1:n, j = 1:n
        dist[i, j] = euclideanDistance(data, i-1, j-1)
    end
    return dist
end

function main(data::DataHVRP, outerIterMax::Int, innerIterMax::Int, seed::Int, timeRef::Float64)
    n  = nbCustomers(data)
    nv = length(data.veh_types)
    Q  = [vehCapacity(data, k) for k = 1:nv]

    customers = Vector{Vertex}()
    for i = 1:n
        push!(customers, Vertex(i))
    end
    d_   = buildDemandMatrix(data)
    Qmax = maximum(Q)

    dataHeuristic = ProblemData(customers, zeros(Float64, n+1, n+1), n)

    customRes = CustomResource(nv, [vehFixed(data, k) for k = 1:nv],
                                   [vehFactor(data, k) for k = 1:nv],
                                   Float64.(Q), d_, buildDistMatrix(data))

    stdRes1 = StandardResource{1}(d_, zeros(Float64, n+1), Float64[Qmax for _ in 1:n+1])
    stdRes2 = StandardResource{2}(zeros(Float64, n+1, n+1), zeros(Float64, n+1), fill(Inf, n+1))
    res     = Resources(customRes, stdRes1, stdRes2)

    maxTime = TIME_FACTOR * timeRef

    algorithm = NILSAlgorithm(res;
        acceptCriteria = NILS.MetropolisTimed(100.0, 100.0, maxTime, 2.0),
        stopCriteria = NILS.ByTemperature(0.0001),
        diversification = NILS.Diversification(outerShift = 4, outerSwap = 4, innerShift = 2, innerSwap = 2),
        innerIterMax = innerIterMax,
        timeLimitSP = 60.0
    )

    solver = Solver(
        res = res,
        data = dataHeuristic,
        neighborhoods = (
            TwoOptStar(),
            IntraShift(),
            InterShift{1}(),
            InterShift{2}(),
            InterSwap{1, 1}(),
            InterSwap{2, 1}(),
            InterSwap{2, 2}()
        ),
        algorithm = algorithm,
        penaltyManager = TargetRatePenaltyManager(
            penaltyCustom = 10.0,
            penaltyStandard1 = 10.0,
            updatePeriod = 50),
        MIPSolver = CPLEX.Optimizer
    )

    setSeed!(solver, Random.MersenneTwister(seed))

    solve!(solver)
    sol = getBestSol(solver)

    return solver.statistics, solver.algorithm.iter
end

brandao      = false
instance     = joinpath(@__DIR__, "..", "data", "c50_15fsmfd.txt")
outerIterMax = 500
innerIterMax = 5
seed         = 1

data = brandao ? readBrandaoData(instance) : readClassicData(instance)
main(data, outerIterMax, innerIterMax, seed, 19.0)
