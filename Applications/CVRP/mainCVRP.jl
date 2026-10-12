using GenericILS
using Random
include("resourcesCVRP_pkg.jl")
ENV["JULIA_HASH_SEED"] = "0"
Random.seed!(0)

using CVRPLIB
using CPLEX

function printCVRP(solver::Solver, sol::Solution)
    for r = 1:length(sol.routes)
        demand = 0.
        time = 0.
        print("#$r: ")
        for i = 1:length(sol.routes[r].visits)
            if i == 1
                print("0 (0) -> ")
            elseif i == length(sol.routes[r].visits)
                time += solver.res.stdResource1.d[sol.routes[r].visits[i-1] + 1, sol.routes[r].visits[i] + 1]
                print("0 ($demand)")
            else
                time += solver.res.stdResource1.d[sol.routes[r].visits[i-1] + 1, sol.routes[r].visits[i] + 1]
                demand += solver.res.customResource.d[sol.routes[r].visits[i-1] + 1, sol.routes[r].visits[i] + 1]
                print("$(sol.routes[r].visits[i]) ($demand) -> ")
            end
        end
        println()
    end
    println("\nCost: $(sol.dist)")
end

function createArcDemands(demands)
    n = length(demands)
    d = zeros(Float64, n, n)
    for i in 1:n
        for j in 1:n
            if i != j
                if j == 1
                    d[i, j] = 0
                else
                    d[i, j] = demands[j]
                end
            end
        end
    end
    return d
end

function main(instance::String, restarts::Int, outerIterMax::Int, innerIterMax::Int, seed::Int)
    instance = joinpath(normpath(joinpath(@__DIR__, "data")), string(instance[1]), instance)

    cvrp = CVRPLIB.readCVRP(instance)
    dist = Float64.(cvrp.weights)
    customers = Vector{Vertex}()
    nbCustomer = size(dist)[1] - 1
    for i = 1:nbCustomer
        push!(customers, Vertex(i))
    end
    maxNbRoute = ceil(Int, sum(cvrp.demand) / cvrp.capacity) + 3
    demands = push!(Float64.(cvrp.demand), 0.0)
    d = createArcDemands(demands)
    customRes = CustomResource(d, cvrp.capacity)
    n = length(customers) + 1
    stdRes1 = StandardResource{1}(d, zeros(Float64, n), Float64[cvrp.capacity for _ in 1:n])
    stdRes2 = StandardResource{2}(zeros(Float64, n, n), zeros(Float64, n), fill(Inf, n))

    res = Resources(customRes, stdRes1, stdRes2)

    algorithm = NILSAlgorithm(res;
        acceptCriteria = NILS.AcceptBest(),
        stopCriteria = NILS.ByIterMax(outerIterMax),
        diversification = NILS.Diversification(outerShift = 1, outerSwap = 0, innerShift = 2, innerSwap = 0),
        innerIterMax = innerIterMax
    )

    solver = Solver(
        res = res,
        data = ProblemData(customers, dist, maxNbRoute),
        neighborhoods = (
            TwoOptStar(),
            IntraShift(),
            InterShift{1}(),
            InterShift{2}(),
            InterSwap{1, 1}(),
            InterSwap{2, 1}(),
            InterSwap{2, 2}()
        ),
        algorithm = algorithm
    )

    setSeed!(solver, Random.MersenneTwister(seed))

    setPenaltyManager!(solver, StandardPenaltyManager(
        penaltyCustom = 1.0, penaltyCustomIncrease = 0.01, penaltyCustomDecrease = 0.01,
        penaltyStandard1 = 1.0, penaltyStandard1Increase = 0.01, penaltyStandard1Decrease = 0.01,
        penaltyStandard2 = 0.0, penaltyStandard2Increase = 0.0, penaltyStandard2Decrease = 0.0
    ))

    setMIPSolver!(solver, CPLEX.Optimizer)

    @time solve!(solver)
    sol = getBestSol(solver)
    printCVRP(solver, solver.algorithm.outerBestSol)
    return sol.cost
end

set = "A"
n = 37
k = 5
instance = "$set-n$n-k$k.vrp"
seed = 2
restarts = 1
outerIterMax = 200
innerIterMax = 5

main(instance, restarts, outerIterMax, innerIterMax, seed)
