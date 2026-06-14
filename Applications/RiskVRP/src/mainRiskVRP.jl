include("../../../src/Include.jl")
include("resources.jl")
include("data.jl")
ENV["JULIA_HASH_SEED"] = "0"
using HiGHS
# using PlotlyJS
# using CPLEX

function printCVRP(solver::Solver, sol::Solution)
    for r = 1:length(sol.routes)
        demand = 0.
        risk = 0.
        print("#$r: ")
        for i = 1:length(sol.routes[r])
            if i == 1
                print("0 (0) -> ")
            elseif i == length(sol.routes[r])
                print("0 ($demand)")
            else
                demand += solver.res.customResource.dt[sol.routes[r][i-1] + 1, sol.routes[r][i] + 1]
                risk   += solver.res.customResource.c[sol.routes[r][i-1] + 1, sol.routes[r][i] + 1]
                print("$(sol.routes[r][i]) ($demand) -> ")
            end
        end
        println()
    end
    println("\nCost: $(sol.dist)")
end

function printSol(sol::Solution)
    R = sol.routes
    for k=1:length(R)
        print("route $(k) : ")
        for i ∈ R[k]
            print("$i ")
        end
        println()
    end
end

function main(data::DataCVRP, restarts::Int, outerIterMax::Int, innerIterMax::Int, seed::Int)
    n    = data.n
    Rmax = data.T

    customers = Vector{Vertex}()
    for i = 1:n
        push!(customers, Vertex(i, [(0, 0)]))
    end

    dataHeuristic = ProblemData(customers, BuildDistMatrix(data), n)

    dt = CreateDtDemands(data)
    customRes = CustomResource(dt, BuildDistMatrix(data), Rmax)

    # Loose first standard resource (no constraint)
    stdRes1 = StandardResource{1}(zeros(Float64, n+1, n+1), zeros(Float64, n+1), fill(Inf, n+1))
    # Loose second standard resource (no constraint)
    stdRes2 = StandardResource{2}(zeros(Float64, n+1, n+1), zeros(Float64, n+1), fill(Inf, n+1))

    res = Resources(customRes, stdRes1, stdRes2)

    parameters = Parameters(restarts = restarts, outerIterMax = outerIterMax, innerIterMax = innerIterMax,
        penaltyCustom = 100.0, penaltyCustomIncrease = 0.01, penaltyCustomDecrease = 0.01,
        penaltyStandard1 = 1.0, penaltyStandard1Increase = 0.01, penaltyStandard1Decrease = 0.01,
        penaltyStandard2 = 0.0, penaltyStandard2Increase = 0.0, penaltyStandard2Decrease = 0.0
    )

    diversif = Diversification(outerShift = 2, outerSwap = 0, innerShift = 2, innerSwap = 0)

    solver = Solver(
        seed = seed,
        parameters = parameters,
        diversification = diversif,
        acceptCriteria = AcceptBest(),
        stopCriteria = ByIterMax(outerIterMax),
        res = res,
        data = dataHeuristic,
        neighborhoods = NEIGHBORHOODS
    )
    t = @elapsed NILS(solver)
    sol = getBestSol(solver)

    println("$(data.name) $(round(t, digits=2)) $(round(solver.outerBestSol.cost, digits=2))")
    printSol(solver.outerBestSol)
    return sol.cost
end

instance = raw"C:\Users\bruna\OneDrive\Documentos\GitHub\ApplicationsGenericILS\RiskVRP\data\O\O137.rctvrp"
restarts     = 1
outerIterMax = 500
innerIterMax = 5
seed         = 1

data = readCVRPData(instance)
main(data, restarts, outerIterMax, innerIterMax, seed)
