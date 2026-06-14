include("../../../src/Include.jl")
include("resources.jl")
include("data.jl")

function printSol(sol::Solution)
    for r = 1:length(sol.routes)
        print("route $r: ")
        for v in sol.routes[r]
            print("$v ")
        end
        println()
    end
    println("Cost: $(round(sol.cost, digits=2))")
end

function main(data::DataVRPSPD, restarts::Int, outerIterMax::Int, innerIterMax::Int, seed::Int)
    n = data.n
    Q = data.Q

    customers = Vector{Vertex}()
    for i = 1:n
        push!(customers, Vertex(i, [(0, 0)]))
    end
    dataHeuristic = ProblemData(customers, buildDistMatrix(data), 10)

    p, d      = createPdDemands(data)
    customRes = CustomResource(p, d, Q)

    stdRes1 = StandardResource{1}(p, zeros(Float64, n+1), Float64[Q for _ in 1:n+1])
    stdRes2 = StandardResource{2}(zeros(Float64, n+1, n+1), zeros(Float64, n+1), fill(Inf, n+1))
    res     = Resources(customRes, stdRes1, stdRes2)

    parameters = Parameters(restarts = restarts, outerIterMax = outerIterMax, innerIterMax = innerIterMax,
        penaltyCustom = 1.0, penaltyCustomIncrease = 0.01, penaltyCustomDecrease = 0.01,
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

    t   = @elapsed NILS(solver)
    sol = getBestSol(solver)

    println("$(data.name) $(round(t, digits=2)) $(round(sol.cost, digits=2))")
    printSol(sol)
    return sol.cost
end

instance     = "path/to/instance"
restarts     = 1
outerIterMax = 500
innerIterMax = 5
seed         = 1

data = readData(instance)
main(data, restarts, outerIterMax, innerIterMax, seed)
