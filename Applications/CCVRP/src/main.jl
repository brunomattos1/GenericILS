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

function main(data::DataCCVRP, restarts::Int, outerIterMax::Int, innerIterMax::Int, seed::Int)
    dist = data.weights
    n    = size(dist, 1) - 1

    customers = Vector{Vertex}()
    for i = 1:n
        push!(customers, Vertex(i, [(0, 0)]))
    end
    maxNbRoute    = ceil(Int, sum(data.demand) / data.capacity)
    dataHeuristic = ProblemData(customers, zeros(Float64, n+1, n+1), maxNbRoute)

    demands   = push!(copy(data.demand), 0.0)
    d         = buildArcDemands(demands)
    customRes = CustomResource(demands, dist, 1000.0, 1000.0)

    nv     = n + 1
    stdRes1 = StandardResource{1}(d, zeros(Float64, nv), Float64[data.capacity for _ in 1:nv])
    stdRes2 = StandardResource{2}(zeros(Float64, nv, nv), zeros(Float64, nv), fill(Inf, nv))
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
    setTimeLimitILS(solver, 30.0)
    setTimeLimitSP(solver, 50.0)
    aggressivePool(solver, false)

    t   = @elapsed NILS(solver)
    sol = getBestSol(solver)

    println("$(data.name) $(round(t, digits=2)) $(round(sol.cost, digits=2))")
    printSol(sol)
    return sol.cost
end

instance     = "P-n19-k2.vrp"
restarts     = 1
outerIterMax = 500
innerIterMax = 30
seed         = 1

data = readData(instance)
main(data, restarts, outerIterMax, innerIterMax, seed)
