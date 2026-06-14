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

function main(data::DataHVRP, restarts::Int, outerIterMax::Int, innerIterMax::Int, seed::Int)
    n  = nbCustomers(data)
    nv = length(data.veh_types)
    Q  = [vehCapacity(data, k) for k = 1:nv]

    customers = Vector{Vertex}()
    for i = 1:n
        push!(customers, Vertex(i, [(0, 0)]))
    end
    dataHeuristic = ProblemData(customers, zeros(Float64, n+1, n+1), 20)

    d_        = buildDemandMatrix(data)
    customRes = CustomResource(nv, [vehFixed(data, k) for k = 1:nv],
                                   [vehFactor(data, k) for k = 1:nv],
                                   Float64.(Q), d_, buildDistMatrix(data))

    Qmax    = maximum(Q)
    stdRes1 = StandardResource{1}(d_, zeros(Float64, n+1), Float64[Qmax for _ in 1:n+1])
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

brandao      = true
instance     = raw"C:\Users\bruna\OneDrive\Documentos\GitHub\GenericILS\Applications\HFVRP\data\brandaoN1hd.txt"
restarts     = 1
outerIterMax = 500
innerIterMax = 5
seed         = 1

data = brandao ? readBrandaoData(instance) : readClassicData(instance)
main(data, restarts, outerIterMax, innerIterMax, seed)
