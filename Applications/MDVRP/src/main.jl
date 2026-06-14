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

struct RealSolution
    routes::Vector{Vector{Int64}}
    cost::Float64
end

function findDepot(data::DataMDVRP,r::Vector{Int64})
    depot = 0
    cost = 100000.0
    for k in data.depot_ids
        aux = c(data,(k,r[1])) + c(data,(r[end],k))
        if aux < cost
            cost = aux
            depot = k
        end
    end
    return depot
end

function realSol(data::DataMDVRP, sol::Solution)
    R = sol.routes
    Real_R = Vector{Vector{Int64}}()
    Q = data.Q
    m = data.nb_depots
    #D = data.D
    n = data.n
    V⁺ = [i for i=1:n]

    cost = 0.0
    for r in R
        real_r = Vector{Int64}()
        for k in r
            if k > 0
                push!(real_r,k+m-1)
            end
        end
        depot = findDepot(data,real_r)
        pushfirst!(real_r, depot)
        push!(real_r, depot)
        #@show real_r
        push!(Real_R,real_r)
        cost_r = 0.0
        for k=1:length(real_r)-1
            cost_r += c(data,(real_r[k],real_r[k+1]))
        end
        cost += cost_r

    end
    return RealSolution(Real_R,cost)

end

function main(data::DataMDVRP, restarts::Int, outerIterMax::Int, innerIterMax::Int, seed::Int)
    n = data.n

    customers = Vector{Vertex}()
    for i = 1:n
        push!(customers, Vertex(i))
    end
    dataHeuristic = ProblemData(customers, zeros(Float64, n+1, n+1), n)

    d_        = buildDemandMatrix(data)
    dist_     = buildDistMatrix(data)
    customRes = CustomResource(d_, dist_, data.Q)

    stdRes1 = StandardResource{1}(d_, zeros(Float64, n+1), Float64[data.Q for _ in 1:n+1])
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
    realSol(data, sol)
    return sol.cost
end

instance     = raw"C:\Users\bruna\OneDrive\Documentos\GitHub\GenericILS\Applications\MDVRP\data\PR02.mdovrp"
restarts     = 1
outerIterMax = 500
innerIterMax = 5
seed         = 1

data = readData(instance)
main(data, restarts, outerIterMax, innerIterMax, seed)
