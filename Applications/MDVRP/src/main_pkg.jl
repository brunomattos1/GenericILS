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
    n = data.n
    V⁺ = [i for i=1:n]

    cost = 0.0
    for r in R
        real_r = Vector{Int64}()
        for k in r.visits
            if k > 0
                push!(real_r,k+m-1)
            end
        end
        isempty(real_r) && continue
        depot = findDepot(data,real_r)
        pushfirst!(real_r, depot)
        push!(real_r, depot)
        push!(Real_R,real_r)
        cost_r = 0.0
        for k=1:length(real_r)-1
            cost_r += c(data,(real_r[k],real_r[k+1]))
        end
        cost += cost_r

    end
    return RealSolution(Real_R,cost)

end

function CostDepotMatrix(data::DataMDVRP)
    n = data.n
    m = data.nb_depots
    D = data.depot_ids
    Cost = zeros(Float64,n,n)
    for i=1:n
        for j=1:n
            Cost[i,j] = c(data,(0,i+m-1)) + c(data,(j+m-1,0))
            for k ∈ D
                if c(data,(k,i+m-1)) + c(data,(j+m-1,k)) < Cost[i,j]
                    Cost[i,j] = c(data,(k,i+m-1)) + c(data,(j+m-1,k))
                end
            end
        end
    end
    return Cost
end

function CostCustomers(data::DataMDVRP)
    n = data.n
    m = data.nb_depots
    Cost = zeros(Float64,n,n)
    for i=1:n
        for j=1:n
            Cost[i,j] = c(data,(i+m-1,j+m-1))
        end
    end
    return Cost
end

function main(data::DataMDVRP, outerIterMax::Int, innerIterMax::Int, seed::Int)
    n = data.n

    customers = Vector{Vertex}()
    for i = 1:n
        push!(customers, Vertex(i))
    end
    dataHeuristic = ProblemData(customers, zeros(Float64, n+1, n+1), n)

    d_ = buildDemandMatrix(data)

    C_0 = CostDepotMatrix(data)
    C_ = CostCustomers(data)
    customRes = CustomResource(C_0,C_)

    stdRes1 = StandardResource{1}(d_, zeros(Float64, n+1), Float64[data.Q for _ in 1:n+1])
    stdRes2 = StandardResource{2}(zeros(Float64, n+1, n+1), zeros(Float64, n+1), fill(Inf, n+1))
    res     = Resources(customRes, stdRes1, stdRes2)

    timeRef = (n / 100) * 60
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
        penaltyManager = TargetRatePenaltyManager(updatePeriod = 100),
        MIPSolver = CPLEX.Optimizer
    )

    setSeed!(solver, Random.MersenneTwister(seed))

    solve!(solver)
    getBestSol(solver)

    return solver.statistics, solver.algorithm.iter
end

data = readData(joinpath(@__DIR__, "..", "data", "P01.mdovrp"))
main(data, 1, 1, 1)
