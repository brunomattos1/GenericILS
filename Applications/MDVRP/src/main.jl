include("../../../src/Include.jl")
include("resources.jl")
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
    #D = data.D
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

function Create_Demand_Matrix(data::DataMDVRP)
    n = data.n
    m = data.nb_depots
    d_ = zeros(Float64,n+1,n+1)
    for i=1:n+1
        for j=1:n+1
            if i != j && j != 1
                ## j = 3 cliente 2 que está na posicao 0, 1, 2, 3
                cust = j-1
                d_[i,j] = d(data,cust+m-1)
            end
        end
    end
    return d_
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

function main(data::DataMDVRP, restarts::Int, outerIterMax::Int, innerIterMax::Int, seed::Int)
    n = data.n

    customers = Vector{Vertex}()
    for i = 1:n
        push!(customers, Vertex(i))
    end
    dataHeuristic = ProblemData(customers, zeros(Float64, n+1, n+1), n)

    d_        = buildDemandMatrix(data)
    # dist_     = buildDistMatrix(data)
    # customRes = CustomResource(d_, dist_, data.Q)

    C_0 = CostDepotMatrix(data)
    C_ = CostCustomers(data)
    customRes = CustomResource(C_0,C_) #forncer e modificar de acordo

    stdRes1 = StandardResource{1}(d_, zeros(Float64, n+1), Float64[data.Q for _ in 1:n+1])
    stdRes2 = StandardResource{2}(zeros(Float64, n+1, n+1), zeros(Float64, n+1), fill(Inf, n+1))
    res     = Resources(customRes, stdRes1, stdRes2)

    parameters = Parameters(restarts = restarts, outerIterMax = outerIterMax, innerIterMax = innerIterMax)
    diversif = Diversification(outerShift = 4, outerSwap = 4, innerShift = 2, innerSwap = 2)
    timeRef = (n / 100) * 60
    maxTime = TIME_FACTOR * timeRef
    solver = Solver(
        seed = seed,
        parameters = parameters,
        penaltyManager = TargetRatePenaltyManager(updatePeriod = 100),
        diversification = diversif,
        acceptCriteria = MetropolisTimed(100.0, 100.0, maxTime, 2.0),
        stopCriteria = ByTemperature(0.0001),
        # acceptCriteria = AcceptBest(),
        # stopCriteria = ByIterMax(500),
        res = res,
        data = dataHeuristic,
        neighborhoods = NEIGHBORHOODS,
        MIPSolver = CPLEX.Optimizer,
        timeLimitSP = 60.0
    )
    NILS(solver)
    getBestSol(solver)

    # println("$(data.name) $(round(t, digits=2)) $(round(sol.cost, digits=2))")
    # printSol(sol)
    return solver.statistics, solver.iter
end
# instance = raw"C:\Users\Administrador\Documents\GitHub\GenericILS\Applications\MDVRP\data\P02.mdovrp"
# data = readData(instance)
# main(data, 1, 1, 5, 1)
function collectInstances(dataDir::String)
    instances = String[]
    for file in readdir(dataDir)
        if endswith(file, ".mdovrp")
            push!(instances, joinpath(dataDir, file))
        end
    end
    return instances
end

struct MDVRPJob
    instancePath::String
    instanceName::String
    seed::Int
end

function buildJobs(instances::Vector{String}, seeds::Vector{Int})
    jobs = Vector{MDVRPJob}()
    for instancePath in instances
        instanceName = splitext(basename(instancePath))[1]
        for seed in seeds
            push!(jobs, MDVRPJob(instancePath, instanceName, seed))
        end
    end
    return jobs
end

function runAll()
    restarts     = 1
    outerIterMax = 500
    innerIterMax = 5
    seeds        = [1, 2, 3, 4, 5]

    baseDir = @__DIR__
    dataDir = joinpath(baseDir, "..", "data")
    outCsv  = joinpath(baseDir, "..", "results.csv")

    instances = collectInstances(dataDir)
    jobs      = buildJobs(instances, seeds)
    nJobs     = length(jobs)

    io   = open(outCsv, "w")
    lock = ReentrantLock()
    println(io, "instance,seed,bestCost,bestCostBefSP,tempAtBest,timeAtBest,iterAtBest,totalIter,pool,totalTime")
    flush(io)

    println("Rodando $nJobs jobs em $(Threads.nthreads()) threads")

    Threads.@threads for idx in 1:nJobs
        job = jobs[idx]
        stats     = Statistics()
        totalIter = 99999
        try
            data = readData(job.instancePath)
            stats, totalIter = main(data, restarts, outerIterMax, innerIterMax, job.seed)
        catch e
            println("ERROR on $(job.instanceName) (seed=$(job.seed)): $e")
            stats = Statistics()
            totalIter = -1
        end
        line = "$(job.instanceName),$(job.seed),$(stats.bestFeasCost),$(stats.bestFeasCostBefSP),$(stats.foundTemperature),$(stats.foundTime),$(stats.foundIter),$totalIter,$(stats.poolSize),$(stats.totalTime)"

        Base.lock(lock) do
            println(io, line)
            flush(io)
        end
        println("[thread $(Threads.threadid())] $line")
    end

    close(io)
end

# runAll()

# One-off check: evaluate PyVRP's P01 solution through our REF labels.
data = readData(joinpath(@__DIR__, "..", "data", "P01.mdovrp"))
main(data, 1, 1, 1, 1)

