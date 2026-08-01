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


# Lower bound on the number of vehicles needed: total demand divided by the
# largest available capacity, rounded up. Only a valid LB when vehicle types
# are "well-behaved" (larger capacity => fixed cost and per-km factor no
# smaller), i.e. Q_k < Q_k' implies fixed_k <= fixed_k' and factor_k <= factor_k',
# so using the biggest vehicles never costs more per unit of demand carried.
function minFleetSize(data::DataHVRP)
    n = nbCustomers(data)
    totalDemand = sum(demand(data, i) for i = 1:n)
    Qmax = minimum(vehCapacity(data, k) for k = 1:length(data.veh_types))
    return ceil(Int, totalDemand / Qmax)
end

function main(data::DataHVRP, restarts::Int, outerIterMax::Int, innerIterMax::Int, seed::Int, timeRef::Float64)
    n  = nbCustomers(data)
    nv = length(data.veh_types)
    Q  = [vehCapacity(data, k) for k = 1:nv]

    customers = Vector{Vertex}()
    for i = 1:n
        push!(customers, Vertex(i))
    end
    d_        = buildDemandMatrix(data)
    Qmax    = maximum(Q)

    dataHeuristic = ProblemData(customers, zeros(Float64, n+1, n+1), n)

    customRes = CustomResource(nv, [vehFixed(data, k) for k = 1:nv],
                                   [vehFactor(data, k) for k = 1:nv],
                                   Float64.(Q), d_, buildDistMatrix(data))

    stdRes1 = StandardResource{1}(d_, zeros(Float64, n+1), Float64[Qmax for _ in 1:n+1])
    stdRes2 = StandardResource{2}(zeros(Float64, n+1, n+1), zeros(Float64, n+1), fill(Inf, n+1))
    res     = Resources(customRes, stdRes1, stdRes2)

    parameters = Parameters(restarts = restarts, outerIterMax = outerIterMax, innerIterMax = innerIterMax)
    diversif = Diversification(outerShift = 4, outerSwap = 4, innerShift = 2, innerSwap = 2)
    maxTime = TIME_FACTOR * timeRef
    solver = Solver(
        seed = seed,
        parameters = parameters,
        penaltyManager = TargetRatePenaltyManager(
            penaltyCustom = 10.0, 
            penaltyStandard1 = 10.0,
            updatePeriod = 50),
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
    # getBestSol(solver)

    # println("$(data.name) $(round(t, digits=2)) $(round(sol.cost, digits=2))")
    # printSol(sol)
    return solver.statistics, solver.iter
end

function collectInstances(dataDir::String)
    instances = String[]
    for file in readdir(dataDir)
        if endswith(file, ".txt")
            push!(instances, joinpath(dataDir, file))
        end
    end
    return instances
end

struct HFVRPJob
    instancePath::String
    instanceName::String
    seed::Int
end

function buildJobs(instances::Vector{String}, seeds::Vector{Int})
    jobs = Vector{HFVRPJob}()
    for instancePath in instances
        instanceName = splitext(basename(instancePath))[1]
        for seed in seeds
            push!(jobs, HFVRPJob(instancePath, instanceName, seed))
        end
    end
    return jobs
end

# Golden-derived instances (c50/c75/c100 prefixes) use the "classic" file
# format; Brandao (2011) instances use the "brandao" format.
function readInstance(path::String)
    name = basename(path)
    return startswith(name, "brandao") ? readBrandaoData(path) : readClassicData(path)
end

# Maps instance name (e.g. "c50_13fsmfd") -> AILS TimeRef (seconds), read from
# literature/HFVRP_literature.csv, so our time budget (TIME_FACTOR * TimeRef)
# scales with the reference algorithm's actual reported running time rather
# than a generic n-based formula.
function readTimeRefs(litCsv::String)
    timeRefs = Dict{String, Float64}()
    for line in readlines(litCsv)[2:end]
        fields = split(line, ',')
        timeRefs[fields[1]] = parse(Float64, fields[6])
    end
    return timeRefs
end

function runJobs(jobs::Vector{HFVRPJob}, timeRefs::Dict{String, Float64}, outCsv::String;
                  restarts::Int = 1, outerIterMax::Int = 500, innerIterMax::Int = 5,
                  writeHeader::Bool = true)
    nJobs = length(jobs)

    io   = open(outCsv, writeHeader ? "w" : "a")
    lock = ReentrantLock()
    if writeHeader
        println(io, "instance,seed,bestCost,bestCostBefSP,tempAtBest,timeAtBest,iterAtBest,totalIter,pool,totalTime")
        flush(io)
    end

    println("Rodando $nJobs jobs em $(Threads.nthreads()) threads")

    Threads.@threads for idx in 1:nJobs
        job = jobs[idx]
        stats     = Statistics()
        totalIter = 99999
        try
            data    = readInstance(job.instancePath)
            timeRef = timeRefs[job.instanceName]
            stats, totalIter = main(data, restarts, outerIterMax, innerIterMax, job.seed, timeRef)
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

function runAll()
    seeds = [1, 2, 3, 4, 5]

    baseDir = @__DIR__
    dataDir = joinpath(baseDir, "..", "data")
    outCsv  = joinpath(baseDir, "..", "results.csv")
    litCsv  = joinpath(baseDir, "..", "..", "..", "literature", "HFVRP_literature.csv")

    timeRefs  = readTimeRefs(litCsv)
    instances = collectInstances(dataDir)
    jobs      = buildJobs(instances, seeds)

    runJobs(jobs, timeRefs, outCsv; writeHeader = true)
end

# Re-runs a single instance (all seeds) and appends the rows to the existing
# results.csv, without touching results already collected for other
# instances. Useful after fixing a bug that only affected specific instances
# (e.g. non-integer coordinates in brandaoN5fsmd).
function runOne(instanceName::String; seeds::Vector{Int} = [1, 2, 3, 4, 5])
    baseDir = @__DIR__
    dataDir = joinpath(baseDir, "..", "data")
    outCsv  = joinpath(baseDir, "..", "results.csv")
    litCsv  = joinpath(baseDir, "..", "..", "..", "literature", "HFVRP_literature.csv")

    timeRefs     = readTimeRefs(litCsv)
    instancePath = joinpath(dataDir, "$instanceName.txt")
    jobs         = [HFVRPJob(instancePath, instanceName, s) for s in seeds]

    runJobs(jobs, timeRefs, outCsv; writeHeader = !isfile(outCsv))
end

# runAll()
runOne("brandaoN5fsmd")

# brandao      = false
# instance     = raw"C:\Users\Administrador\Documents\GitHub\GenericILS\Applications\HFVRP\data\c50_15fsmfd.txt"
# restarts     = 1
# outerIterMax = 500
# innerIterMax = 5
# seed         = 1

# data = brandao ? readBrandaoData(instance) : readClassicData(instance)
# main(data, restarts, outerIterMax, innerIterMax, seed, 19.0)
