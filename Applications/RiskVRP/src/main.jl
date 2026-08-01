include("../../../src/Include.jl")
include("resources.jl")
include("data.jl")
using CPLEX

const TIME_FACTOR = 10.0

function loadLiteratureTimes(path::String)
    times = Dict{String, Float64}()
    open(path) do io
        header = true
        for line in eachline(io)
            if header
                header = false
                continue
            end
            isempty(strip(line)) && continue
            fields = split(line, ',')
            times[fields[1]] = parse(Float64, fields[end])
        end
    end
    return times
end

const LITERATURE_TIMES = loadLiteratureTimes(joinpath(@__DIR__, "..", "..", "..", "literature", "RiskVRP_literature.csv"))

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

function main(data::DataRiskVRP, restarts::Int, outerIterMax::Int, innerIterMax::Int, seed::Int)
    n    = data.n
    Rmax = data.T

    customers = Vector{Vertex}()
    for i = 1:n
        push!(customers, Vertex(i))
    end
    dataHeuristic = ProblemData(customers, buildDistMatrix(data), n)

    dt        = createDtDemands(data)
    customRes = CustomResource(dt, buildDistMatrix(data), Float64(Rmax))

    stdRes1 = StandardResource{1}(zeros(Float64, n+1, n+1), zeros(Float64, n+1), fill(Inf, n+1))
    stdRes2 = StandardResource{2}(zeros(Float64, n+1, n+1), zeros(Float64, n+1), fill(Inf, n+1))
    res     = Resources(customRes, stdRes1, stdRes2)

    parameters = Parameters(restarts = restarts, outerIterMax = outerIterMax, innerIterMax = innerIterMax)
    diversif = Diversification(outerShift = 3, outerSwap = 3, innerShift = 1, innerSwap = 1)
    timeRef = get(LITERATURE_TIMES, data.name, 45.0)
    maxTime = TIME_FACTOR * timeRef
    solver = Solver(
        seed = seed,
        parameters = parameters,
        penaltyManager = TargetRatePenaltyManager(penaltyCustom = 1000.0 ;updatePeriod = 100),
        diversification = diversif,
        acceptCriteria = MetropolisTimed(100.0, 100.0, maxTime, 2.0),
        stopCriteria = ByTemperature(0.001),
        # acceptCriteria = AcceptBest(),
        # stopCriteria = ByIterMax(500),
        res = res,
        data = dataHeuristic,
        neighborhoods = NEIGHBORHOODS,
        MIPSolver = CPLEX.Optimizer,
        timeLimitSP = 60.0
    )

    NILS(solver)
    sol = getBestSol(solver)
    # printSol(sol)
    # println("$(data.name) $(round(t, digits=2)) $(round(sol.cost, digits=2))")
    return solver.statistics, solver.iter
end

function collectInstances(dataDir::String)
    instances = String[]
    for folder in ("O", "S", "V")
        folderPath = joinpath(dataDir, folder)
        for file in readdir(folderPath)
            if endswith(file, ".rctvrp")
                push!(instances, joinpath(folderPath, file))
            end
        end
    end
    return instances
end

struct RiskJob
    instancePath::String
    instanceName::String
    seed::Int
end

function buildJobs(instances::Vector{String}, seeds::Vector{Int})
    jobs = Vector{RiskJob}()
    for instancePath in instances
        instanceName = splitext(basename(instancePath))[1]
        for seed in seeds
            push!(jobs, RiskJob(instancePath, instanceName, seed))
        end
    end
    return jobs
end

function runAll()
    restarts     = 1
    outerIterMax = 500
    innerIterMax = 5
    seeds        = [1, 2, 3, 4, 5]

    baseDir  = @__DIR__
    dataDir  = joinpath(baseDir, "..", "data")
    outCsv   = joinpath(baseDir, "..", "results.csv")

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

runAll()
# data = readData(raw"C:\Users\Administrador\Documents\GitHub\GenericILS\Applications\RiskVRP\data\V\121_1.5.rctvrp")
# stats, totalIter = main(data, 1, 1, 5, 1)
