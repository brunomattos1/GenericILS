include("../../../src/Include.jl")
include("resources.jl")
include("data.jl")
using CPLEX
# ---------------------------------------------------------------------------
# Script de calibragem de alpha (resfriamento do Metropolis) e innerIterMax
# Instancias: sn4x, sn4y, cc0582, R1_2_1, C1_2_1, RC2_2_1
# Cada combinacao (instancia, alpha, innerIterMax) e executada 3x (seeds 1,2,3)
# ---------------------------------------------------------------------------

const DATA_DIR = raw"C:\Users\Administrador\Documents\GitHub\GenericILS\Applications\VRPSPD\data"

const INSTANCES = [
    joinpath(DATA_DIR, "SN", "sn4x"),
    joinpath(DATA_DIR, "SN", "sn4y"),
    joinpath(DATA_DIR, "D",  "cc0582"),
    joinpath(DATA_DIR, "MG", "R1_2_1"),
    joinpath(DATA_DIR, "MG", "C1_2_1"),
    joinpath(DATA_DIR, "MG", "RC2_2_1"),
]

# Valores a testar - editar livremente
const ALPHAS         = [0.995]
const INNER_ITER_MAX = [5]

const SEEDS    = [1, 2, 3]
const RESTARTS = 1
const OUTER_ITER_MAX = 500

const OUTPUT_FILE = joinpath(@__DIR__, "..", "calibration_results.csv")

function runInstance(data::DataVRPSPD, alpha::Float64, innerIterMax::Int, seed::Int)
    n = data.n
    Q = data.Q

    customers = Vector{Vertex}()
    for i = 1:n
        push!(customers, Vertex(i))
    end
    dataHeuristic = ProblemData(customers, buildDistMatrix(data), ceil(Int, length(customers) / 2))

    p, d      = createPdDemands(data)
    customRes = CustomResource(p, d, Q)

    stdRes1 = StandardResource{1}(p, zeros(Float64, n+1), Float64[Q for _ in 1:n+1])
    stdRes2 = StandardResource{2}(zeros(Float64, n+1, n+1), zeros(Float64, n+1), fill(Inf, n+1))
    res     = Resources(customRes, stdRes1, stdRes2)

    parameters = Parameters(restarts = RESTARTS, outerIterMax = OUTER_ITER_MAX, innerIterMax = innerIterMax)
    diversif   = Diversification(outerShift = 4, outerSwap = 0, innerShift = 2, innerSwap = 0)
    pm = TargetRatePenaltyManager(
        penaltyCustom = 10.0,
        penaltyCustomIncrease = 0.4,
        penaltyCustomDecrease = 0.2,
        penaltyStandard1 = 1.0,
        penaltyStandard1Increase = 0.4,
        penaltyStandard1Decrease = 0.2,
        penaltyStandard2 = 1.0,
        penaltyStandard2Increase = 0.4,
        penaltyStandard2Decrease = 0.2,
        targetFeasRate = 0.7,
        feasRateTolerance = 0.05,
        updatePeriod = 50
    )
    solver = Solver(
        seed = seed,
        parameters = parameters,
        diversification = diversif,
        penaltyManager = pm,
        acceptCriteria = Metropolis(100.0, alpha),
        stopCriteria = ByTemperature(0.1),
        res = res,
        data = dataHeuristic,
        neighborhoods = NEIGHBORHOODS,
        MIPSolver = CPLEX.Optimizer,
        timeLimitSP = 30.0
    )

    t   = @elapsed NILS(solver)
    sol = getBestSol(solver)

    return sol.cost, t
end

struct Job
    instancePath::String
    instanceName::String
    alpha::Float64
    innerIterMax::Int
    seed::Int
end

function buildJobs()
    jobs = Vector{Job}()
    for instancePath in INSTANCES
        instanceName = splitext(basename(instancePath))[1]
        for alpha in ALPHAS, innerIterMax in INNER_ITER_MAX, seed in SEEDS
            push!(jobs, Job(instancePath, instanceName, alpha, innerIterMax, seed))
        end
    end
    return jobs
end

function main()
    jobs = buildJobs()
    nJobs = length(jobs)

    # Cache dos dados de cada instancia, carregados uma vez por caminho
    dataCache = Dict{String, DataVRPSPD}()
    for instancePath in INSTANCES
        dataCache[instancePath] = readData(instancePath)
    end

    io   = open(OUTPUT_FILE, "w")
    lock = ReentrantLock()
    println(io, "instance,alpha,innerIterMax,seed,cost,time")
    flush(io)

    println("Rodando $nJobs jobs em $(Threads.nthreads()) threads")

    for idx in 1:nJobs
        job  = jobs[idx]
        data = dataCache[job.instancePath]

        println("-> [thread $(Threads.threadid())] iniciando [$(job.instanceName)] alpha=$(job.alpha) innerIterMax=$(job.innerIterMax) seed=$(job.seed)")

        cost, t = runInstance(data, job.alpha, job.innerIterMax, job.seed)

        line = "$(job.instanceName),$(job.alpha),$(job.innerIterMax),$(job.seed),$(round(cost, digits=2)),$(round(t, digits=2))"

        Base.lock(lock) do
            println(io, line)
            flush(io)
        end
        println("[thread $(Threads.threadid())] [$(job.instanceName)] alpha=$(job.alpha) innerIterMax=$(job.innerIterMax) seed=$(job.seed) -> cost=$(round(cost, digits=2)) time=$(round(t, digits=2))s")
    end

    close(io)
    println("Resultados salvos em $OUTPUT_FILE")
end

main()
