include("../../../src/Include.jl")
include("resources.jl")
include("data.jl")
using CPLEX

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
    diversif = Diversification(outerShift = 2, outerSwap = 0, innerShift = 1, innerSwap = 0)
    solver = Solver(
        seed = seed,
        parameters = parameters,
        penaltyManager = TargetRatePenaltyManager(),
        diversification = diversif,
        acceptCriteria = Metropolis(100.0, 0.992),
        stopCriteria = ByTemperature(0.1),
        # acceptCriteria = AcceptBest(),
        # stopCriteria = ByIterMax(500),
        res = res,
        data = dataHeuristic,
        neighborhoods = NEIGHBORHOODS,
        MIPSolver = CPLEX.Optimizer,
        timeLimitSP = 30.0
    )

    t        = @elapsed NILS(solver)
    sol      = getBestSol(solver)
    poolSize = length(solver.route_storage)

    println("$(data.name) $(round(t, digits=2)) $(round(sol.cost, digits=2))")
    return sol.cost, t, poolSize
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

function runAll()
    restarts     = 1
    outerIterMax = 500
    innerIterMax = 5
    seeds        = [1, 2, 3, 4, 5]

    baseDir  = @__DIR__
    dataDir  = joinpath(baseDir, "..", "data")
    outCsv   = joinpath(baseDir, "..", "results.csv")

    instances = collectInstances(dataDir)

    open(outCsv, "w") do io
        println(io, "instance,cost,time,pool")
        flush(io)

        for instancePath in instances
            for seed in seeds
                instanceName = splitext(basename(instancePath))[1]
                cost = 99999
                t    = 99999
                pool = 99999
                try
                    data = readData(instancePath)
                    cost, t, pool = main(data, restarts, outerIterMax, innerIterMax, seed)
                catch e
                    println("ERROR on $instanceName (seed=$seed): $e")
                    cost = 99999
                    t    = 99999
                    pool = 99999
                end
                println(io, "$instanceName,$cost,$t,$pool")
                flush(io)
            end
        end
    end
end

runAll()
# data = readData(raw"C:\Users\Administrador\Documents\GitHub\GenericILS\Applications\RiskVRP\data\O\O103.rctvrp")
# cost, t = main(data, 1, 1, 5, 1)
