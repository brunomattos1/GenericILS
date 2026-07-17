include("../../../src/Include.jl")
include("resources.jl")
include("data.jl")
using CPLEX
function printSol(sol::Solution)
    for r = 1:length(sol.routes)
        print("Route #$r: ")
        for v in sol.routes[r].visits
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
        push!(customers, Vertex(i))
    end
    dataHeuristic = ProblemData(customers, buildDistMatrix(data), ceil(Int, length(customers) / 2))

    p, d      = createPdDemands(data)
    customRes = CustomResource(p, d, Q)

    stdRes1 = StandardResource{1}(p, zeros(Float64, n+1), Float64[Q for _ in 1:n+1])
    stdRes2 = StandardResource{2}(zeros(Float64, n+1, n+1), zeros(Float64, n+1), fill(Inf, n+1))
    res     = Resources(customRes, stdRes1, stdRes2)

    parameters = Parameters(restarts = restarts, outerIterMax = outerIterMax, innerIterMax = innerIterMax)
    diversif = Diversification(outerShift = 3, outerSwap = 0, innerShift = 1, innerSwap = 0)
    pm = TargetRatePenaltyManager(
        penaltyCustom = 100.0,
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
        updatePeriod = 30
    )
    solver = Solver(
        seed = seed,
        parameters = parameters,
        diversification = diversif,
        penaltyManager = pm,
        acceptCriteria = Metropolis(100.0, 0.997),
        stopCriteria = ByTemperature(0.1),
        res = res,
        data = dataHeuristic,
        neighborhoods = NEIGHBORHOODS,
        MIPSolver = CPLEX.Optimizer
    )

    t        = @elapsed NILS(solver)
    sol      = getBestSol(solver)
    poolSize = length(solver.route_storage)

    # println("$(data.name) $(round(t, digits=2)) $(round(sol.cost, digits=2))")
    # printSol(sol)
    return sol.cost, t, poolSize
end

function collectInstances(dataDir::String)
    instances = String[]
    for folder in ("D", "MG", "SN")
        folderPath = joinpath(dataDir, folder)
        for file in readdir(folderPath)
            push!(instances, joinpath(folderPath, file))
        end
    end
    return instances
end

function runAll()
    restarts     = 1
    outerIterMax = 500
    innerIterMax = 5
    seeds        = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

    baseDir = @__DIR__
    dataDir = joinpath(baseDir, "..", "data")
    outCsv  = joinpath(baseDir, "..", "results.csv")

    instances = collectInstances(dataDir)

    open(outCsv, "w") do io
        println(io, "instance,seed,cost,time,pool")
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
                println(io, "$instanceName,$seed,$cost,$t,$pool")
                flush(io)
                println("$instanceName $seed $cost $t $pool")
            end
        end
    end
end

runAll()

