include("../../../src/Include.jl")
include("resources.jl")
include("data.jl")

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

function main(data::DataCCVRP, restarts::Int, outerIterMax::Int, innerIterMax::Int, seed::Int)
    dist = data.weights
    n    = size(dist, 1) - 1

    customers = Vector{Vertex}()
    for i = 1:n
        push!(customers, Vertex(i))
    end
    maxNbRoute    = ceil(Int, sum(data.demand) / data.capacity)
    dataHeuristic = ProblemData(customers, zeros(Float64, n+1, n+1), maxNbRoute)
    demands   = push!(copy(data.demand), 0.0)
    weights   = vcat(0.0, ones(Float64, n), 0.0)
    d         = buildArcDemands(demands)
    customRes = CustomResource(weights, dist, Inf, Inf)

    nv     = n + 1
    stdRes1 = StandardResource{1}(d, zeros(Float64, nv), Float64[data.capacity for _ in 1:nv])
    stdRes2 = StandardResource{2}(zeros(Float64, nv, nv), zeros(Float64, nv), fill(Inf, nv))
    res     = Resources(customRes, stdRes1, stdRes2)

    parameters = Parameters(restarts = restarts, outerIterMax = outerIterMax, innerIterMax = innerIterMax)
    diversif = Diversification(outerShift = 3, outerSwap = 0, innerShift = 1, innerSwap = 0)
    solver = Solver(
        seed = seed,
        parameters = parameters,
        penaltyManager = TargetRatePenaltyManager(),
        diversification = diversif,
        acceptCriteria = Metropolis(100.0, 0.997),
        stopCriteria = ByTemperature(0.1),
        # acceptCriteria = AcceptBest(),
        # stopCriteria = ByIterMax(500),
        res = res,
        data = dataHeuristic,
        neighborhoods = NEIGHBORHOODS,
        MIPSolver = CPLEX.Optimizer,
        timeLimitSP = 30.0
    )

    t   = @elapsed NILS(solver)
    sol = getBestSol(solver)

    # println("$(data.name) $(round(t, digits=2)) $(round(sol.cost, digits=2))")
    # printSol(sol)
    # printLabels(solver, sol)
    return sol.cost, t
end

function collectInstances(dataDir::String)
    instances = String[]
    for folder in ("Golden", "CMT", "Li")
        folderPath = joinpath(dataDir, folder)
        for file in readdir(folderPath)
            endswith(file, ".vrp") && push!(instances, joinpath(folderPath, file))
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
        println(io, "instance,seed,cost,time")
        flush(io)

        for instancePath in instances
            for seed in seeds
                instanceName = splitext(basename(instancePath))[1]
                cost = 99999
                t    = 99999
                try
                    data = readData(instancePath)
                    cost, t = main(data, restarts, outerIterMax, innerIterMax, seed)
                catch e
                    println("ERROR on $instanceName (seed=$seed): $e")
                    cost = 99999
                    t    = 99999
                end
                println(io, "$instanceName,$seed,$cost,$t")
                flush(io)
                println("$instanceName $seed $cost $t")
            end
        end
    end
end

runAll()

