using GenericILS
using Random
include("resources_pkg.jl")
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

function main(data::DataRiskVRP, outerIterMax::Int, innerIterMax::Int, seed::Int)
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

    maxTime = 20.0

    algorithm = NILSAlgorithm(res;
        acceptCriteria = NILS.MetropolisTimed(100.0, 100.0, maxTime, 2.0),
        stopCriteria = NILS.ByTemperature(0.001),
        diversification = NILS.Diversification(outerShift = 3, outerSwap = 3, innerShift = 1, innerSwap = 1),
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
        penaltyManager = TargetRatePenaltyManager(penaltyCustom = 1000.0; updatePeriod = 100),
        MIPSolver = CPLEX.Optimizer
    )

    setSeed!(solver, Random.MersenneTwister(seed))

    solve!(solver)
    sol = getBestSol(solver)

    return solver.statistics, solver.algorithm.iter
end

data = readData(joinpath(@__DIR__, "..", "data", "V", "121_1.5.rctvrp"))
stats, totalIter = main(data, 1, 5, 1)
