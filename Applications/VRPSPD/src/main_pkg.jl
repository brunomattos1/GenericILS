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

const LITERATURE_TIMES = loadLiteratureTimes(joinpath(@__DIR__, "..", "..", "..", "literature", "VRPSPD_literature.csv"))

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

function main(data::DataVRPSPD, outerIterMax::Int, innerIterMax::Int, seed::Int)
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
        updatePeriod = 100
    )
    maxTime = 20.0

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
        penaltyManager = pm,
        MIPSolver = CPLEX.Optimizer
    )

    setSeed!(solver, Random.MersenneTwister(seed))

    @elapsed solve!(solver)
    getBestSol(solver)

    return solver.statistics, solver.algorithm.iter
end

instance = joinpath(@__DIR__, "..", "data", "MG", "C1_4_1")
data = readData(instance)
main(data, 1, 5, 1)
