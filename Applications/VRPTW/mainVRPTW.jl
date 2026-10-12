using GenericILS
using Random
include("resourcesVRPTW_pkg.jl")
ENV["JULIA_HASH_SEED"] = "0"
Random.seed!(0)

using CPLEX

function read_solomon(filename::String)
    open(filename, "r") do io
        text = open(filename, "r") do io
            read(io, String)
        end
        tokens = split(strip(text))
        vehicles = parse(Int, tokens[5])
        capacity = parse(Int, tokens[6])
        id = Int[]
        customers = Vector{Vertex}()
        ready_time = Int[]
        due_date = Int[]
        demands = Int[]
        service_time = Int[]
        x = Int[]
        y = Int[]
        idIdx = 0
        i = 19
        while i <= length(tokens) - 6
            push!(id, idIdx)
            push!(customers, Vertex(idIdx))
            push!(x, parse(Int, tokens[i+1]))
            push!(y, parse(Int, tokens[i+2]))
            push!(demands, parse(Int, tokens[i+3]))
            push!(ready_time, parse(Int, tokens[i+4]))
            push!(due_date, parse(Int, tokens[i+5]))
            push!(service_time, parse(Int, tokens[i+6]))
            i += 7
            idIdx += 1
        end
        n = length(customers)

        dist  = zeros(Float64, n, n)
        time  = zeros(Float64, n, n)
        ready = zeros(Int, n, n)
        due   = zeros(Int, n, n)
        dmat  = zeros(Float64, n, n)

        for i in 1:n, j in 1:n
            if i != j
                xi, yi = x[i], y[i]
                xj, yj = x[j], y[j]
                d = floor(10*sqrt((xi-xj)^2 + (yi-yj)^2)) / 10
                dist[i,j] = d
                time[i,j] = d + service_time[i]
                ready[i,j] = ready_time[j]
                due[i,j]   = due_date[j]
                if id[j] == 0
                    dmat[i,j] = 0
                else
                    dmat[i,j] = demands[j]
                end
            end
        end
        return x, y, demands, vehicles, capacity, customers, dist, time, dmat, ready_time, due_date
    end
end

function printVRPTW(solver::Solver, sol::Solution)
    cont = 0
    dist = 0.0
    for r = 1:length(sol.routes)
        if length(sol.routes[r].visits) <= 2
            continue
        end
        cont += 1
        time = 0.0
        demand = 0.
        print("#$cont: ")
        for i = 1:length(sol.routes[r].visits)
            if i == 1
                print("0 ", " -> ")
            else
                dist += solver.data.costMatrix[sol.routes[r].visits[i-1]+1, sol.routes[r].visits[i]+1]
                time += round(solver.res.stdResource1.d[sol.routes[r].visits[i-1]+1, sol.routes[r].visits[i]+1], digits = 1)
                if time < solver.res.stdResource1.lb[sol.routes[r].visits[i] + 1]
                    time = solver.res.stdResource1.lb[sol.routes[r].visits[i] + 1]
                end
                demand += solver.res.customResource.d[sol.routes[r].visits[i-1] + 1, sol.routes[r].visits[i] + 1]
                print("$(sol.routes[r].visits[i]) ($demand) {$(round(time, digits = 1))} [$(solver.res.stdResource1.lb[sol.routes[r].visits[i] + 1]), $(solver.res.stdResource1.ub[sol.routes[r].visits[i] + 1])]")
                if i < length(sol.routes[r].visits) print(" -> ") end
            end
        end
        println()
    end
    println("\nDist: $(sol.dist). Cost: $(sol.cost)")
end

function checkVRPTW(solver::Solver, sol::Solution)
    for r = 1:length(sol.routes)
        time = 0.0
        demand = 0
        for i = 1:length(sol.routes[r].visits)-1
            time += round(solver.res.stdResource1.d[sol.routes[r].visits[i]+1, sol.routes[r].visits[i+1]+1], digits = 1)
            demand += solver.res.customResource.d[sol.routes[r].visits[i]+1, sol.routes[r].visits[i+1]+1]
            if time < solver.res.stdResource1.lb[sol.routes[r].visits[i+1] + 1]
                time = solver.res.stdResource1.lb[sol.routes[r].visits[i+1] + 1]
            end
            if time > solver.res.stdResource1.ub[sol.routes[r].visits[i+1] + 1] + 1e-6
                throw("violou janela do cliente $(sol.routes[r].visits[i+1]) na rota $r")
            end
            if demand > solver.res.customResource.Q + 1e-6
                throw("rota $r viola capacidade do veiculo")
            end
        end
    end
end

function main(instance::String, outerIterMax::Int, innerIterMax::Int, seed::Int)
    basepath = normpath(joinpath(@__DIR__), "")
    instance = joinpath(basepath, splitpath(instance)...)
    x, y, demands, vehicles, capacity, customers, dist, time, dmat, ready, due = read_solomon(instance)
    deleteat!(customers, 1)

    maxNbRoute = vehicles
    data = ProblemData(customers, dist, maxNbRoute)

    customRes = CustomResource(dmat, capacity)

    # Time as standard resource 1
    stdRes1 = StandardResource{1}(time, Float64.(ready), Float64.(due))
    # Capacity also as standard resource 2
    stdRes2 = StandardResource{2}(dmat, Float64[0.0 for i = 1:length(customers)+1], Float64[capacity for i = 1:length(customers)+1])

    res = Resources(customRes, stdRes1, stdRes2)

    algorithm = NILSAlgorithm(res;
        acceptCriteria = NILS.AcceptBest(),
        stopCriteria = NILS.ByIterMax(outerIterMax),
        diversification = NILS.Diversification(outerShift = 2, outerSwap = 0, innerShift = 2, innerSwap = 0),
        innerIterMax = innerIterMax
    )

    solver = Solver(
        res = res,
        data = data,
        neighborhoods = (
            TwoOptStar(),
            IntraShift(),
            InterShift{1}(),
            InterShift{2}(),
            InterSwap{1, 1}(),
            InterSwap{2, 1}(),
            InterSwap{2, 2}()
        ),
        algorithm = algorithm
    )

    setSeed!(solver, Random.MersenneTwister(seed))

    setPenaltyManager!(solver, StandardPenaltyManager(
        penaltyCustom = 100.0, penaltyCustomIncrease = 0.01, penaltyCustomDecrease = 0.01,
        penaltyStandard1 = 100.0, penaltyStandard1Increase = 0.01, penaltyStandard1Decrease = 0.01,
        penaltyStandard2 = 100.0, penaltyStandard2Increase = 0.01, penaltyStandard2Decrease = 0.01
    ))

    setMIPSolver!(solver, CPLEX.Optimizer)

    println("Solving...")
    @time solve!(solver)
    sol = getBestSol(solver)
    return sol.cost
end

instance     = "Solomon/R101.txt"
outerIterMax = 30
innerIterMax = 5
seed         = 1

main(instance, outerIterMax, innerIterMax, seed)
