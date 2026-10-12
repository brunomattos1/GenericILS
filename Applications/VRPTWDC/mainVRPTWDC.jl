using GenericILS
using Random
include("resourcesSylvain_pkg.jl")
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

        dist = zeros(Float64, n, n)
        time = zeros(Float64, n, n)
        dmat = zeros(Float64, n, n)

        for i in 1:n, j in 1:n
            if i != j
                xi, yi = x[i], y[i]
                xj, yj = x[j], y[j]
                d = floor(10*sqrt((xi-xj)^2 + (yi-yj)^2)) / 10
                dist[i,j] = d
                time[i,j] = d + service_time[i]
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
        rt = sol.routes[r]
        if length(rt.visits) <= 2
            continue
        end
        cont += 1
        time = rt.backwardLabels[end].state.ET
        demand = 0.
        print("#$cont: ")
        for i = 1:length(rt.visits)
            if i == 1
                print("0 (0.0) {$time}", " -> ")
            else
                dist += solver.data.costMatrix[rt.visits[i-1]+1, rt.visits[i]+1]
                time += round(solver.res.customResource.t[rt.visits[i-1]+1, rt.visits[i]+1], digits = 1)
                if time < solver.res.customResource.l[rt.visits[i] + 1]
                    time = solver.res.customResource.l[rt.visits[i] + 1]
                end
                demand += solver.res.customResource.d[rt.visits[i-1] + 1, rt.visits[i] + 1]
                print("$(rt.visits[i]) ($demand) {$(round(time, digits = 1))} [$(solver.res.customResource.l[rt.visits[i] + 1]), $(solver.res.customResource.u[rt.visits[i] + 1])]")
                if i < length(rt.visits) print(" -> ") end
            end
        end
        println()
    end
    println("\nDist: $(sol.dist). Cost: $(sol.cost)")
end

function checkVRPTW(solver::Solver, sol::Solution)
    for r = 1:length(sol.routes)
        rt = sol.routes[r]
        time = rt.backwardLabels[end].state.ET
        dur = 0.0
        demand = 0
        for i = 1:length(rt.visits)-1
            time += round(solver.res.customResource.t[rt.visits[i]+1, rt.visits[i+1]+1], digits = 1)
            demand += solver.res.customResource.d[rt.visits[i]+1, rt.visits[i+1]+1]
            if time < solver.res.customResource.l[rt.visits[i+1] + 1]
                time = solver.res.customResource.l[rt.visits[i+1] + 1]
            end
            dur += time
            if time > solver.res.customResource.u[rt.visits[i+1] + 1] + 1e-6
                println("violou janela do cliente $(rt.visits[i+1]) na rota $r")
                return false
            end
            if demand > solver.res.customResource.Q + 1e-6
                println("rota $r viola capacidade do veiculo")
                return false
            end
            if time - rt.backwardLabels[end].state.ET > solver.res.customResource.D
                println("rota $r viola duration")
                return false
            end
        end
    end
    return true
end

function main(instance::String, duration, outerIterMax::Int, innerIterMax::Int, seed::Int)
    instance = joinpath(@__DIR__, "Solomon", instance)
    @show instance
    x, y, demands, vehicles, capacity, customers, dist, time, dmat, ready, due = read_solomon(instance)
    deleteat!(customers, 1)

    maxNbRoute = length(customers)
    data = ProblemData(customers, dist, maxNbRoute)

    D = parse(Float64, duration)
    customRes = CustomResource(dmat, time, Float64.(ready), Float64.(due), D, capacity)

    # Time windows as standard resource 1
    stdRes1 = StandardResource{1}(time, Float64.(ready), Float64.(due))
    # Capacity as standard resource 2
    n = length(customers) + 1
    stdRes2 = StandardResource{2}(dmat, zeros(Float64, n), Float64[capacity for _ in 1:n])

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
        penaltyCustom = 1.0, penaltyCustomIncrease = 0.01, penaltyCustomDecrease = 0.01,
        penaltyStandard1 = 1.0, penaltyStandard1Increase = 0.01, penaltyStandard1Decrease = 0.01,
        penaltyStandard2 = 1.0, penaltyStandard2Increase = 0.01, penaltyStandard2Decrease = 0.01
    ))

    setMIPSolver!(solver, CPLEX.Optimizer)

    println("Solving...")
    @time solve!(solver)
    sol = getBestSol(solver)
    printVRPTW(solver, solver.algorithm.outerBestSol)
    checkVRPTW(solver, solver.algorithm.outerBestSol)
    return sol.cost
end

instance     = "C101.txt"
duration     = "360"
outerIterMax = 500
innerIterMax = 7
seed         = 1

main(instance, duration, outerIterMax, innerIterMax, seed)
