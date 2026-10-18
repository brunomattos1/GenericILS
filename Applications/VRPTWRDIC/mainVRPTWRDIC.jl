using GenericILS
using Random
include("resourcesVRPTWRDIC.jl")
ENV["JULIA_HASH_SEED"] = "0"
Random.seed!(0)

using CPLEX

# function fastbitset_str(bs::FastBitSet32)
#     elems = Int[]
#     for (blockIdx, block) in enumerate(bs.data)
#         if block == 0
#             continue
#         end
#         for bit in 0:31
#             if (block & (UInt32(1) << bit)) != 0
#                 push!(elems, (blockIdx - 1) * 32 + bit + 1)
#             end
#         end
#     end
#     return "{" * join(elems, ", ") * "}"
# end


"""Read a Solomon-format instance file and derive the data used by `main` (demands, time windows, distance and time matrices)."""
function readSolomon(path::String)
    lines = readlines(path)  # read all lines
    matrixLines = [split(strip(linha)) for linha in lines]  # split on whitespace

    # skip the header
    matrixLines = matrixLines[2:end]
    if length(matrixLines[end]) == 0
        matrixLines = matrixLines[1:end-1]
    end
    name = basename(path)[1:end-4]
    n = parse(Int, split(name, "_")[1])
    vehicles = n
    capacity = parse(Float64, matrixLines[1][end-1])
    Vcap = parse(Float64, matrixLines[1][end])
    customers = Vector{Vertex}()
    id = Int[]
    x = Float64[]
    y = Float64[]
    release = Float64[]
    dueDate = Float64[]
    readyTime = Float64[]
    serviceTime = Float64[]
    w = Float64[]
    vol = Float64[]
    inc = Vector{BitSet}()
    for i = 0:n
        row = matrixLines[i+1]
        push!(id, parse(Int, row[1]))
        push!(x, parse(Float64, row[3]))
        push!(y, parse(Float64, row[4]))
        push!(release, parse(Float64, row[5]))
        push!(readyTime, parse(Int, row[6]))
        push!(dueDate, parse(Int, row[7]))
        push!(serviceTime, parse(Int, row[8]))
        push!(w, parse(Int, row[9]))
        push!(vol, parse(Int, row[10]))

        inc_ = BitSet([])
        
        if i != 0 && length(row) > 10
            for k in row[11:end]  # all conflicting items (before Peso_Total and Volume_Total)
                if !isempty(k)
                    Base.union!(inc_, parse(Int, k))
                end
            end
        end
        push!(inc, inc_)
        push!(customers, Vertex(parse(Int, row[1]), Group(parse(Int, row[1]), 1, false)))   # alone in its group, which needs one visit: a classic VRP
    end
    n = length(customers)
    dist  = zeros(Float64, n, n)
    time  = zeros(Float64, n, n)
    ready = zeros(Int, n, n)
    due   = zeros(Int, n, n)
    dmat  = zeros(Float64, n, n)  # demand matrix

    for i in 1:n, j in 1:n
        if i != j
            # distance
            xi, yi = x[i], y[i]
            xj, yj = x[j], y[j]
            d = sqrt((xi-xj)^2 + (yi-yj)^2)
            dist[i,j] = d
            
            # time = service at i + travel to j
            time[i,j] = d + serviceTime[i]
            
            # time windows depend on the destination j
            ready[i,j] = readyTime[j]
            due[i,j]   = dueDate[j]
            
            # demand of the destination j (0 when returning to the depot)
            if id[j] == 0
                dmat[i,j] = 0
            else
                dmat[i,j] = w[j]
            end
        end
    end
    function makeSymmetric!(inc::Vector{BitSet})
        n = length(inc)
        for i in 1:n
            customerI = i - 1  # customer at position i
            for j in copy(inc[i])
                # j is a customer id: shift to a vector index
                push!(inc[j+1], customerI)
            end
        end
    end
    makeSymmetric!(inc)
    return x, y, w, vehicles, capacity, customers, dist, time, dmat, readyTime, dueDate, release, inc
end

"""Print the routes with load and arrival times, and the cost of `sol`."""
function printVRPTW(solver::Solver, sol::Solution)
    cont = 0
    dist = 0.0
    for r = 1:length(sol.routes)
        if length(sol.routes[r].visits) <= 2
            continue
        end
        cont += 1
        rt = sol.routes[r]
        time = rt.forwardLabels[end].state.RD
        demand = 0.
        print("#$cont: ")
        for i = 1:length(rt.visits)
            if i == 1
                print("0 (0.0) {$(round(time, digits = 2))}", " -> ")
            else
                dist += solver.data.costMatrix[rt.visits[i-1]+1, rt.visits[i]+1]
                time += round(solver.res.customResource.t[rt.visits[i-1]+1, rt.visits[i]+1], digits = 1)
                if time < solver.res.stdResource1.lb[rt.visits[i] + 1]
                    time = solver.res.stdResource1.lb[rt.visits[i] + 1]
                end
                demand += solver.res.customResource.q[rt.visits[i-1] + 1, rt.visits[i] + 1]
                print("$(rt.visits[i]) ($demand) {$(round(time, digits = 1))} [$(solver.res.stdResource1.lb[rt.visits[i] + 1]), $(solver.res.stdResource1.ub[rt.visits[i] + 1])]")
                if i < length(rt.visits) print(" -> ") end
            end
        end
        println()
    end
    println("\nDist: $(sol.dist). Cost: $(sol.cost)")
    # println("Violation: $(sol.resViolation)")
end

"""Independent check of `sol` against time windows and capacity (does not use the labels)."""
function checkVRPTW(solver::Solver, sol::Solution)
    feasible = true
    for r = 1:length(sol.routes)
        rt = sol.routes[r]
        time = rt.forwardLabels[end].state.RD
        demand = 0
        for i = 1:length(rt.visits)-1
            time += round(solver.res.customResource.t[rt.visits[i]+1, rt.visits[i+1]+1], digits = 1)
            demand += solver.res.customResource.q[rt.visits[i]+1, rt.visits[i+1]+1]
            if time < solver.res.stdResource1.lb[rt.visits[i+1] + 1]
                time = solver.res.stdResource1.lb[rt.visits[i+1] + 1]
            end
            if time > solver.res.stdResource1.ub[rt.visits[i+1] + 1] + 1e-6
                feasible = false
                throw("violou janela do cliente $(rt.visits[i+1]) na rota $r")
            end
            if demand > solver.res.customResource.Q + 1e-6
                feasible = false
                throw("rota $r viola capacidade do veiculo")
            end
        end
    end
    return feasible
end


"""Solve one instance with NILS (build resources, solver and algorithm, run, report the result)."""
function main(instance::String, outerIterMax::Int, innerIterMax::Int, seed::Int)
    instance = joinpath(@__DIR__, splitpath(instance)...)
    x, y, demands, vehicles, capacity, customers, dist, time, dmat, ready, due, release, inc = readSolomon(instance)
    deleteat!(customers, 1)

    maxNbRoute = vehicles
    data = ProblemData(customers, dist, maxNbRoute)

    n = length(inc)
    fastinc = Vector{FastBitSet32}(undef, n)
    for i in 1:n
        fb = FastBitSet32(100)
        for x in inc[i]
            add!(fb, x)
        end
        fastinc[i] = fb
    end
    customRes = CustomResource(time, due, release, dmat, capacity, fastinc)

    # Time windows as standard resource 1
    stdRes1 = StandardResource{1}(time, Float64.(ready), Float64.(due))
    # Capacity as standard resource 2
    stdRes2 = StandardResource{2}(Float64.(dmat), zeros(Float64, n), Float64[capacity for _ in 1:n])

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

    println("Solving instance $instance...")
    @time solve!(solver)
    sol = getBestSol(solver)
    printVRPTW(solver, solver.algorithm.outerBestSol)
    checkVRPTW(solver, solver.algorithm.outerBestSol)
    return sol.cost
end

outerIterMax = 1000
innerIterMax = 20
seed         = 1
instance = "TWRD/incomp/25/Incompatibility0.1/25_con_rate_01_C101.txt"

main(instance, outerIterMax, innerIterMax, seed)
