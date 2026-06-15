include("resourcesTWRDIC.jl")
include("../../src/include.jl")
using CPLEX
Random.seed!(0)  # inicializa o GLOBAL_RNG (se precisar)
ENV["JULIA_HASH_SEED"] = "0"

# function fastbitset_str(bs::FastBitSet32)
#     elems = Int[]
#     for (block_idx, block) in enumerate(bs.data)
#         if block == 0
#             continue
#         end
#         for bit in 0:31
#             if (block & (UInt32(1) << bit)) != 0
#                 push!(elems, (block_idx - 1) * 32 + bit + 1)
#             end
#         end
#     end
#     return "{" * join(elems, ", ") * "}"
# end


function read_solomon(path::String)
    lines = readlines(path)  # lê todas as linhas
    matrix_lines = [split(strip(linha)) for linha in lines]  # separa por espaço/tab

    # ignora cabeçalho
    matrix_lines = matrix_lines[2:end]
    if length(matrix_lines[end]) == 0
        matrix_lines = matrix_lines[1:end-1]
    end
    name = basename(path)[1:end-4]
    n = parse(Int, split(name, "_")[1])
    vehicles = n
    capacity = parse(Float64, matrix_lines[1][end-1])
    Vcap = parse(Float64, matrix_lines[1][end])
    customers = Vector{Vertex}()
    id = Int[]
    x = Float64[]
    y = Float64[]
    release = Float64[]
    due_date = Float64[]
    ready_time = Float64[]
    service_time = Float64[]
    w = Float64[]
    vol = Float64[]
    inc = Vector{BitSet}()
    for i = 0:n
        row = matrix_lines[i+1]
        push!(id, parse(Int, row[1]))
        push!(x, parse(Float64, row[3]))
        push!(y, parse(Float64, row[4]))
        push!(release, parse(Float64, row[5]))
        push!(ready_time, parse(Int, row[6]))
        push!(due_date, parse(Int, row[7]))
        push!(service_time, parse(Int, row[8]))
        push!(w, parse(Int, row[9]))
        push!(vol, parse(Int, row[10]))

        inc_ = BitSet([])
        
        if i != 0 && length(row) > 10
            for k in row[11:end]  # todos os itens conflitantes (antes do Peso_Total e Volume_Total)
                if !isempty(k)
                    Base.union!(inc_, parse(Int, k))
                end
            end
        end
        push!(inc, inc_)
        push!(customers, Vertex(parse(Int, row[1]), [(0.0, 0.0)]))
    end
    n = length(customers)
    dist  = zeros(Float64, n, n)
    time  = zeros(Float64, n, n)
    ready = zeros(Int, n, n)
    due   = zeros(Int, n, n)
    dmat  = zeros(Float64, n, n)  # matriz de demandas

    for i in 1:n, j in 1:n
        if i != j
            # distância
            xi, yi = x[i], y[i]
            xj, yj = x[j], y[j]
            d = sqrt((xi-xj)^2 + (yi-yj)^2)
            dist[i,j] = d
            
            # tempo = serviço no i + viagem até j
            time[i,j] = d + service_time[i]
            
            # janelas de tempo em função do destino j
            ready[i,j] = ready_time[j]
            due[i,j]   = due_date[j]
            
            # demanda associada ao destino j (0 se retorno ao depósito)
            if id[j] == 0
                dmat[i,j] = 0
            else
                dmat[i,j] = w[j]
            end
        end
    end
    function make_symmetric!(inc::Vector{BitSet})
        n = length(inc)
        for i in 1:n
            cliente_i = i - 1  # cliente correspondente à posição i
            for j in copy(inc[i])
                # j é o cliente, precisa ajustar para índice do vetor
                push!(inc[j+1], cliente_i)
            end
        end
    end
    make_symmetric!(inc)
    @show ready_time
    @show due_date
    return x, y, w, vehicles, capacity, customers, dist, time, dmat, ready_time, due_date, release, inc
end

function printVRPTW(solver::Solver, sol::Solution)
    cont = 0
    dist = 0.0
    for r = 1:length(sol.routes)
        if length(sol.routes[r].visits) <= 2
            continue
        end
        cont += 1
        time = sol.forwardLabels[r][end].state.RD
        demand = 0.
        print("#$cont: ")
        for i = 1:length(sol.routes[r].visits)
            if i == 1
                print("0 (0.0) {$(round(time, digits = 2))}", " -> ")
            else
                dist += solver.data.costMatrix[sol.routes[r].visits[i-1]+1, sol.routes[r].visits[i]+1]
                time += round(solver.res.customResource.t[sol.routes[r].visits[i-1]+1, sol.routes[r].visits[i]+1], digits = 1)
                if time < solver.res.stdResource.lb[sol.routes[r].visits[i] + 1]
                    time = solver.res.stdResource.lb[sol.routes[r].visits[i] + 1]
                end
                demand += solver.res.customResource.q[sol.routes[r].visits[i-1] + 1, sol.routes[r].visits[i] + 1]
                print("$(sol.routes[r].visits[i]) ($demand) {$(round(time, digits = 1))} [$(solver.res.stdResource.lb[sol.routes[r].visits[i] + 1]), $(solver.res.stdResource.ub[sol.routes[r].visits[i] + 1])]")
                if i < length(sol.routes[r].visits) print(" -> ") end
            end
        end
        println()
    end
    println("\nDist: $(sol.dist). Cost: $(sol.cost)")
    # println("Violation: $(sol.resViolation)")
end

function checkVRPTW(solver::Solver, sol::Solution)
    feasible = true
    for r = 1:length(sol.routes)
        time = sol.forwardLabels[r][end].state.RD
        demand = 0
        for i = 1:length(sol.routes[r].visits)-1
            time += round(solver.res.customResource.t[sol.routes[r].visits[i]+1, sol.routes[r].visits[i+1]+1], digits = 1)
            demand += solver.res.customResource.q[sol.routes[r].visits[i]+1, sol.routes[r].visits[i+1]+1]
            if time < solver.res.stdResource.lb[sol.routes[r].visits[i+1] + 1]
                time = solver.res.stdResource.lb[sol.routes[r].visits[i+1] + 1]
            end
            if time > solver.res.stdResource.ub[sol.routes[r].visits[i+1] + 1] + 1e-6
                feasible = false
                throw("violou janela do cliente $(sol.routes[r].visits[i+1]) na rota $r")
            end
            if demand > solver.res.customResource.Q + 1e-6
                feasible = false
                throw("rota $r viola capacidade do veiculo")
            end
        end
    end
    # for r = 1:length(sol.routes)
    #     for i = 2:length(sol.routes[r].visits) - 1
    #         for j = 2:length(sol.routes[r].visits) - 1
    #             if i == j 
    #                 continue
    #             end
    #             if has(solver.res.customResource.inc[sol.routes[r].visits[j]+1], sol.routes[r].visits[i])
    #                 # @show sol.routes[r]
    #                 feasible = false
    #                 # throw("clientes $(sol.routes[r].visits[i]) e $(sol.routes[r].visits[j]) são incompativeis e estao na rota $r")
    #             end
    #         end
    #     end
    # end
    return feasible
end


function main(instance::String, restarts::Int, outerIterMax::Int, innerIterMax::Int, seed::Int)
    basepath = normpath(joinpath(@__DIR__, ".."), "VRPTWRDIC")
    instance = joinpath(basepath, splitpath(instance)...)
    x, y, demands, vehicles, capacity, customers, dist, time, dmat, ready, due, release, inc = read_solomon(instance)
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
    # inc = [BitSet() for _ = 1:26]
    customRes = CustomResource(time, due, release, dmat, capacity, fastinc)
    # stdRes = StandardResource(zeros(Float64, size(time)[1], size(time)[1]), ready, due)
    stdRes = StandardResource(time, ready, due)
    
    res = Resources(customRes, stdRes)
    parameters = Parameters(restarts = restarts, outerIterMax = outerIterMax, innerIterMax = innerIterMax, 
        penaltyCustom = 1.0, penaltyCustomIncrease = 0.01, penaltyCustomDecrease = 0.01, 
        penaltyStandard = 1.0, penaltyStandardIncrease = 0.01, penaltyStandardDecrease = 0.01
    )

    diversif = Diversification(outerShift = 2, outerSwap = 0, innerShift = 2, innerSwap = 0)

    solver = Solver(
        seed = seed,
        res = res,
        stdResource = stdRes,
        params = parameters,
        diversification = diversif,
        data = data, 
        neighborhoods = [1,2,3,4]
    )
    println("Solving instance $instance...")
    @time NILS(solver)
    printVRPTW(solver, solver.outerBestSol)
    checkVRPTW(solver, solver.outerBestSol)
    return solver.outerBestSol.cost
end

# seed = 1
# restarts = 1
# outerIterMax = 2000
# innerIterMax = 15

# instance     = ARGS[1]
restarts     = 1
outerIterMax = 1000
innerIterMax = 20
seed         = 1
instance = "TWRD/incomp/25/Incompatibility0.1/25_con_rate_01_C101.txt"

main(instance, restarts, outerIterMax, innerIterMax, seed)

