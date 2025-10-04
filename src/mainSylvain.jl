include("include.jl")
using PlotlyJS
using CPLEX
Random.seed!(0)  # inicializa o GLOBAL_RNG (se precisar)
ENV["JULIA_HASH_SEED"] = "0"
 
function read_solomon(filename::String)
    open(filename, "r") do io
        # Nome da instância
        text = open(filename, "r") do io
            read(io, String)
        end
        tokens = split(strip(text))     # vetor de strings limpas        @show tokens
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
            push!(customers, Vertex(idIdx, Vector{Tuple{Float64, Float64}}()))
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
        
        # Matrizes
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
                d = floor(10*sqrt((xi-xj)^2 + (yi-yj)^2)) / 10
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
                    dmat[i,j] = demands[j]
                end
            end
        end
        # for i = 1:n
        #     for j = 1:n
        #         println("vertex $i to vertex $j ready = $(ready[i,j]) due = $(due[i,j])")
        #     end
        # end
        return x, y, demands, vehicles, capacity, customers, dist, time, dmat, ready_time, due_date
    end
end

function plot_vrptw(x, y, demands, solution; filename)
    coords = [(x[i], y[i]) for i = 2:length(x)]
    depot_coord = (x[1], y[1])
    routes = solution.routes
    traces = GenericTrace{Dict{Symbol, Any}}[]

    # Depósito
    push!(traces, PlotlyJS.scatter(
        x=[depot_coord[1]], y=[depot_coord[2]],
        mode="markers+text",
        marker=attr(color="yellow", size=12, symbol="square"),
        text=["1 (" * string(demands[1]) * ")"],
        hoverinfo="text",
        name="Depot"
    ))

    # Rotas
    for route in routes
        route_coords = vcat([depot_coord], [coords[route[i]] for i = 2:length(route)-1], [depot_coord])
        xs, ys = first.(route_coords), last.(route_coords)
        push!(traces, PlotlyJS.scatter(x=xs, y=ys, mode="lines", line=attr(width=2), showlegend=false))

        # Pontos clientes com tooltip
        for i in 2:length(route)-1
            cliente = route[i]
            coord = coords[cliente]
            demanda = demands[cliente+1]
            texto = "Cliente $(cliente) (Demanda: $(demanda))"
            push!(traces, PlotlyJS.scatter(
                x=[coord[1]], y=[coord[2]],
                mode="markers",
                marker=attr(size=8, color = "black"),
                text=[texto],
                hoverinfo="text",
                showlegend=false
            ))
        end
    end
    layout = Layout(title="Cost: $(solution.cost)", width=1200, height=800, plot_bgcolor = "white")
    config = PlotConfig(
        displayModeBar=true,
        scrollZoom=true,
        modeBarButtonsToRemove=[
            "select2d", "lasso2d", "zoomIn2d", "zoomOut2d",
            "drawline", "drawopenpath", "drawclosedpath", "drawcircle", "drawrect", "eraseshape"
        ]
    )
    plt = PlotlyJS.Plot(traces, layout, config = config)
    open(joinpath(pwd(), "..", "plots", "$filename.html"),"w") do f
        PlotlyJS.PlotlyBase.to_html(f, plt; include_plotlyjs="cdn", full_html=true)
    end
    println("Gráfico interativo salvo como '$filename'")
end

function printVRPTW(solver::Solver, sol::Solution)
    cont = 0
    dist = 0.0
    for r = 1:length(sol.routes)
        if length(sol.routes[r]) <= 2
            continue
        end
        cont += 1
        time = sol.backwardLabels[r][end].custom_res.ET
        demand = 0.
        print("#$cont: ")
        for i = 1:length(sol.routes[r])
            if i == 1
                print("0 (0.0) {$time}", " -> ")
            else
                dist += solver.data.costMatrix[sol.routes[r][i-1]+1, sol.routes[r][i]+1]
                time += round(solver.res.customResource.t[sol.routes[r][i-1]+1, sol.routes[r][i]+1], digits = 1)
                if time < solver.res.customResource.l[sol.routes[r][i] + 1]
                    time = solver.res.customResource.l[sol.routes[r][i] + 1]
                end
                demand += solver.res.customResource.d[sol.routes[r][i-1] + 1, sol.routes[r][i] + 1]
                print("$(sol.routes[r][i]) ($demand) {$(round(time, digits = 1))} [$(solver.res.customResource.l[sol.routes[r][i] + 1]), $(solver.res.customResource.u[sol.routes[r][i] + 1])]")
                if i < length(sol.routes[r]) print(" -> ") end
            end
        end
        println()
    end
    println("\nDist: $(sol.dist). Cost: $(sol.cost)")
    # println("Violation: $(sol.resViolation)")
end

function checkVRPTW(solver::Solver, sol::Solution)
    for r = 1:length(sol.routes)
        time = sol.backwardLabels[r][end].custom_res.ET
        dur = 0.0
        demand = 0
        for i = 1:length(sol.routes[r])-1
            time += round(solver.res.customResource.t[sol.routes[r][i]+1, sol.routes[r][i+1]+1], digits = 1)
            demand += solver.res.customResource.d[sol.routes[r][i]+1, sol.routes[r][i+1]+1]
            if time < solver.res.customResource.l[sol.routes[r][i+1] + 1]
                time = solver.res.customResource.l[sol.routes[r][i+1] + 1]
            end
            dur += time
            if time > solver.res.customResource.u[sol.routes[r][i+1] + 1] + 1e-6
                println("violou janela do cliente $(sol.routes[r][i+1]) na rota $r")
                return false
            end
            if demand > solver.res.customResource.Q + 1e-6
                println("rota $r viola capacidade do veiculo")
                return false
            end
            if time - sol.backwardLabels[r][end].custom_res.ET > solver.res.customResource.D
                println("rota $r viola duration")
                return false
            end
        end
    end
    return true
end

function main(instance::String, duration, restarts::Int, outerIterMax::Int, innerIterMax::Int, seed::Int)
    instName = split(instance, ".")[1]
    instName2 = split(split(instance, "/")[2], ".")[1]
    instance = joinpath(normpath(joinpath(@__DIR__, "..")), instance)
    x, y, demands, vehicles, capacity, customers, dist, time, dmat, ready, due = read_solomon(instance)
    deleteat!(customers, 1)
    
    maxNbRoute = 50

    data = ProblemData(customers, dist, maxNbRoute)
    D = parse(Float64, duration)
    customRes = CustomResource(dmat, time, ready, due, D, capacity)
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
    println("Solving...")
    @time NILS(solver)
    # bestFeasSol = deepcopy(solver.outerBestSol)
    # computeLabels(solver, bestFeasSol)
    # for r = 1:length(bestFeasSol.routes)
    #     println("-"^150)
    #     @printf("%-50s | %-12s | %-12s | %-12s | %-12s | %-12s\n", "Fwd path", "Cap", "ET", "RD", "TB", "Cost")
    #     for i = 1:length(bestFeasSol.routes[r])
    #         path = bestFeasSol.routes[r][1:i]
    #         q = bestFeasSol.forwardLabels[r][i].custom_res.q
    #         ET = bestFeasSol.forwardLabels[r][i].custom_res.ET
    #         RD = bestFeasSol.forwardLabels[r][i].custom_res.RD
    #         TB = bestFeasSol.forwardLabels[r][i].custom_res.TB
    #         cost = bestFeasSol.forwardLabels[r][i].cost
    #         @printf("%-50s | %-12.2f | %-12.2f | %-12.2f | %-12.2f | %-12.2f\n", path, q, ET, RD, TB, cost)
    #     end
    #     println("-"^150)
    #     route = bestFeasSol.routes[r]
    #     n = length(route)
    #     @printf("%-50s | %-12s | %-12s | %-12s | %-12s | %-12s\n", "Bwd path", "Cap", "ET", "RD", "TB", "Cost")
    #     for i = 1:n
    #         start = n - i + 1
    #         path = route[start:n]           # SUFIXO que termina em `end`
    #         q = bestFeasSol.backwardLabels[r][i].custom_res.q
    #         ET = bestFeasSol.backwardLabels[r][i].custom_res.ET
    #         RD = bestFeasSol.backwardLabels[r][i].custom_res.RD
    #         TB = bestFeasSol.backwardLabels[r][i].custom_res.TB
    #         cost = bestFeasSol.backwardLabels[r][i].cost
    #         @printf("%-50s | %-12.2f | %-12.2f | %-12.2f | %-12.2f | %-12.2f\n", path, q, ET, RD, TB, cost)
    #     end
    #     println("-"^150)
    # end
    printVRPTW(solver, solver.outerBestSol)
    checkVRPTW(solver, solver.outerBestSol)
    # solver.outerBestSol.routes = [[0, 98, 96, 100, 99, 2, 0], [0, 63, 81, 77, 79, 80, 0], [0, 83, 82, 84, 85, 91, 0], [0, 95, 94, 92, 93, 97, 0], [0, 90, 87, 86, 88, 89, 0], [0, 54, 53, 56, 58, 60, 0], [0, 78, 76, 71, 70, 73, 0], [0, 67, 65, 66, 69, 0], [0, 13, 15, 16, 14, 12, 0], [0, 5, 3, 8, 11, 10, 23, 0], [0, 48, 51, 50, 52, 49, 47, 0], [0, 20, 43, 42, 44, 45, 46, 0], [0, 41, 40, 55, 57, 59, 0], [0, 62, 74, 72, 61, 64, 68, 0], [0, 7, 9, 6, 4, 1, 75, 0], [0, 37, 38, 39, 36, 34, 0], [0, 31, 35, 33, 32, 29, 0], [0, 17, 19, 18, 30, 28, 0], [0, 24, 25, 27, 26, 22, 21, 0]]
    # open("/mnt/c/Users/bruno.mattos/OneDrive - americanas s.a/Documentos/GitHub/GenericILS/out/DCVRPTW/$(instName[9:end]).out", "w") do f
    #     write(f, "$(instName[9:end]),$(solver.outerBestSol.cost)\n")
    # end
    return solver.outerBestSol.cost
end

instance     = "Solomon/C101.txt"#ARGS[1]
duration     = "360"#ARGS[2]
restarts     = 1
outerIterMax = 500
innerIterMax = 7
seed         = 1

# const U = 1236.0
main(instance, duration, restarts, outerIterMax, innerIterMax, seed)
