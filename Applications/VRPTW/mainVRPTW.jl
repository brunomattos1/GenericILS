include("resourcesVRPTW.jl")
include("../../src/Include.jl")
Random.seed!(0)

# using PlotlyJS
# using CPLEX

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
        time = 0.0
        demand = 0.
        print("#$cont: ")
        for i = 1:length(sol.routes[r])
            if i == 1
                print("0 ", " -> ")
            else
                dist += solver.data.costMatrix[sol.routes[r][i-1]+1, sol.routes[r][i]+1]
                time += round(solver.res.stdResource1.d[sol.routes[r][i-1]+1, sol.routes[r][i]+1], digits = 1)
                if time < solver.res.stdResource1.lb[sol.routes[r][i] + 1]
                    time = solver.res.stdResource1.lb[sol.routes[r][i] + 1]
                end
                demand += solver.res.customResource.d[sol.routes[r][i-1] + 1, sol.routes[r][i] + 1]
                print("$(sol.routes[r][i]) ($demand) {$(round(time, digits = 1))} [$(solver.res.stdResource1.lb[sol.routes[r][i] + 1]), $(solver.res.stdResource1.ub[sol.routes[r][i] + 1])]")
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
        time = 0.0
        demand = 0
        for i = 1:length(sol.routes[r])-1
            time += round(solver.res.stdResource.d[sol.routes[r][i]+1, sol.routes[r][i+1]+1], digits = 1)
            demand += solver.res.customResource.d[sol.routes[r][i]+1, sol.routes[r][i+1]+1]
            if time < solver.res.stdResource.lb[sol.routes[r][i+1] + 1]
                time = solver.res.stdResource.lb[sol.routes[r][i+1] + 1]
            end
            if time > solver.res.stdResource.ub[sol.routes[r][i+1] + 1] + 1e-6
                throw("violou janela do cliente $(sol.routes[r][i+1]) na rota $r")
            end
            if demand > solver.res.customResource.Q + 1e-6
                throw("rota $r viola capacidade do veiculo")
            end
        end
    end
end

function main(instance::String, restarts::Int, outerIterMax::Int, innerIterMax::Int, seed::Int)
    instName = split(instance, ".")[1]
    instName2 = split(split(instance, "/")[2], ".")[1]
    basepath = normpath(joinpath(@__DIR__, ".."), "VRPTW")
    instance = joinpath(basepath, splitpath(instance)...)
    x, y, demands, vehicles, capacity, customers, dist, time, dmat, ready, due = read_solomon(instance)
    deleteat!(customers, 1)

    maxNbRoute = vehicles
    data = ProblemData(customers, dist, maxNbRoute)
    # Capacity as custom resource
    # capacity = 50
    customRes = CustomResource(dmat, capacity)
    # customRes = CustomResource(zeros(Float64, length(customers)+1, length(customers)+1), capacity)

    # Time as standard resource
    stdRes1 = StandardResource{1}(time, Float64.(ready), Float64.(due))
    # Capacity also as standard resource
    stdRes2 = StandardResource{2}(dmat, Float64[0.0 for i = 1:length(customers)+1], Float64[capacity for i = 1:length(customers)+1])
    
    res = Resources(customRes, stdRes1, stdRes2)

    parameters = Parameters(restarts = restarts, outerIterMax = outerIterMax, innerIterMax = innerIterMax, 
        penaltyCustom = 100.0, penaltyCustomIncrease = 0.01, penaltyCustomDecrease = 0.01, 
        penaltyStandard1 = 100.0, penaltyStandard1Increase = 0.01, penaltyStandard1Decrease = 0.01,
        penaltyStandard2 = 100.0, penaltyStandard2Increase = 0.01, penaltyStandard2Decrease = 0.01
    )

    diversif = Diversification(outerShift = 2, outerSwap = 0, innerShift = 2, innerSwap = 0)

    solver = Solver(
        seed = seed,
        parameters = parameters,
        diversification = diversif,
        acceptCriteria = MetropolisTimed(100.0, 100.0, 15.0, 2.0),
        stopCriteria = ByTemperature(0.1),
        # acceptCriteria = AcceptBest(),
        # stopCriteria = ByIterMax(50),
        res = res,
        data = data,
        neighborhoods = NEIGHBORHOODS
    )
    println("Solving...")
    @time NILS(solver)
    sol = getBestSol(solver)
    return sol.cost
end

instance     = "Solomon/R101.txt"
restarts     = 1
outerIterMax = 50
innerIterMax = 5
seed         = 1

main(instance, restarts, outerIterMax, innerIterMax, seed)

