include("include.jl")
using PlotlyJS
using CPLEX

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
                    dmat[i,j] = demands[j]
                end
            end
        end
        # for i = 1:n
        #     for j = 1:n
        #         println("vertex $i to vertex $j ready = $(ready[i,j]) due = $(due[i,j])")
        #     end
        # end
        return vehicles, capacity, customers, dist, time, dmat, ready, due
    end
end

function plot_cvrp_interactive_html(cvrp, solution; filename::String="cvrp_solution.html")
    coords = [(cvrp.coordinates[i, 1], cvrp.coordinates[i, 2]) for i = 2:cvrp.dimension]
    depot_coord = (cvrp.coordinates[1, 1], cvrp.coordinates[1, 2])
    routes = solution.routes
    traces = GenericTrace{Dict{Symbol, Any}}[]

    # Depósito
    push!(traces, PlotlyJS.scatter(
        x=[depot_coord[1]], y=[depot_coord[2]],
        mode="markers+text",
        marker=attr(color="yellow", size=12, symbol="square"),
        text=["1 (" * string(cvrp.demand[1]) * ")"],
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
            demanda = cvrp.demand[cliente+1]
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
    layout = Layout(title="Cost: $(solution.cost)", width=1200, height=800, plot_bgcolor = "white", dragmode = "pan")
    config = PlotConfig(
        displayModeBar=true,
        scrollZoom=true,
        modeBarButtonsToRemove=[
            "select2d", "lasso2d", "zoomIn2d", "zoomOut2d", "autoScale2d", "resetScale2d",
            "drawline", "drawopenpath", "drawclosedpath", "drawcircle", "drawrect", "eraseshape"
        ]
    )
    plt = PlotlyJS.Plot(traces, layout, config = config)
    open("C:\\Users\\bruno.mattos\\Downloads\\$(filename).html","w") do f
        PlotlyJS.PlotlyBase.to_html(f, plt; include_plotlyjs="cdn", full_html=true)
    end
    println("Gráfico interativo salvo como '$filename'")
end

function printVRPTW(solver::Solver, sol::Solution)
    
    for r = 1:length(sol.routes)
        time = 0.0
        demand = 0.
        print("#$r: ")
        for i = 1:length(sol.routes[r])
            if i == 1
                # time += round(solver.res.t[sol.routes[r][1]+1, sol.routes[r][2]+1],digits = 1)
                # @show time
                print("0 (0)", " -> ")
            else
                time += round(solver.res.t[sol.routes[r][i-1]+1, sol.routes[r][i]+1], digits = 1)
                if time < solver.res.early[sol.routes[r][i-1] + 1, sol.routes[r][i] + 1]
                    time = solver.res.early[sol.routes[r][i-1] + 1, sol.routes[r][i] + 1]
                end
                demand += solver.res.d[sol.routes[r][i-1] + 1, sol.routes[r][i] + 1]
                print("$(sol.routes[r][i]) ($demand) {$(round(time, digits = 1))} [$(solver.res.early[sol.routes[r][i-1] + 1, sol.routes[r][i] + 1]), $(solver.res.late[sol.routes[r][i-1] + 1, sol.routes[r][i] + 1])]")
                if i < length(sol.routes[r]) print(" -> ") end
            end
        end
        println()
    end
    println("\nCost: $(sol.cost)")
    # println("Violation: $(sol.resViolation)")
end

function checkInfeasibles(solver::Solver, route1::Vector{Int}, route2::Vector{Int})
    feas1F = 0
    feas2F = 0
    feas1B = 0
    feas2B = 0

    demand1F = 0
    accDemand1F = Vector{Int}()

    demand2F = 0
    accDemand2F = Vector{Int}()

    demand1B = 0
    accDemand1B = Vector{Int}()

    demand2B = 0
    accDemand2B = Vector{Int}()
    for i = 1:length(route1)-2
        demand1F += solver.res.d[route1[i]+1, route1[i+1]+1]
        if demand1F <= solver.res.Q
            feas1F += 1
        end
        push!(accDemand1F, demand1F)
    end
    for i = 1:length(route2)-2
        demand2F += solver.res.d[route2[i]+1, route2[i+1]+1]
        if demand2F <= solver.res.Q
            feas2F += 1
        end
        push!(accDemand2F, demand2F)
    end
    for i = length(route1):-1:3
        demand1B += solver.res.d[route1[i]+1, route1[i-1]+1]
        if demand1B <= solver.res.Q
            feas1B += 1
        end
        push!(accDemand1B, demand1B)
    end
    for i = length(route2):-1:3
        demand2B += solver.res.d[route2[i]+1, route2[i-1]+1]
        if demand2B <= solver.res.Q
            feas2B += 1
        end
        push!(accDemand2B, demand2B)
    end
    return length(route1) + length(route2) - max(feas1F, feas1B) - max(feas2F, feas2B) - 4, accDemand1F, accDemand2F, accDemand1B, accDemand2B
end

function checkInfeasibles(solver::Solver, route1::Vector{Int})
    feas1F = 0
    feas1B = 0

    demand1F = 0
    accDemand1F = Vector{Int}()
    demand1B = 0
    accDemand1B = Vector{Int}()

    for i = 1:length(route1)-2
        demand1F += solver.res.d[route1[i]+1, route1[i+1]+1]
        if demand1F <= solver.res.Q
            feas1F += 1
        end
        push!(accDemand1F, demand1F)
    end
    for i = length(route1):-1:3
        demand1B += solver.res.d[route1[i]+1, route1[i-1]+1]
        if demand1B <= solver.res.Q
            feas1B += 1
        end
        push!(accDemand1B, demand1B)
    end
    return length(route1) - max(feas1F, feas1B) - 2, accDemand1F,accDemand1B
end

function main(instance::String, restarts::Int, outerIterMax::Int, innerIterMax::Int, seed::Int)
    instName = split(instance, ".")[1]
    instance = joinpath(normpath(joinpath(@__DIR__, "..")), "Vrp-Set-Solomon", instance)
    vehicles, capacity, customers, dist, time, dmat, ready, due = read_solomon(instance)
    deleteat!(customers, 1)

    maxNbRoute = ceil(Int, sum(dmat[1, i] for i = 1:length(customers)+1) / capacity)
    
    data = ProblemData(customers, dist, maxNbRoute)
    time = zeros(length(customers)+1, length(customers)+1)
    ready = zeros(length(customers)+1, length(customers)+1)
    due = 100000 * ones(length(customers)+1, length(customers)+1)

    res = Resource(dmat, capacity, time, ready, due)

    solver = Solver(
        seed = seed,
        res = res,
        initState = initStateForward,
        extendAlongArc = extendAlongArc, 
        concatenationCost = concatenationCost, 
        params = Parameters(restarts, outerIterMax, innerIterMax, 10), 
        diversification = Diversification(2, 0, 2, 0),
        data = data, 
        neighborhoods = Set{Int}([1, 2, 3, 4])
    )

    println("Solving...")
    @time NILS(solver)
    # @time classicILS(solver)
    printVRPTW(solver, solver.outerBestSol)
    # plot_cvrp_interactive_html(cvrp, solver.outerBestSol, filename = instName)
end

seed = 1
restarts = 10
outerIterMax = 50
innerIterMax = 2

instance = "C101.txt"
# instance = "toy.txt"

main(instance, restarts, outerIterMax, innerIterMax, seed)