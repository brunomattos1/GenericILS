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
        return vehicles, capacity, customers, dist, time, dmat, ready_time, due_date
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
                time += round(solver.res.stdResource.d[sol.routes[r][i-1]+1, sol.routes[r][i]+1], digits = 1)
                if time < solver.res.stdResource.lb[sol.routes[r][i] + 1]
                    time = solver.res.stdResource.lb[sol.routes[r][i] + 1]
                end
                demand += solver.res.customResource.d[sol.routes[r][i-1] + 1, sol.routes[r][i] + 1]
                print("$(sol.routes[r][i]) ($demand) {$(round(time, digits = 1))} [$(solver.res.stdResource.lb[sol.routes[r][i] + 1]), $(solver.res.stdResource.ub[sol.routes[r][i] + 1])]")
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
    instance = joinpath(normpath(joinpath(@__DIR__, "..")), instance)
    vehicles, capacity, customers, dist, time, dmat, ready, due = read_solomon(instance)
    deleteat!(customers, 1)

    maxNbRoute = vehicles

    data = ProblemData(customers, dist, maxNbRoute)
    customRes = CustomResource(dmat, capacity)
    stdRes = StandardResource(time, ready, due)

    res = Resources(customRes, stdRes)
    
    @show capacity
    solver = Solver(
        seed = seed,
        res = res,
        stdResource = stdRes,
        initState = initStateForward,
        extendAlongArc = extendAlongArc, 
        concatenationCost = concatenationCost, 
        params = Parameters(restarts, outerIterMax, innerIterMax, 100, 100, 0.01, 0.01), 
        diversification = Diversification(2, 0, 2, 0),
        data = data, 
        neighborhoods = Set{Int}([2, 3, 4])
    )
    # constructSol!(solver)
    # sol = deepcopy(solver.outerCurrSol)
    # sol.routes = [[0, 2, 1, 0], [0, 5, 3, 0], [0, 4, 0]]
    # computeLabels(solver, sol)
    # r1 = 1
    # r2 = 2
    # i = 2
    # j = 2
    # @show computeStdViolTwoOptStar(solver, sol, r1, r2, i, j)
    # sol.routes = [[0, 2, 3, 0], [0, 5, 1, 0]]
    # printVRPTW(solver, sol)
    # # @show computeStdViolInsertion1(solver, sol, 1, customer, pos)
    # # @show computeStdViolRemove1(solver, sol, 1, 4)
    # # @show computeStdViolSwap11(solver, sol, r, pos, customer)
    # # @show concatenationCost(solver.res.stdResource, 2, sol.forwardLabels[1][2], sol.backwardLabels[2][2])

    # return
    println("Solving...")
    @time NILS(solver)
    # classicILS(solver)
    sol = deepcopy(solver.outerBestSol)
    printVRPTW(solver, solver.outerBestSol)
    # for r = 1:length(sol.routes)
    #     println("Infeas r: $(sol.infeas[r]), warp r: $(sol.warps[r])")
    # end
    # println("Total infeas: $(sol.totalInfeas), total warp: $(sol.totalWarp)")
    checkVRPTW(solver, sol)
    # computeLabels(solver, sol)
    # for r = 1:length(sol.routes)
    #     for i = 1:length(sol.forwardLabels[r])
    #         println(sol.forwardLabels[r][i]," ", solver.res.stdResource.lb[sol.routes[r][i]+1], " ",solver.res.stdResource.ub[sol.routes[r][i]+1])
    #     end
    # end
    # plot_cvrp_interactive_html(cvrp, solver.outerBestSol, filename = instName)
end

seed = 1
restarts = 1
outerIterMax = 400
innerIterMax = 2

instance = "Homberger/C2_4_3.txt"
# instance = "RC103.txt"
const U = 3693.0
# instance = "toy.txt"
# const U = 1236.0
main(instance, restarts, outerIterMax, innerIterMax, seed)