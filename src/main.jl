include("include.jl")
using CVRPLIB, Hygese, PlotlyJS

function plot_cvrp_solution(cvrp, solution::Solution; filename::String="solution.pdf")
    # Coordenadas
    routes = solution.routes
    coords = [(cvrp.coordinates[i, 1], cvrp.coordinates[i, 2]) for i = 2:cvrp.dimension]
    depot_coord = (cvrp.coordinates[1, 1], cvrp.coordinates[1, 2])

    # Iniciar o plot
    plt = plot(; title="Cost: $(solution.cost)", aspect_ratio=:equal, legend = false)

    # Rotas
    for (r, route) in enumerate(routes)
        route_coords = vcat([depot_coord], [coords[route[i]] for i = 2:length(route)-1], [depot_coord])
        xs, ys = first.(route_coords), last.(route_coords)
        plot!(plt, xs, ys, lw=2, marker=:circle, label="")
        for i in 2:length(route)-1
            cliente = route[i]
            cliente_coord = coords[cliente]
            demanda = cvrp.demand[cliente+1]
            label = string(cliente+1, " (", demanda, ")")
            annotate!(plt, cliente_coord[1], cliente_coord[2], text(label, :black, 5, :bottom))
        end
    end

    # Destacar o depósito
    scatter!(plt, [first(depot_coord)], [last(depot_coord)], marker=:square, color=:yellow, label="Depot")

    # Salvar
    savefig(plt, "C:\\Users\\bruno.mattos\\Downloads\\$(filename)")
    println("Gráfico salvo como '$filename'")
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

function printCVRP(solver::Solver, sol::Solution)
    for r = 1:length(sol.routes)
        demand = 0.
        print("#$r: ")
        for i = 1:length(sol.routes[r])
            if i == 1
                print("0 (0)", " -> ")
            elseif i == length(sol.routes[r])
                print("0 ($demand)")
            else
                demand += solver.res.d[sol.routes[r][i-1] + 1, sol.routes[r][i] + 1]
                print("$(sol.routes[r][i]) ($demand) -> ")
            end
        end
        println()
    end
    println("\nCost: $(sol.cost)")
    # println("Violation: $(sol.resViolation)")
end

function checkCVRP(solver::Solver, sol::Solution)
    visits = zeros(Int, length(solver.data.vertices))
    cost = 0.
    demands = solver.res.d[1,:][2:end-1]      
    resViol = Int[]
    for r = 1:length(sol.routes)
        load = 0.
        for i = 1:length(sol.routes[r])-1
            cost += solver.data.costMatrix[sol.routes[r][i]+1, sol.routes[r][i+1]+1]
            if sol.routes[r][i] > 0
                load += demands[sol.routes[r][i]]
            end
            # load -= solver.res.d[sol.routes[r][end-1]+1, sol.routes[r][end]+1]
            if sol.routes[r][i] > 0
                visits[sol.routes[r][i]] += 1
            end
        end
    end
    feasiblesF = zeros(Int, length(sol.routes))
    for r = 1:length(sol.routes)
        load = 0
        feasibles = 0
        for i = 1:length(sol.routes[r])-2
            load += solver.res.d[sol.routes[r][i]+1, sol.routes[r][i+1]+1]
            if load <= solver.res.Q
                feasibles += 1
            end
        end
        feasiblesF[r] = feasibles
        if feasiblesF[r] != sol.feasiblesF[r]
            @show sol.routes[r]
            @show sol.feasiblesF[r]
            @show feasiblesF[r]
            @show load, solver.res.Q
            throw("feasiblesF wrong")
        end
    end
    if abs(sol.cost - cost) > 1e-6
        throw("cost is: $(sol.cost) but should be $(cost)")
    end
    for i = 1:length(solver.data.vertices)
        if visits[i] > 1
            throw("customer $i visited more than once")
        end
        if visits[i] < 1
            throw("customer $i visited less than once")
        end
    end
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

function createArcDemands(demands)
    n = length(demands)
    d = zeros(Float64, n, n)

    for i in 1:n
        for j in 1:n
            if i != j
                if j == 1
                    d[i, j] = 0#demands[i]  # retorno ao depósito -> demanda do i
                else
                    d[i, j] = demands[j]  # demanda associada ao destino j
                end
            end
        end
    end
    return d
end

function initState()
    if DEBUG_MODE
        return CapacityState(0.0, 0.0, [0], 0)
    else
        return CapacityState(0.0, 0.0)
    end
end

function extendAlongArc(res::CapacityResource, state::CapacityState, a::Tuple{Int, Int}, buffer::CapacityState)
    if DEBUG_MODE
        # @show state
        buffer.q = state.q
        buffer.last = state.last
        empty!(buffer.path)
        append!(buffer.path, state.path)
        # @show buffer
        buffer.q += res.d[a...]
        append!(buffer.path, a[2] - 1)
        buffer.last = a[2] - 1
        if buffer.q > res.Q + 1e-5
            buffer.cost = Inf
            return buffer
        else
            buffer.cost = 0
            return buffer
        end
    else
        buffer.q = state.q
        buffer.q += res.d[a...]
        if buffer.q > res.Q + 1e-5
            buffer.cost = Inf
            return buffer
        else
            buffer.cost = 0
            return buffer
        end
    end
end

function extendAlongArc(res::CapacityResource, state::CapacityState, a::Tuple{Int, Int})
    if DEBUG_MODE
        state.q += res.d[a...]
        append!(state.path, a[2] - 1)
        state.last = a[2] - 1
        if state.q > res.Q + 1e-5
            state.cost = Inf
            return state
        else
            state.cost = 0
            return state
        end
    else
        state.q += res.d[a...]
        if state.q > res.Q + 1e-5
            state.cost = Inf
            return state
        else
            state.cost = 0
            return state
        end
    end
end

function concatenationCost(res::CapacityResource, v::Int, state1::CapacityState, state2::CapacityState)
    if DEBUG_MODE
        if state1.q + state2.q > res.Q + 1e-5
            newState = CapacityState(state1.q + state2.q, Inf, vcat(state1.path, reverse(state2.path)), state2.last)
            return newState
        else
            newState = CapacityState(state1.q + state2.q, 0.0, vcat(state1.path, reverse(state2.path)), state2.last)
            return newState
        end
    else
        if state1.q + state2.q > res.Q + 1e-5
            newState = CapacityState(state1.q + state2.q, Inf)
            return newState
        else
            newState = CapacityState(state1.q + state2.q, 0.0)
            return newState
        end
    end
end

function main(instance::String, restarts::Int, iter::Int, seed::Int)
    instName = instance[1:end-4]
    instance = joinpath(normpath(joinpath(@__DIR__, "..")), "PilsCvrp-main","PilsCvrp-main","data", string(instance[1]), instance)

    cvrp = CVRPLIB.readCVRP(instance)

    dist = Float64.(cvrp.weights)
    customers = Vector{Vertex}()
    nbCustomer = size(dist)[1]-1
    for i = 1:nbCustomer
        push!(customers, Vertex(i, [(0, 0)]))
    end

    maxNbRoute = ceil(Int, sum(cvrp.demand)/cvrp.capacity)
    data = ProblemData(customers, dist, maxNbRoute)

    demands = push!(Float64.(cvrp.demand), 0.0)
    d = createArcDemands(demands)
    res = CapacityResource(d, cvrp.capacity)

    solver = Solver(
        seed = seed,
        res = res,
        initState = initState,
        extendAlongArc = extendAlongArc, 
        concatenationCost = concatenationCost, 
        params = Parameters(restarts, iter, 10), 
        diversification = Diversification(2, 2),
        data = data, 
        neighborhoods = Set([1,3,5,7])
    )
    @time ILS(solver)
    printCVRP(solver, solver.currSol)
    plot_cvrp_interactive_html(cvrp, solver.currSol, filename = instName)
end

set = "M"
n = 151
k = 12
instance = "$set-n$n-k$k.vrp"
seed = 1
restarts = 50
iter = 100
main(instance, restarts, iter, seed)

# cvrp = CVRPLIB.readCVRP("C:\\Users\\bruno.mattos\\OneDrive - americanas s.a\\Documentos\\GitHub\\GenericILS\\PilsCvrp-main\\PilsCvrp-main\\data\\M\\M-n101-k10.vrp")
# result = solve_cvrp(cvrp)