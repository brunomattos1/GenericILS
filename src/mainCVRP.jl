include("include.jl")
using CVRPLIB, PlotlyJS
using CPLEX
Random.seed!(0)  # inicializa o GLOBAL_RNG (se precisar)
ENV["JULIA_HASH_SEED"] = "0"
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
    open(joinpath(pwd(), "..", "plots", "$filename.html"),"w") do f
        PlotlyJS.PlotlyBase.to_html(f, plt; include_plotlyjs="cdn", full_html=true)
    end
    println("Gráfico interativo salvo como '$filename'")
end

function printCVRP(solver::Solver, sol::Solution)
    for r = 1:length(sol.routes)
        demand = 0.
        time = 0.
        print("#$r: ")
        for i = 1:length(sol.routes[r])
            if i == 1
                print("0 (0) -> ")
            elseif i == length(sol.routes[r])
                time += solver.res.stdResource.d[sol.routes[r][i-1] + 1, sol.routes[r][i] + 1]
                print("0 ($demand)")
            else
                time += solver.res.stdResource.d[sol.routes[r][i-1] + 1, sol.routes[r][i] + 1]
                demand += solver.res.customResource.d[sol.routes[r][i-1] + 1, sol.routes[r][i] + 1]
                print("$(sol.routes[r][i]) ($demand) -> ")
            end
        end
        println()
    end
    println("\nCost: $(sol.dist)")
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

function isFeasible(solver::Solver, solution::Solution)
    visits = zeros(Int, length(solver.data.vertices))
    cost = 0.
    demands = solver.res.customResource.d[1,:][2:end-1]      
    for r = 1:length(solution.routes)
        load = 0.
        for i = 1:length(solution.routes[r])-1
            cost += solver.data.costMatrix[solution.routes[r][i]+1, solution.routes[r][i+1]+1]
            if solution.routes[r][i] > 0
                # load += demands[solution.routes[r][i]]
                load += solver.res.customResource.d[solution.routes[r][i-1] + 1, solution.routes[r][i] + 1]
            end
            if load > solver.res.customResource.Q + 1e-6
                throw("load of route $r is greater than Q ($(solver.res.customResource.Q))")
            end
            if solution.routes[r][i] > 0
                visits[solution.routes[r][i]] += 1
            end
        end
    end
    if abs(solution.dist - cost) > 1e-6
        throw("dist is: $(solution.cost) but should be $(cost)")
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
        demand1F += solver.res.customResource.d[route1[i]+1, route1[i+1]+1]
        if demand1F <= solver.res.customResource.Q
            feas1F += 1
        end
        push!(accDemand1F, demand1F)
    end
    for i = 1:length(route2)-2
        demand2F += solver.res.customResource.d[route2[i]+1, route2[i+1]+1]
        if demand2F <= solver.res.customResource.Q
            feas2F += 1
        end
        push!(accDemand2F, demand2F)
    end
    for i = length(route1):-1:3
        demand1B += solver.res.customResource.d[route1[i]+1, route1[i-1]+1]
        if demand1B <= solver.res.customResource.Q
            feas1B += 1
        end
        push!(accDemand1B, demand1B)
    end
    for i = length(route2):-1:3
        demand2B += solver.res.customResource.d[route2[i]+1, route2[i-1]+1]
        if demand2B <= solver.res.customResource.Q
            feas2B += 1
        end
        push!(accDemand2B, demand2B)
    end
    return length(route1) - max(feas1F, feas1B) - 2, length(route2) - max(feas2F, feas2B) - 2, accDemand1F, accDemand2F, accDemand1B, accDemand2B
end

function checkInfeasibles(solver::Solver, route1::Vector{Int})
    feas1F = 0
    feas1B = 0

    demand1F = 0
    accDemand1F = Vector{Int}()
    demand1B = 0
    accDemand1B = Vector{Int}()

    for i = 1:length(route1)-2
        demand1F += solver.res.customResource.d[route1[i]+1, route1[i+1]+1]
        if demand1F <= solver.res.customResource.Q
            feas1F += 1
        end
        push!(accDemand1F, demand1F)
    end
    for i = length(route1):-1:3
        demand1B += solver.res.customResource.d[route1[i]+1, route1[i-1]+1]
        if demand1B <= solver.res.customResource.Q
            feas1B += 1
        end
        push!(accDemand1B, demand1B)
    end
    return length(route1) - max(feas1F, feas1B) - 2, accDemand1F, accDemand1B
end

function createArcDemands(demands)
    n = length(demands)
    d = zeros(Float64, n, n)

    for i in 1:n
        for j in 1:n
            if i != j
                if j == 1
                    d[i, j] = 0#demands[i]
                else
                    d[i, j] = demands[j]
                end
            end
        end
    end
    return d
end

function distanceMatrix(cvrp)
    dist = zeros(Float64, cvrp.dimension, cvrp.dimension)
    @show cvrp.coordinates[1, :]
    for i = 1:size(cvrp.coordinates)[1]
        for j = 1:size(cvrp.coordinates)[1]
            if i != j
                dist[i,j] = floor(10*sqrt((cvrp.coordinates[i,:][1] - cvrp.coordinates[j,:][1])^2 + (cvrp.coordinates[i,:][2] - cvrp.coordinates[j,:][2])^2))/10
            end
        end
    end
    return dist
end

function main(instance::String, restarts::Int, outerIterMax::Int, innerIterMax::Int, seed::Int)
    instName = instance[1:end-4]
    instance = joinpath(normpath(joinpath(@__DIR__, "..")), "PilsCvrp-main","PilsCvrp-main","data", string(instance[1]), instance)
    # instance = joinpath(normpath(joinpath(@__DIR__, "..")), "PilsCvrp-main","PilsCvrp-main","data", "Golden", "Golden_1.vrp")

    cvrp = CVRPLIB.readCVRP(instance)
    dist = Float64.(cvrp.weights)
    customers = Vector{Vertex}()
    nbCustomer = size(dist)[1]-1
    for i = 1:nbCustomer
        push!(customers, Vertex(i, [(0, 0)]))
    end
    maxNbRoute = ceil(Int, sum(cvrp.demand)/cvrp.capacity) + 3
    data = ProblemData(customers, dist, maxNbRoute)

    demands = push!(Float64.(cvrp.demand), 0.0)
    d = createArcDemands(demands)

    # cvrp.capacity = 20
    customRes = CustomResource(d, cvrp.capacity)
    stdRes = StandardResource(d, Float64[0.0 for i = 1:length(customers)+1], Float64[cvrp.capacity for i = 1:length(customers)+1])

    res = Resources(customRes, stdRes)
    
    @show cvrp.capacity
    parameters = Parameters(restarts = restarts, outerIterMax = outerIterMax, innerIterMax = innerIterMax, 
        penaltyCustom = 1.0, penaltyCustomIncrease = 0.01, penaltyCustomDecrease = 0.01, 
        penaltyStandard = 1.0, penaltyStandardIncrease = 0.01, penaltyStandardDecrease = 0.01
    )

    diversif = Diversification(outerShift = 1, outerSwap = 0, innerShift = 2, innerSwap = 0)
    solver = Solver(
        seed = seed,
        res = res,
        stdResource = stdRes,
        params = parameters, 
        diversification = diversif,
        data = data, 
        neighborhoods = Int[1, 2, 3, 4]
    )
    # constructSol!(solver)
    # sol = deepcopy(solver.outerCurrSol)
    # sol.routes = [[0,1,3,0], [0,2,5,0], [0,4,0]]
    # cost = 0.0
    # for r = 1:3
    #     cost += c(solver, sol.routes[r])
    # end
    # sol.cost, sol.dist = cost, cost
    # computeLabels(solver, sol)
    # @show interSwap11!(solver, sol)
    # for r1 = 1:length(sol.routes)
    #     @show sol.lastModif[r1]
    #     for r2 = 1:length(sol.routes)
    #         if r1 != r2
    #             @show sol.lastEval[(:interSwap, r1, r2)]
    #         end
    #     end
    # end
    # return
    @time NILS(solver)
    printCVRP(solver, solver.outerBestSol)
    isFeasible(solver, solver.outerBestSol)
    plot_cvrp_interactive_html(cvrp, solver.outerBestSol, filename = instName)
    return
end

set = "M"
n = 151
k = 12
instance = "$set-n$n-k$k.vrp"
seed = 2
restarts = 1
outerIterMax = 500
innerIterMax = 5

main(instance, restarts, outerIterMax, innerIterMax, seed)
