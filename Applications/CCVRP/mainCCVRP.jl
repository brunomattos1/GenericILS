include("../../src/Include.jl")
include("resources.jl")
ENV["JULIA_HASH_SEED"] = "0"
Random.seed!(0)

using CVRPLIB
# using PlotlyJS
# using CPLEX

function plot_cvrp_interactive_html(cvrp, solution; filename::String="cvrp_solution.html")
    coords = [(cvrp.coordinates[i, 1], cvrp.coordinates[i, 2]) for i = 2:cvrp.dimension]
    depot_coord = (cvrp.coordinates[1, 1], cvrp.coordinates[1, 2])
    routes = solution.routes
    traces = GenericTrace{Dict{Symbol, Any}}[]

    push!(traces, PlotlyJS.scatter(
        x=[depot_coord[1]], y=[depot_coord[2]],
        mode="markers+text",
        marker=attr(color="yellow", size=12, symbol="square"),
        text=["1 (" * string(cvrp.demand[1]) * ")"],
        hoverinfo="text",
        name="Depot"
    ))

    for route in routes
        route_coords = vcat([depot_coord], [coords[route[i]] for i = 2:length(route)-1], [depot_coord])
        xs, ys = first.(route_coords), last.(route_coords)
        push!(traces, PlotlyJS.scatter(x=xs, y=ys, mode="lines", line=attr(width=2), showlegend=false))

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
                time += solver.res.stdResource1.d[sol.routes[r][i-1] + 1, sol.routes[r][i] + 1]
                print("0 ($demand)")
            else
                time += solver.res.stdResource1.d[sol.routes[r][i-1] + 1, sol.routes[r][i] + 1]
                demand += solver.res.customResource.t[sol.routes[r][i-1] + 1, sol.routes[r][i] + 1]
                print("$(sol.routes[r][i]) ($demand) -> ")
            end
        end
        println()
    end
    println("\nCost: $(sol.dist + sol.totalLabelCost) ")
end

function createArcDemands(demands)
    n = length(demands)
    d = zeros(Float64, n, n)
    for i in 1:n
        for j in 1:n
            if i != j
                if j == 1
                    d[i, j] = 0
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
    instance = joinpath(normpath(joinpath(@__DIR__, "data")), string(instance[1]), instance)

    cvrp = CVRPLIB.readCVRP(instance)
    dist = Float64.(cvrp.weights)
    customers = Vector{Vertex}()
    nbCustomer = size(dist)[1] - 1
    for i = 1:nbCustomer
        push!(customers, Vertex(i, [(0, 0)]))
    end
    maxNbRoute = ceil(Int, sum(cvrp.demand) / cvrp.capacity)
    data = ProblemData(customers, zeros(Float64, size(dist)[1], size(dist)[1]), maxNbRoute)

    demands = push!(Float64.(cvrp.demand), 0.0)
    d = createArcDemands(demands)

    customRes = CustomResource(demands, dist, 1000.0, 1000.0)

    # Capacity as standard resource 1
    n = length(customers) + 1
    stdRes1 = StandardResource{1}(d, zeros(Float64, n), Float64[cvrp.capacity for _ in 1:n])
    # Loose second standard resource (no constraint)
    stdRes2 = StandardResource{2}(zeros(Float64, n, n), zeros(Float64, n), fill(Inf, n))

    res = Resources(customRes, stdRes1, stdRes2)

    parameters = Parameters(restarts = restarts, outerIterMax = outerIterMax, innerIterMax = innerIterMax,
        penaltyCustom = 1.0, penaltyCustomIncrease = 0.01, penaltyCustomDecrease = 0.01,
        penaltyStandard1 = 1.0, penaltyStandard1Increase = 0.01, penaltyStandard1Decrease = 0.01,
        penaltyStandard2 = 0.0, penaltyStandard2Increase = 0.0, penaltyStandard2Decrease = 0.0
    )

    diversif = Diversification(outerShift = 2, outerSwap = 0, innerShift = 2, innerSwap = 0)

    solver = Solver(
        seed = seed,
        parameters = parameters,
        diversification = diversif,
        acceptCriteria = AcceptBest(),
        stopCriteria = ByIterMax(outerIterMax),
        res = res,
        data = data,
        neighborhoods = NEIGHBORHOODS
    )
    setTimeLimitILS(solver, 30.0)
    setTimeLimitSP(solver, 50.0)
    aggressivePool(solver, false)
    @time NILS(solver)
    sol = getBestSol(solver)
    printCVRP(solver, solver.outerBestSol)
    return sol.cost
end

set = "P"
n = 19
k = 2
instance = "$set-n$n-k$k.vrp"
seed = 1
restarts = 1
outerIterMax = 500
innerIterMax = 30

main(instance, restarts, outerIterMax, innerIterMax, seed)
