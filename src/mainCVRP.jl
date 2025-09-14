include("include.jl")
using CVRPLIB, PlotlyJS
using CPLEX

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
                print("0 (0) {$time} [$(solver.res.stdResource.lb[sol.routes[r][i]+1]), $(solver.res.stdResource.ub[sol.routes[r][i]+1])]", " -> ")
            elseif i == length(sol.routes[r])
                time += solver.res.stdResource.d[sol.routes[r][i-1] + 1, sol.routes[r][i] + 1]
                print("0 ($demand) {$time} [$(solver.res.stdResource.lb[sol.routes[r][i]+1]), $(solver.res.stdResource.ub[sol.routes[r][i]+1])]")
            else
                time += solver.res.stdResource.d[sol.routes[r][i-1] + 1, sol.routes[r][i] + 1]
                demand += solver.res.customResource.d[sol.routes[r][i-1] + 1, sol.routes[r][i] + 1]
                print("$(sol.routes[r][i]) ($demand) {$time} [$(solver.res.stdResource.lb[sol.routes[r][i]+1]), $(solver.res.stdResource.ub[sol.routes[r][i]+1])] -> ")
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
                    d[i, j] = 0#demands[i]  # retorno ao depósito -> demanda do i
                else
                    d[i, j] = demands[j]  # demanda associada ao destino j
                end
            end
        end
    end
    return d
end

function main(instance::String, restarts::Int, outerIterMax::Int, innerIterMax::Int, seed::Int)
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
    customRes = CustomResource(d, cvrp.capacity)
    stdRes = StandardResource(d, Float64[0.0 for i = 1:length(customers)+1], Float64[cvrp.capacity for i = 1:length(customers)+1])

    res = Resources(customRes, stdRes)
    
    @show cvrp.capacity
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
        neighborhoods = Set{Int}([1, 2, 3, 4])
    )
    # constructSol!(solver)
    # # solver.outerCurrSol.routes[2] = [0, 1, 2, 3, 0]
    # # solver.outerCurrSol.routes[3] = [0, 5, 6, 7, 0]
    # solver.outerCurrSol.routes[1] = [0, 4, 0]
    # sol = deepcopy(solver.outerCurrSol)
    # computeLabels(solver, sol)
    # printCVRP(solver, sol)
    # @show dist[1, 2]
    # @show dist[2, 7]
    # @show dist[7, 1]
    # for r = 1:length(sol.routes)
    #     println("Infeas r: $(sol.infeas[r]), warp r: $(sol.warps[r])")
    # end
    # println("Total infeas: $(sol.totalInfeas), total warp: $(sol.totalWarp)")
    # println(stdRes)
    # @show dist[1,3] + dist[3, 5]
    # @show computeStdViolInsertion1(solver, sol, 1, 2, 2)
    # printCVRP(solver, sol)
    # r = 2
    # customer = 5
    # pos = 4
    # @show feas1 = computeViolSwap11(solver, sol, r, pos, customer)
    # @show infeas1 = length(sol.routes[r]) - 1 - feas1 - 1
    # @show sol.dist, sol.cost
    # for r = 1:length(sol.routes)
    #     println("Infeas r: $(sol.infeas[r]), warp r: $(sol.warps[r])")
    # end
    # println("Total infeas: $(sol.totalInfeas), total warp: $(sol.totalWarp)")
    # r, pos = 2,2
    # println("-"^50)
    # @show feas = computeViolRemove1(solver, sol, r, pos)
    # @show infeas = length(sol.routes[r]) - feas - 1 - 1
    # @show feas = computeViolInsertion1(solver, sol, r, 7, pos)
    # @show infeas = length(sol.routes[r]) - feas - 1 + 1
    # @show feas = computeViolSwap11(solver, sol, r, pos, 7)
    # @show infeas = length(sol.routes[r]) - feas - 1
    # sol.routes[r][pos] = 7
    # @show sol.routes[r]
    # r1, r2, pos1, pos2 = 2, 3, 3, 2 
    # @show feas1, feas2 = computeViolTwoOptStar(solver, sol, r1, r2, pos1, pos2)
    # @show infeas = length(sol.routes[r1]) + length(sol.routes[r2]) - feas1 - feas2 - 4
    # println("-"^50)
    # return
    # classicILS(solver)
    @time NILS(solver)
    printCVRP(solver, solver.outerBestSol)
    sol = deepcopy(solver.outerBestSol)

    @show solver.outerBestSol
    @show solver.outerBestSol.dist, solver.outerBestSol.cost
    for r = 1:length(solver.outerBestSol.routes)
        println("Infeas r: $(solver.outerBestSol.infeas[r]), warp r: $(solver.outerBestSol.warps[r])")
    end
    println("Total infeas: $(solver.outerBestSol.totalInfeas), total warp: $(solver.outerBestSol.totalWarp)")
    isFeasible(solver, solver.outerBestSol)
    plot_cvrp_interactive_html(cvrp, solver.outerBestSol, filename = instName)
    return
end

# set = "B"
# n = 38
# k = 6
set = "A"
n = 37
k = 6
const U = 100.0
instance = "$set-n$n-k$k.vrp"
seed = 1
restarts = 1
outerIterMax = 500
innerIterMax = 10

main(instance, restarts, outerIterMax, innerIterMax, seed)
