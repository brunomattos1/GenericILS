include("resources.jl")
include("../../src/include.jl")
include("data.jl")
import Unicode
using CPLEX

const MAX_VEHICLES = 6


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


function Create_Demand_Matrix(data::DataHVRP)
    n = nb_customers(data)
    d_ = zeros(Float64,n+1,n+1)
    for i=1:n+1
        for j=1:n+1
            if i != j && j != 1
                d_[i,j] = d(data,j-1)
            end
        end
    end
    return d_
end

function CostDepotMatrix(data::DataHVRP)
    n = data.n
    m = data.nb_depots
    D = data.depot_ids
    Cost = zeros(Float64,n,n)
    for i=1:n
        for j=1:n
            Cost[i,j] = c(data,(0,i+m-1)) + c(data,(j+m-1,0))
            for k ∈ D
                if c(data,(k,i+m-1)) + c(data,(j+m-1,k)) < Cost[i,j]
                    Cost[i,j] = c(data,(k,i+m-1)) + c(data,(j+m-1,k))
                end
            end
        end
    end
    return Cost
end

function DistMatrix(data::DataHVRP)
    n = length(data.G′.V′)
    Cost = zeros(Float64,n,n)
    for i=1:n
        for j=1:n
            Cost[i,j] = distance(data,i-1,j-1)
        end
    end
    return Cost
end


function main(data::DataHVRP,restarts::Int,outerIterMax::Int,innerIterMax::Int,seed::Int;
    penaltyCustom::Float64 = 1.0,
    penaltyCustomIncrease::Float64 = 0.01,
    penaltyCustomDecrease::Float64 = 0.01,
    penaltyStandard::Float64 = 1.0,
    penaltyStandardIncrease::Float64 = 0.01,
    penaltyStandardDecrease::Float64 = 0.01,
    outerShift::Int = 2,
    outerSwap::Int = 0,
    innerShift::Int = 2,
    innerSwap::Int = 0
    )

    println("-----------------------------")
    println("Main Parameters")
    println()

    println("ILS Parameters")
    println("restarts        : $(restarts)")
    println("outerIterMax    : $(outerIterMax)")
    println("innerIterMax    : $(innerIterMax)")
    println("seed            : $(seed)")
    println()

    println("LS Operators")
    println("outerShift      : $(outerShift)")
    println("outerSwap       : $(outerSwap)")
    println("innerShift      : $(innerShift)")
    println("innerSwap       : $(innerSwap)")
    println()

    println("Penalty Parameters")
    println("penaltyCustom            : $(penaltyCustom)")
    println("penaltyCustomIncrease    : $(penaltyCustomIncrease)")
    println("penaltyCustomDecrease    : $(penaltyCustomDecrease)")
    println("penaltyStandard          : $(penaltyStandard)")
    println("penaltyStandardIncrease  : $(penaltyStandardIncrease)")
    println("penaltyStandardDecrease  : $(penaltyStandardDecrease)")
    println("-----------------------------")

    n = nb_customers(data) #number of customers
    nv = length(data.veh_types) #number of vehicles
    fac = [veh_factor(data,k) for k=1:nv]
    fixed = [veh_fixed(data,k) for k=1:nv]
    Q = [veh_capacity(data,k) for k=1:nv]
    Qmax = maximum(Q)
    K = n #an upper bound for the total number of routes 

    @show n
    @show nv
    @show Q
    @show fac
    @show Qmax

    #customer is an important structure from the heuristic. We can edit only the value of n
    customers = Vector{Vertex}() #não mexer
    for i = 1:n
        push!(customers, Vertex(i, [(0, 0)])) #não mexer
    end
    
    # ProblemData receives three components: 1) a vector customers where index 1 represents the depot; 2) a cost matrix representing the cost between the arcs and the maximum number of routes  
    null_cost = zeros(Float64,n+1,n+1)
    dataHeuristic = ProblemData(customers, null_cost, 10) #cost is matrix is null because the cost is in the path
    
    # Standard Resource 
    d_ = Create_Demand_Matrix(data) #head demand of product 1
    stdRes = StandardResource(d_, Float64[0.0 for i = 1:length(customers)+1], Float64[Qmax for i = 1:length(customers)+1]) # 0.0 e n são os bounds nos vertices (grafo RCSPP)
    
    # Custom Resource
    #=
struct CustomResource
    nv::Int64 #number of vehicles
    fixed::Vector{Int64}
    factor::Vector{Int64}
    cap::Vector{Float64}
    demand::Matrix{Float64} # head demand
    dist::Matrix{Float64}
end
    =#
    
    customRes = CustomResource(nv,fixed,fac,Q,d_,DistMatrix(data)) #forncer e modificar de acordo
    
    # Defining all resources
    res = Resources(customRes, stdRes)
    
    parameters = Parameters(restarts = restarts, outerIterMax = outerIterMax, innerIterMax = innerIterMax, 
        penaltyCustom = penaltyCustom, penaltyCustomIncrease = penaltyCustomIncrease, penaltyCustomDecrease = penaltyCustomDecrease, 
        penaltyStandard = penaltyStandard, penaltyStandardIncrease = penaltyStandardIncrease, penaltyStandardDecrease = penaltyStandardDecrease
    )

    diversif = Diversification(outerShift = outerShift, outerSwap = outerShift, innerShift = outerShift, innerSwap = outerShift)
    solver = Solver(
        seed = seed,
        res = res,
        stdResource = stdRes,
        params = parameters, 
        diversification = diversif,
        data = dataHeuristic, 
        neighborhoods = Int[1,2,3,4]
    )
    t = @elapsed NILS(solver) # Resolvedor propriamente dito
    sol = solver.outerBestSol

    println()
    println("######################")
    println("Columns: Name Time(s) Cost")
    println("satistics: $(data.name) $(round(t, digits=2)) $(round(sol.cost, digits=2))")
    println("######################")
    
    println()
    println("****** Heuristic Sol ********")
    @show sol.routes
    @show solver.outerCurrSol.cost
    @show solver.outerCurrSol.labelCosts

    println("Cost: $(round(sol.cost, digits=2))")
    println("**************")

    return
end


