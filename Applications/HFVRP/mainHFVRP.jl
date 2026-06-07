include("../../src/Include.jl")
include("resources.jl")
include("data.jl")
ENV["JULIA_HASH_SEED"] = "0"

# using PlotlyJS
# using CPLEX

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
                demand += solver.res.customResource.d[sol.routes[r][i-1] + 1, sol.routes[r][i] + 1]
                print("$(sol.routes[r][i]) ($demand) -> ")
            end
        end
        println()
    end
    println("\nCost: $(sol.dist)")
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
                load += solver.res.customResource.d[solution.routes[r][i-1] + 1, solution.routes[r][i] + 1]
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
    feas1F = 0; feas2F = 0; feas1B = 0; feas2B = 0
    demand1F = 0; accDemand1F = Vector{Int}()
    demand2F = 0; accDemand2F = Vector{Int}()
    demand1B = 0; accDemand1B = Vector{Int}()
    demand2B = 0; accDemand2B = Vector{Int}()
    for i = 1:length(route1)-2
        demand1F += solver.res.customResource.d[route1[i]+1, route1[i+1]+1]
        if demand1F <= solver.res.customResource.cap[end]
            feas1F += 1
        end
        push!(accDemand1F, demand1F)
    end
    for i = 1:length(route2)-2
        demand2F += solver.res.customResource.d[route2[i]+1, route2[i+1]+1]
        if demand2F <= solver.res.customResource.cap[end]
            feas2F += 1
        end
        push!(accDemand2F, demand2F)
    end
    for i = length(route1):-1:3
        demand1B += solver.res.customResource.d[route1[i]+1, route1[i-1]+1]
        if demand1B <= solver.res.customResource.cap[end]
            feas1B += 1
        end
        push!(accDemand1B, demand1B)
    end
    for i = length(route2):-1:3
        demand2B += solver.res.customResource.d[route2[i]+1, route2[i-1]+1]
        if demand2B <= solver.res.customResource.cap[end]
            feas2B += 1
        end
        push!(accDemand2B, demand2B)
    end
    return length(route1) - max(feas1F, feas1B) - 2, length(route2) - max(feas2F, feas2B) - 2, accDemand1F, accDemand2F, accDemand1B, accDemand2B
end

function checkInfeasibles(solver::Solver, route1::Vector{Int})
    feas1F = 0; feas1B = 0
    demand1F = 0; accDemand1F = Vector{Int}()
    demand1B = 0; accDemand1B = Vector{Int}()
    for i = 1:length(route1)-2
        demand1F += solver.res.customResource.d[route1[i]+1, route1[i+1]+1]
        if demand1F <= solver.res.customResource.cap[end]
            feas1F += 1
        end
        push!(accDemand1F, demand1F)
    end
    for i = length(route1):-1:3
        demand1B += solver.res.customResource.d[route1[i]+1, route1[i-1]+1]
        if demand1B <= solver.res.customResource.cap[end]
            feas1B += 1
        end
        push!(accDemand1B, demand1B)
    end
    return length(route1) - max(feas1F, feas1B) - 2, accDemand1F, accDemand1B
end

function Create_Demand_Matrix(data::DataHVRP)
    n = nb_customers(data)
    d_ = zeros(Float64, n+1, n+1)
    for i = 1:n+1
        for j = 1:n+1
            if i != j && j != 1
                d_[i,j] = d(data, j-1)
            end
        end
    end
    return d_
end

function DistMatrix(data::DataHVRP)
    n = length(data.G′.V′)
    Cost = zeros(Float64, n, n)
    for i = 1:n
        for j = 1:n
            Cost[i,j] = distance(data, i-1, j-1)
        end
    end
    return Cost
end

function main(data::DataHVRP, restarts::Int, outerIterMax::Int, innerIterMax::Int, seed::Int)
    n  = nb_customers(data)
    nv = length(data.veh_types)
    fac   = [veh_factor(data, k) for k = 1:nv]
    fixed = [veh_fixed(data, k)  for k = 1:nv]
    Q     = [veh_capacity(data, k) for k = 1:nv]
    Qmax  = maximum(Q)

    customers = Vector{Vertex}()
    for i = 1:n
        push!(customers, Vertex(i, [(0, 0)]))
    end

    dataHeuristic = ProblemData(customers, zeros(Float64, n+1, n+1), 10)

    d_ = Create_Demand_Matrix(data)

    customRes = CustomResource(nv, Float64.(fixed), Float64.(fac), Float64.(Q), d_, DistMatrix(data))

    # Capacity (max vehicle) as standard resource 1
    stdRes1 = StandardResource{1}(d_, zeros(Float64, n+1), Float64[Qmax for _ in 1:n+1])
    # Loose second standard resource (no constraint)
    stdRes2 = StandardResource{2}(zeros(Float64, n+1, n+1), zeros(Float64, n+1), fill(Inf, n+1))

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
        data = dataHeuristic,
        neighborhoods = NEIGHBORHOODS
    )
    t = @elapsed NILS(solver)
    sol = getBestSol(solver)

    println("$(data.name) $(round(t, digits=2)) $(round(solver.outerBestSol.cost, digits=2))")
    printCVRP(solver, solver.outerBestSol)
    return sol.cost
end

brandao     = true
instance    = "path/to/instance"
restarts    = 1
outerIterMax = 500
innerIterMax = 5
seed        = 1

data = brandao ? readHVRPBrandaoData(instance) : readHVRPClassicData(instance)
main(data, restarts, outerIterMax, innerIterMax, seed)
