include("include.jl")
using CVRPLIB, Hygese

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

# instance = raw"C:\Users\bruno.mattos\OneDrive - americanas s.a\Documentos\GitHub\GMHVRP\PilsCvrp-main\PilsCvrp-main\data\A\A-n8-k4.vrp"
# instance = joinpath(@__DIR__, "..", "PilsCvrp-main","PilsCvrp-main","data")
instance = "A-n37-k6.vrp"
instance = joinpath(normpath(joinpath(@__DIR__, "..")), "PilsCvrp-main","PilsCvrp-main","data", string(instance[1]), instance)

# instance = raw"C:\Users\bruno.mattos\OneDrive - americanas s.a\Documentos\GitHub\GMHVRP\PilsCvrp-main\PilsCvrp-main\data\P\P-n20-k2.vrp"
# instance = raw"C:\Users\bruno.mattos\OneDrive - americanas s.a\Documentos\GitHub\GMHVRP\PilsCvrp-main\PilsCvrp-main\data\A\A-n37-k5.vrp"

# instance = "/home/logis/Documentos/GitHub/GMHVRP/PilsCvrp-main/PilsCvrp-main/data/A/A-n37-k6.vrp"
cvrp = CVRPLIB.readCVRP(instance)
# cvrp.capacity = 20
dist = Float64.(cvrp.weights)#criar_matriz_distancia(cvrp.coordinates)
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

function initState()
    return CapacityState(0.0, 0.0, [0], 0)
    # return CapacityState(0.0, 0.0)
end

function extendAlongArc(res::CapacityResource, state::CapacityState, a::Tuple{Int, Int})
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
end

function concatenationCost(res::CapacityResource, v::Int, state1::CapacityState, state2::CapacityState)
    if state1.q + state2.q > res.Q + 1e-5
        newState = CapacityState(state1.q + state2.q, Inf, vcat(state1.path, reverse(state2.path)), state2.last)
        return newState
    else
        newState = CapacityState(state1.q + state2.q, 0.0, vcat(state1.path, reverse(state2.path)), state2.last)
        return newState
    end
    # if state1.q + state2.q > res.Q + 1e-5
    #     # newState = CapacityState(state1.q + state2.q, Inf, Int[], state2.last)
    #     return Inf
    # else
    #     # newState = CapacityState(state1.q + state2.q, 0.0, Int[], state2.last)
    #     return 0
    # end
end

solver = Solver(
    seed = 1,
    res = res,
    initState = initState,
    extendAlongArc = extendAlongArc, 
    concatenationCost = concatenationCost, 
    params = Parameters(30, 10, 10), 
    diversification = Diversification(2, 0),
    data = data, 
    neighborhoods = Set([3])
)
# @time constructSol!(solver)
# solver.currSol.routes[1] = [0, 13, 12, 22, 23, 28, 2, 19, 14, 0]
# solver.currSol.routes[2] = [0, 20, 33, 35, 1, 3, 5, 8, 6, 0]

# computeLabels(solver)
# @show length(solver.currSol.routes[1])
# @show length(solver.currSol.routes[2])

# @show computeViolRemove1(solver, 1, 2)
# @show computeViolInsertion1(solver, 2, 13, 5)
ILS(solver)
printCVRP(solver, solver.currSol)
# [0, 15, 19, 30, 16, 21, 25, 8, 27, 11, 9, 24, 23, 17, 3, 0]
# [0, 0]
# @show checkInfeasibles(solver, [0, 15, 19, 30, 16, 25, 8, 27, 11, 9, 24, 23, 17, 3, 0], [0, 21, 0])

ap = AlgorithmParameters(timeLimit=0.01, seed=3) # `timeLimit` in seconds, `seed` is the seed for random values.
cvrp = CVRPLIB.readCVRP(instance)
result = solve_cvrp(cvrp, ap; verbose=false) # verbose=false to turn off all outputs

# @show sum(cvrp.demand[i+1] for i in [0, 21, 16, 22, 13, 6, 7, 0])

# r1: [0, 20, 33, 35, 1, 3, 5, 8, 6, 0] r2: [0, 12, 22, 23, 28, 2, 9, 21, 19, 13, 34, 0]
# acum D r1 F: [5, 13, 79, 80, 103, 110, 130, 148], acum D r2 F: [2, 14, 23, 26, 49, 68, 87, 97, 123, 142]
# acum D r1 B: [18, 38, 45, 68, 69, 135, 143, 148], acum D r2 B: [19, 45, 55, 74, 93, 116, 119, 128, 140, 142]
# Infeas: 6, Computed Infeas: 7
10 -> 5
12 -> 8

22 - 4 - 5 - 8

