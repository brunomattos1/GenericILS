using GenericILS
using Random
using VrpSolver
using JuMP: @variable, @objective, @constraint, value
include("data.jl")

using CPLEX

# The VRPSolver model of the CCVRP (the one of VRPSolverDemos) is built here and the heuristic is derived from it with
# heuristicModelFromVRPSolver: the model is the only description of the problem. It needs no BaPCod, since the model
# is only read, never optimized. The custom resource is the C++ one of the model (MetaSolver format), the same files
# that the BCP uses.
const CPP_DIR = joinpath(@__DIR__, "meta_solver")

############################ the VRPSolver model ############################

# Types of the parameters of the custom resource; the layout is the one of the C++ structs (field order, not names)
struct CumulativeResArcParams
    t::Cdouble # time consumption
    w::Cdouble # weight
end

struct CumulativeResVertexParams end

struct CumulativeResConstParams
    Wmax::Cdouble # maximum accumulated weight
    Tmax::Cdouble # maximum accumulated time consumption
end

@register_custom_res_param_types(CumulativeResArcParams, CumulativeResVertexParams, CumulativeResConstParams)

"""
Build the VRPSolver model of `data`: the vertices are `0:n` (0 is the depot), one integer variable `x` per edge and the
cost variable `z` of the custom resource. A route is a path of the graph from 0 to 0 that carries at most the capacity;
the cost of a route is the sum, over its customers, of the demand times the time at which it is served. `maxRoutes`
bounds the number of routes and `ub` is a known upper bound of the cost (it only defines `Tmax` of the resource).
"""
function buildVrpModel(data::DataCCVRP, maxRoutes::Int; ub::Float64 = 1e7)
    n      = length(data.demand) - 1
    V      = 0:n                                                    # vertices of the graphs, the depot is 0
    V⁺     = 1:n                                                    # customers
    Q      = data.capacity
    dist(i, j) = data.weights[i+1, j+1]
    demand(i)  = data.demand[i+1]
    E      = [(i, j) for i in V for j in V if i < j]                # an edge for every pair of vertices
    δ(i)   = [(min(i, j), max(i, j)) for j in V if j != i]          # edges incident to i

    cvrp = VrpModel()
    @variable(cvrp.formulation, x[e in E], Int)
    @variable(cvrp.formulation, z >= 0)
    @objective(cvrp.formulation, Min, z)
    @constraint(cvrp.formulation, deg[i in V⁺], sum(x[e] for e in δ(i)) == 2.0)

    L = max(1, ceil(Int, sum(data.demand) / Q))
    U = min(maxRoutes, n)
    G = VrpGraph(cvrp, collect(V), 0, 0, (L, U))                    # source = sink = depot
    capRes = add_resource!(G, main = true)
    cumRes = add_resource!(G, custom = true, cost_var = z)
    Tmax   = ub / minimum(demand(i) for i in V⁺ if demand(i) > 0.0)
    set_const_custom_res_params!(G, cumRes, CumulativeResConstParams(Q, Tmax))

    for i in V
        set_resource_bounds!(G, i, capRes, 0.0, Q)                  # accumulated load in [0, Q]
    end

    # two arcs for each edge; the load of an arc is the mean of the demands of its ends (a route consumes the sum of the
    # demands of its customers), the time is the distance and the weight is the demand of the vertex that it enters
    for (i, j) in E
        for (a, b) in ((i, j), (j, i))
            arc = add_arc!(G, a, b)
            add_arc_var_mapping!(G, arc, x[(i, j)])
            set_arc_consumption!(G, arc, capRes, (demand(a) + demand(b)) / 2)
            set_arc_custom_res_params!(G, arc, cumRes, CumulativeResArcParams(dist(a, b), demand(b)))
        end
    end
    add_graph!(cvrp, G)

    set_vertex_packing_sets!(cvrp, [[(G, i)] for i in V⁺])          # every customer is visited once
    define_elementarity_sets_distance_matrix!(cvrp, G, [[dist(i, j) for j in V⁺] for i in V⁺])
    add_capacity_cut_separator!(cvrp, [([(G, i)], demand(i)) for i in V⁺], Q)
    set_branching_priority!(cvrp, "x", 1)

    function edge_ub_callback()                                     # x ≤ 1 for the edges that do not touch the depot
        for (i, j) in E
            if i != 0 && value(x[(i, j)]) > 1.001
                add_dynamic_constr!(cvrp.optimizer, [x[(i, j)]], [1.0], <=, 1.0, "edge_ub")
            end
        end
    end
    add_cut_callback!(cvrp, edge_ub_callback, "edge_ub")

    return cvrp
end

############################ solution ############################

"""Print the routes of `sol` with the ids of the model (`userIds[v + 1]` is the id of the vertex `v` of the heuristic)."""
function printSol(sol::Solution, userIds::Vector{Int})
    k = 0
    for r in sol.routes
        length(r.visits) > 2 || continue                            # an unused route
        k += 1
        println("route $k: ", join((userIds[v+1] for v in r.visits), " "))
    end
    println("Cost: $(round(sol.cost, digits=2))")
end

"""
Check `sol` against the model: the values of `x` that it defines give every customer degree 2, and the cost of the
routes by its definition (demand times the time of service) is the cost of the solution.
"""
function checkAgainstModel(data::DataCCVRP, sol::Solution, userIds::Vector{Int})
    n = length(data.demand) - 1
    x = Dict{Tuple{Int,Int},Int}()
    cost = 0.0
    for r in sol.routes
        length(r.visits) > 2 || continue
        time = 0.0
        for k in 2:length(r.visits)
            a, b = userIds[r.visits[k-1]+1], userIds[r.visits[k]+1]
            e = (min(a, b), max(a, b))
            x[e] = get(x, e, 0) + 1
            time += data.weights[a+1, b+1]
            cost += data.demand[b+1] * time
        end
    end
    degree(i) = sum(v for (e, v) in x if i in e)
    all(degree(i) == 2 for i in 1:n) || error("the solution does not give degree 2 to every customer")
    isapprox(cost, sol.cost; rtol = 1e-6) || error("cost $(sol.cost) of the solution is not $cost by its definition")
end

"""Solve one instance with NILS from its VRPSolver model; returns the cost and the time."""
function main(data::DataCCVRP, innerIterMax::Int, seed::Int)
    maxRoutes = ceil(Int, sum(data.demand) / data.capacity)
    model     = buildVrpModel(data, maxRoutes)
    h         = heuristicModelFromVRPSolver(model; cppDir = CPP_DIR)

    algorithm = NILSAlgorithm(h.res;
        acceptCriteria = NILS.Metropolis(100.0, 0.9),
        stopCriteria = NILS.ByTemperature(0.1),
        diversification = NILS.Diversification(outerShift = 3, outerSwap = 0, innerShift = 1, innerSwap = 0),
        innerIterMax = innerIterMax,
        timeLimitSP = 30.0
    )

    solver = Solver(
        res = h.res,
        data = h.data,
        neighborhoods = (
            TwoOptStar(),
            IntraShift(),
            InterShift{1}(),
            InterShift{2}(),
            InterSwap{1, 1}(),
            InterSwap{2, 1}(),
            InterSwap{2, 2}()
        ),
        algorithm = algorithm,
        penaltyManager = TargetRatePenaltyManager(),
        MIPSolver = CPLEX.Optimizer
    )

    setSeed!(solver, Random.MersenneTwister(seed))

    t   = @elapsed solve!(solver)
    sol = getBestSol(solver)

    printSol(sol, h.userIds)
    checkAgainstModel(data, sol, h.userIds)

    freeCppResource!(h.res.customResource)

    return sol.cost, t
end

instance = joinpath(@__DIR__, "..", "data", "Golden", "Golden_1.vrp")
data = readData(instance)
main(data, 3, 1)
