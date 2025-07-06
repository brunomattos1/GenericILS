@with_kw struct Config
    max_nb_closest::Int
end

mutable struct InsertionChoice
    cost::Int
    l::Int
    route::Int
    pos::Int
end

mutable struct Solution
    routes::Vector{Vector{Int}}
    loads::Vector{Vector{Int}}
    cost::Int
    viol::Int
end

function get_load(loads::Vector{Vector{Int}}, i::Int)
    return loads[i][end]
end

struct Solver
    cfg::Config
    rng::MersenneTwister
    Q::Int
    customers::Vector{Int}
    demands::Vector{Int}
    cost::Matrix{Int}
    work_sol::Solution
    curr_sol::Solution
    best_sol::Solution
    unused_routes::Vector{Vector{Int}}
    route_costs::Vector{Int}
    inter_neighs::Vector{Int}
    intra_neighs::Vector{Int}
    penal::Vector{Float64}
end

abstract type EvalMode end
struct HierarquicalEval <: EvalMode end
struct PenalizedEval <: EvalMode end
struct TestEval <: EvalMode
    i::Int
    j::Int
end

function improves(
    _::HierarquicalEval,
    cost::Int,
    viol::Int,
    best_cost::Int,
    best_viol::Int,
    _::Float64,
    _::Int,
    _::Int,
)
    if viol < best_viol
        return true
    end
    return cost < best_cost
end

function improves(
    _::PenalizedEval,
    cost::Int,
    viol::Int,
    best_cost::Int,
    best_viol::Int,
    penal::Float64,
    _::Int,
    _::Int,
)
    return cost + penal * viol < best_cost + penal * best_viol - 1e-5
end

function improves(
    eval_move::TestEval,
    _::Int,
    _::Int,
    _::Int,
    _::Int,
    _::Float64,
    i::Int,
    j::Int,
)
    return i == eval_move.i && j == eval_move.j
end

include("intra_neighs.jl")
include("inter_neighs.jl")

function copy_to!(dst::Solution, src::Solution)
    resize!(dst.routes, length(src.routes))
    for i in eachindex(src.routes)
        resize!(dst.routes[i], length(src.routes[i]))
        dst.routes[i] .= src.routes[i]
        resize!(dst.loads[i], length(src.loads[i]))
        dst.loads[i] .= src.loads[i]
    end
    dst.cost = src.cost
end

function routes_resize!(solver::Solver, routes::Vector{Vector{Int}}, new_size::Int)
    old_size = length(routes)
    for i in (new_size+1):old_size
        push!(solver.unused_routes, routes[i])
    end
    resize!(routes, new_size)
    for i in (old_size+1):new_size
        if length(solver.unused_routes) > 0
            routes[i] = pop!(solver.unused_routes)
            empty!(routes[i])
        else
            routes[i] = Int[]
        end
    end
end

function insertion_cost(
    cost_matrix::Matrix{Int},
    c::Int,
    route::Vector{Int},
    pos::Int,
)
    cost = cost_matrix[route[pos-1], c] + cost_matrix[c, route[pos]]
    cost -= cost_matrix[route[pos-1], route[pos]]
    return cost
end

function best_insertion(
    solver::Solver,
    routes::Vector{Vector{Int}},
    loads::Vector{Vector{Int}},
    i::Int,
    nb_routes::Int,
)
    best_cost = typemax(Int)
    best_l = 0
    best_route = 0
    best_pos = 0
    cost_matrix = solver.cost
    for l in i:length(solver.customers)
        c = solver.customers[l]
        d = solver.demands[c]
        for j in 1:nb_routes
            if get_load(loads, j) + d > solver.Q
                continue
            end
            route_cost = solver.route_costs[j]
            route = routes[j]
            prev_best_cost = best_cost
            max_pos = length(route)
            for k in 2:max_pos
                cost = route_cost + insertion_cost(cost_matrix, c, route, k)
                if cost < best_cost
                    best_cost = cost
                    best_pos = k
                end
            end
            if best_cost < prev_best_cost
                best_l = l
                best_route = j
            end
        end
    end
    return best_cost, best_l, best_route, best_pos
end

function construct!(solver::Solver)
    # Set the number of routes
    nb_routes = ceil(Int, sum(solver.demands) / solver.Q)

    # Initialize the solution with single-customer routes
    sol = solver.work_sol
    routes_resize!(solver, sol.routes, nb_routes)
    routes_resize!(solver, sol.loads, nb_routes)
    resize!(solver.route_costs, nb_routes)
    sol.cost = 0
    for i in 1:nb_routes
        sel = rand(solver.rng, i:length(solver.customers))
        empty!(sol.routes[i])
        push!(sol.routes[i], 1)
        push!(sol.routes[i], solver.customers[sel])
        push!(sol.routes[i], 1)
        empty!(sol.loads[i])
        push!(sol.loads[i], 0)
        push!(sol.loads[i], solver.demands[solver.customers[sel]])
        push!(sol.loads[i], solver.demands[solver.customers[sel]])
        solver.customers[sel], solver.customers[i] =
            solver.customers[i], solver.customers[sel]
        solver.route_costs[i] = solver.cost[1, solver.customers[i]] * 2
        sol.cost += solver.route_costs[i]
    end

    # Insert the remaining customers in a greedy way
    for i in (nb_routes+1):length(solver.customers)
        best_cost, best_l, best_route, best_pos =
            best_insertion(solver, sol.routes, sol.loads, i, nb_routes)
        if best_route == 0
            nb_routes += 1
            routes_resize!(solver, sol.routes, nb_routes)
            push!(sol.routes[nb_routes], 1)
            push!(sol.routes[nb_routes], 1)
            routes_resize!(solver, sol.loads, nb_routes)
            push!(sol.loads[nb_routes], 0)
            push!(sol.loads[nb_routes], 0)
            resize!(solver.route_costs, nb_routes)
            for l in i:length(solver.customers)
                c = solver.customers[l]
                cost = solver.cost[1, c] * 2
                if cost < best_cost
                    best_l = l
                    best_cost = cost
                    best_route = nb_routes
                    best_pos = 2
                end
            end
        end
        solver.customers[best_l], solver.customers[i] =
            solver.customers[i], solver.customers[best_l]
        c = solver.customers[i]
        insert!(sol.routes[best_route], best_pos, c)
        load = solver.demands[c] + sol.loads[best_route][best_pos-1]
        insert!(sol.loads[best_route], best_pos, load)
        for j in (best_pos+1):length(sol.routes[best_route])
            sol.loads[best_route][j] += solver.demands[c]
        end
        solver.route_costs[best_route] = best_cost
    end
    sol.cost = sum(solver.route_costs)
end

function do_rand_descent!(solver::Solver)
    shuffle!(solver.rng, solver.inter_neighs)
    changed = true
    while changed
        changed = false
        for neigh in solver.inter_neighs
        end
    end
end

function build_solver(data::DataCVRP, cfg::Config)
    demands = [Int(i.demand) for i in data.G.V]
    n = length(data.G.V)
    cost = zeros(Int, n, n)
    for i in 1:n
        for j in 1:n
            cost[i, j] = distance(data, (i - 1, j - 1))
        end
    end
    closest = [[j for j in 1:n if j != i] for i in 1:n]
    for i in 1:n
        partialsort!(closest[i], 1:min(n, cfg.max_nb_closest), by = j -> cost[i, j])
        if n > cfg.max_nb_closest
            resize!(closest[i], cfg.max_nb_closest)
        end
    end
    return Solver(
        cfg,
        MersenneTwister(),
        data.Q,
        2:length(data.G.V),
        demands,
        cost,
        Solution(Vector{Int}[], Vector{Int}[], 0, 0),
        Solution(Vector{Int}[], Vector{Int}[], 0, 0),
        Solution(Vector{Int}[], Vector{Int}[], 0, 0),
        Int[],
        Int[],
        1:NB_INTER_NEIGHS,
        1:NB_INTRA_NEIGHS,
        [0.0],
    )
end

function solve(data::DataCVRP, cfg::Config)
    solver = build_solver(data, cfg)
    @time construct!(solver)
    return solver.work_sol
end

function check_solution(data::DataCVRP, sol::Solution)
    n = length(data.G.V)
    visited = zeros(Bool, n)
    for route in sol.routes
        for i in 2:(length(route)-1)
            visited[route[i]] = true
        end
    end
    for i in 2:n
        if !visited[i]
            println("Customer $i not visited")
        end
    end
end
