function do_intra_two_opt!(solver::Solver, r::Int)
    sol = solver.work_sol
    route = sol.routes[r]
    cost_matrix = solver.cost
    best_cost = 0
    best_i = 0
    best_j = 0
    n = length(route)
    for i in 1:(n-2)
        for j in (i+2):n
            cost = cost_matrix[route[i], route[j]] + cost_matrix[route[i+1], route[j+1]]
            cost -= cost_matrix[route[i], route[i+1]] + cost_matrix[route[j], route[j+1]]
            if improves(eval_mode, cost, 0, best_cost, 0, 0.0, i, j)
                best_cost = cost
                best_i = i
                best_j = j
            end
        end
    end
    if best_i > 0
        reverse!(route, best_i + 1, best_j)
        sol.cost += best_cost
    end
end

function do_intra_shift_1!(solver::Solver, r::Int)
    sol = solver.work_sol
    route = sol.routes[r]
    cost_matrix = solver.cost
    best_cost = 0
    best_i = 0
    best_j = 0
    n = length(route)
    for i in 1:(n-1)
        for j in 2:(n-1)
            if i == j || i + 1 == j
                continue
            end
            cost =
                cost_matrix[route[i], route[j]] + cost_matrix[route[j], route[i+1]] +
                cost_matrix[route[j-1], route[j+1]]
            cost -=
                cost_matrix[route[i], route[i+1]] + cost_matrix[route[j-1], route[j]] +
                cost_matrix[route[j], route[j+1]]
            if improves(eval_mode, cost, 0, best_cost, 0, 0.0, i, j)
                best_cost = cost
                best_i = i
                best_j = j
            end
        end
    end
    if best_i > 0
        insert!(route, best_i + 1, route[best_j])
        if best_j < best_i
            deleteat!(route, best_j)
        else
            deleteat!(route, best_j + 1)
        end
        sol.cost += best_cost
    end
end

function insert_2!(route::Vector{Int}, pos::Int, i::Int, j::Int)
    n = length(route)
    resize!(route, n + 2)
    for k in n:-1:pos
        route[k+2] = k
    end
    route[pos] = i
    route[pos+1] = j
end

function deleteat_2!(route::Vector{Int}, pos::Int)
    n = length(route)
    for i in pos:(n-2)
        route[i] = route[i+2]
    end
    resize!(route, n - 2)
end

function do_intra_shift_2f!(solver::Solver, r::Int)
    sol = solver.work_sol
    route = sol.routes[r]
    cost_matrix = solver.cost
    best_cost = 0
    best_i = 0
    best_j = 0
    n = length(route)
    for i in 1:(n-1)
        for j in 2:(n-2)
            if i == j + 1 || i == j || i + 1 == j
                continue
            end
            cost =
                cost_matrix[route[i], route[j]] + cost_matrix[route[j+1], route[i+1]] +
                cost_matrix[route[j-1], route[j+2]]
            cost -=
                cost_matrix[route[i], route[i+1]] + cost_matrix[route[j-1], route[j]] +
                cost_matrix[route[j+1], route[j+2]]
            if improves(eval_mode, cost, 0, best_cost, 0, 0.0, i, j)
                best_cost = cost
                best_i = i
                best_j = j
            end
        end
    end
    if best_i > 0
        insert_2!(route, best_i + 1, route[best_j], route[best_j+1])
        if best_j < best_i
            deleteat_2!(route, best_j)
        else
            deleteat_2!(route, best_j + 2)
        end
        sol.cost += best_cost
    end
end

function do_intra_shift_2f!(solver::Solver, r::Int)
    sol = solver.work_sol
    route = sol.routes[r]
    cost_matrix = solver.cost
    best_cost = 0
    best_i = 0
    best_j = 0
    n = length(route)
    for i in 1:(n-1)
        for j in 2:(n-2)
            if i == j + 1 || i == j || i + 1 == j
                continue
            end
            cost =
                cost_matrix[route[i], route[j+1]] + cost_matrix[route[j], route[i+1]] +
                cost_matrix[route[j-1], route[j+2]]
            cost -=
                cost_matrix[route[i], route[i+1]] + cost_matrix[route[j-1], route[j]] +
                cost_matrix[route[j+1], route[j+2]]
            if improves(eval_mode, cost, 0, best_cost, 0, 0.0, i, j)
                best_cost = cost
                best_i = i
                best_j = j
            end
        end
    end
    if best_i > 0
        insert_2!(route, best_i + 1, route[best_j+1], route[best_j])
        if best_j < best_i
            deleteat_2!(route, best_j)
        else
            deleteat_2!(route, best_j + 2)
        end
        sol.cost += best_cost
    end
end

function do_intra_swap_1_1!(solver::Solver, r::Int)
    sol = solver.work_sol
    route = sol.routes[r]
    cost_matrix = solver.cost
    best_cost = 0
    best_i = 0
    best_j = 0
    n = length(route)
    for i in 2:(n-2)
        for j in (i+1):(n-1)
            if j == i + 1
                cost =
                    cost_matrix[route[i-1], route[j]] + cost_matrix[route[j], route[i]] +
                    cost_matrix[route[i], route[j+1]]
                cost -=
                    cost_matrix[route[i-1], route[i]] + cost_matrix[route[i], route[j]] +
                    cost_matrix[route[j], route[j+1]]
            else
                cost =
                    cost_matrix[route[i-1], route[j]] + cost_matrix[route[j], route[i+1]] +
                    cost_matrix[route[j-1], route[i]] + cost_matrix[route[i], route[j+1]]
                cost -=
                    cost_matrix[route[i-1], route[i]] + cost_matrix[route[i], route[i+1]] +
                    cost_matrix[route[j-1], route[j]] + cost_matrix[route[j], route[j+1]]
            end
            if improves(eval_mode, cost, 0, best_cost, 0, 0.0, i, j)
                best_cost = cost
                best_i = i
                best_j = j
            end
        end
    end
    if best_i > 0
        route[best_i], route[best_j] = route[best_j], route[best_i]
        sol.cost += best_cost
    end
end

const NB_INTRA_NEIGHS = 10
function search_inter_neigh!(solver::Solver, neigh::Int, r::Int)
    if neigh <= 5
        (neigh == 1) && return do_intra_two_opt!(solver, r)
        (neigh == 2) && return do_intra_shift_1!(solver, r)
        (neigh == 3) && return do_intra_shift_2f!(solver, r)
        (neigh == 4) && return do_intra_shift_2b!(solver, r)
        (neigh == 5) && return do_intra_swap_1_1!(solver, r)
    else
        (neigh == 6) && return do_intra_swap_2f_1!(solver, r)
        (neigh == 7) && return do_intra_swap_2b_1!(solver, r)
        (neigh == 8) && return do_intra_swap_2f_2f!(solver, r)
        (neigh == 9) && return do_intra_swap_2f_2b!(solver, r)
        (neigh == 10) && return do_intra_swap_2b_2b!(solver, r)
    end
end
