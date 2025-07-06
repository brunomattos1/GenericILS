function swap!(a::Vector{Int}, b::Vector{Int}, i::Int, j::Int, len::Int)
    for k in 1:len
        a[i+k], b[j+k] = b[j+k], a[i+k]
    end
end

function do_inter_two_opt_fwd!(
    eval_mode::EM,
    solver::Solver,
    r1::Int,
    r2::Int,
) where EM <: EvalMode
    sol = solver.work_sol
    route1 = sol.routes[r1]
    route2 = sol.routes[r2]
    cost_matrix = solver.cost
    best_cost = 0
    best_viol = 0
    best_i = 0
    best_j = 0
    prev_viol = 0
    l = sol.loads[r1][end]
    if l > solver.Q
        prev_viol += l - solver.Q
    end
    l = sol.loads[r2][end]
    if l > solver.Q
        prev_viol += l - solver.Q
    end
    n1 = length(route1)
    n2 = length(route2)
    for i in 1:(n1-1)
        for j in 1:(n2-1)
            cost = cost_matrix[route1[i], route2[j+1]] + cost_matrix[route2[j], route1[i+1]]
            cost -= cost_matrix[route1[i], route1[i+1]] + cost_matrix[route2[j], route2[j+1]]
            viol = -prev_viol
            l = sol.loads[r1][i] + sol.loads[r2][end] - sol.loads[r2][j]
            if l > solver.Q
                viol += l - solver.Q
            end
            l = sol.loads[r2][j] + sol.loads[r1][end] - sol.loads[r1][i]
            if l > solver.Q
                viol += l - solver.Q
            end
            if improves(eval_mode, cost, viol, best_cost, best_viol, solver.penal[1], i, j)
                best_cost = cost
                best_viol = viol
                best_i = i
                best_j = j
            end
        end
    end
    if best_i > 0
        len1 = n1 - best_i
        len2 = n2 - best_j
        if len1 < len2
            swap!(route1, route2, best_i + 1, best_j + 1, len1)
            resize!(route1, best_i + len2)
            copyto!(route1, n1 + 1, route2, best_j + 1 + len1, len2 - len1)
            resize!(route2, best_j + len1)
        else
            swap!(route1, route2, best_i + 1, best_j + 1, len2)
            resize!(route2, best_j + len1)
            copyto!(route2, n2 + 1, route1, best_i + 1 + len2, len1 - len2)
            resize!(route1, best_i + len2)
        end
        sol.cost += best_cost
        sol.viol += best_viol
    end
end

const NB_INTER_NEIGHS = 10
function search_inter_neigh!(
    eval_mode::EM,
    solver::Solver,
    neigh::Int,
    r1::Int,
    r2::Int,
) where EM <: EvalMode
    if neigh <= 5
        (neigh == 1) && return do_inter_two_opt_fwd!(eval_mode, solver, r1, r2)
        (neigh == 2) && return do_inter_two_opt_bwd!(eval_mode, solver, r1, r2)
        (neigh == 3) && return do_inter_shift_1!(eval_mode, solver, r1, r2)
        (neigh == 4) && return do_inter_shift_2!(eval_mode, solver, r1, r2)
        (neigh == 5) && return do_inter_swap_1_1!(eval_mode, solver, r1, r2)
    else
        (neigh == 6) && return do_inter_swap_2f_1!(eval_mode, solver, r1, r2)
        (neigh == 7) && return do_inter_swap_2b_1!(eval_mode, solver, r1, r2)
        (neigh == 8) && return do_inter_swap_2f_2f!(eval_mode, solver, r1, r2)
        (neigh == 9) && return do_inter_swap_2f_2b!(eval_mode, solver, r1, r2)
        (neigh == 10) && return do_inter_swap_2b_2b!(eval_mode, solver, r1, r2)
    end
end
