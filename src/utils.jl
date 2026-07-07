function move_blocks!(route1::Vector{Int}, i::Int, k1::Int,
                      route2::Vector{Int}, j::Int, k2::Int,
                      buffer::Vector{Int})
    n1 = length(route1)
    n2 = length(route2)

    if k1 > 0
        copyto!(buffer, 1, route1, i, k1)
    end

    if k2 > k1
        resize!(route1, n1 + k2 - k1)
        copyto!(route1, i + k2, route1, i + k1, n1 - i - k1 + 1)
    elseif k2 < k1
        copyto!(route1, i + k2, route1, i + k1, n1 - i - k1 + 1)
        resize!(route1, n1 + k2 - k1)
    end

    if k2 > 0
        copyto!(route1, i, route2, j, k2)
    end

    if k1 > k2
        resize!(route2, n2 + k1 - k2)
        copyto!(route2, j + k1, route2, j + k2, n2 - j - k2 + 1)
    elseif k1 < k2
        copyto!(route2, j + k1, route2, j + k2, n2 - j - k2 + 1)
        resize!(route2, n2 + k1 - k2)
    end

    if k1 > 0
        copyto!(route2, j, buffer, 1, k1)
    end
end

function move_blocks_intra!(route::Vector{Int}, i::Int, k::Int, j::Int, buffer::Vector{Int})
    if i == j || k == 0
        return nothing
    end
    resize!(buffer, k)
    copyto!(buffer, 1, route, i, k)
    if i < j
        shift_len = j - i - k
        if shift_len > 0
            copyto!(route, i, route, i + k, shift_len)
        end
        copyto!(route, j - k, buffer, 1, k)
    else
        shift_len = i - j
        if shift_len > 0
            copyto!(route, j + k, route, j, shift_len)
        end
        copyto!(route, j, buffer, 1, k)
    end
    return nothing
end

totalTime(solver::Solver) = time() - solver.startTime
get_stop_info(solver::Solver, ::Any) = NaN  # default (não tem temperatura)

get_stop_info(solver::Solver, c::ByIterMax) = solver.iter  # default (não tem temperatura)

get_stop_info(solver::Solver, c::ByTemperature) = solver.acceptCriteria.temperature

function printInfo(solver::Solver)
    total_algorithm_time = time() - solver.startTime
    stopInfo = get_stop_info(solver, solver.stopCriteria)
    if mod(total_algorithm_time, 10.0) == 0
        println("-"^135)
        @printf("| %10s | %10s | %12s | %12s | %10s | %15s | %15s | %6s | %10s |\n",
            "Temp.", "Best Feas", "Best", "Candidate",
            "Pen. Custom", "Pen. Standard 1", "Pen. Standard 2", "Pool", "Time (s)")
        println("-"^135)
        # header_time += 10.0
    end

    @printf("| %10.6f | %10.2f | %12.2f | %12.2f | %11.2f | %15.2f | %15.2f | %6d | %10.4f |\n",
        stopInfo,
        solver.bestFeasSol.cost, solver.outerBestSol.cost, solver.outerCandidateSol.cost,
        solver.penaltyManager.penaltyCustom,
        solver.penaltyManager.penaltyStandard1, solver.penaltyManager.penaltyStandard2,
        length(solver.route_storage), total_algorithm_time)

end



function manualCost(sol::Union{Solution, UserSolution}, costMatrix::Matrix{Float64})
    cost = 0.
    for r = 1:length(sol.routes)
        visits = sol.routes[r].visits
        for i = 1:length(visits)-1
            cost += costMatrix[visits[i]+1, visits[i+1]+1]
        end
    end
    return cost
end

# Custo fixo do resto da solucao (tudo exceto dist e as violacoes das rotas envolvidas),
# assumindo o melhor caso possivel (violacoes novas = 0). Usado para podar um movimento
# sem calcular suas violacoes: se dist + fixedPenalty >= bestCost, o movimento nao pode
# melhorar. So valido quando isCostResource() == false (labelCost sem sinal garantido).
function pruningFixedPenalty(solver::Solver, sol::Solution, r1::Int, r2::Int)
    rt1 = sol.routes[r1]
    rt2 = sol.routes[r2]
    fixedPenalty = sol.cost - sol.dist
    fixedPenalty -= solver.penaltyManager.penaltyCustom    * (rt1.infeas   + rt2.infeas)
    fixedPenalty -= solver.penaltyManager.penaltyStandard1 * (rt1.warpStd1 + rt2.warpStd1)
    fixedPenalty -= solver.penaltyManager.penaltyStandard2 * (rt1.warpStd2 + rt2.warpStd2)
    return fixedPenalty
end

function pruningFixedPenalty(solver::Solver, sol::Solution, r::Int)
    rt = sol.routes[r]
    fixedPenalty = sol.cost - sol.dist
    fixedPenalty -= solver.penaltyManager.penaltyCustom    * rt.infeas
    fixedPenalty -= solver.penaltyManager.penaltyStandard1 * rt.warpStd1
    fixedPenalty -= solver.penaltyManager.penaltyStandard2 * rt.warpStd2
    return fixedPenalty
end

canPruneByDist(solver::Solver) = !isCostResource()

function objectiveValue(solver::Solver, sol::Solution)
    objVal = sol.dist
    if isCostResource()
        objVal += sol.totalLabelCost
    end
    objVal += solver.penaltyManager.penaltyCustom * (sol.totalInfeas)
    objVal += solver.penaltyManager.penaltyStandard1 * (sol.totalWarpStd1)
    objVal += solver.penaltyManager.penaltyStandard2 * (sol.totalWarpStd2)

    return objVal
end

function objectiveValue(solver::Solver, sol::Solution, r::Int, dist::Float64, infeas::Int, labelCost::Float64, warpStd1::Float64, warpStd2::Float64)
    rt = sol.routes[r]
    objVal = dist
    if isCostResource()
        objVal += sol.totalLabelCost - rt.labelCost + labelCost
    end
    objVal += solver.penaltyManager.penaltyCustom    * (sol.totalInfeas   - rt.infeas   + infeas)
    objVal += solver.penaltyManager.penaltyStandard1 * (sol.totalWarpStd1 - rt.warpStd1 + warpStd1)
    objVal += solver.penaltyManager.penaltyStandard2 * (sol.totalWarpStd2 - rt.warpStd2 + warpStd2)

    return objVal
end

function objectiveValue(solver::Solver, sol::Solution, costing::Cost)
    rt1 = sol.routes[costing.route1]
    rt2 = sol.routes[costing.route2]
    objVal = costing.dist
    if isCostResource()
        objVal += sol.totalLabelCost - rt1.labelCost + costing.violInfo.firstRouteLabelCost - rt2.labelCost + costing.violInfo.secondRouteLabelCost
    end
    objVal += solver.penaltyManager.penaltyCustom    * (sol.totalInfeas   - rt1.infeas   + costing.violInfo.firstRouteInfeas  - rt2.infeas   + costing.violInfo.secondRouteInfeas)
    objVal += solver.penaltyManager.penaltyStandard1 * (sol.totalWarpStd1 - rt1.warpStd1 + costing.warpStd1[1]               - rt2.warpStd1 + costing.warpStd1[2])
    objVal += solver.penaltyManager.penaltyStandard2 * (sol.totalWarpStd2 - rt1.warpStd2 + costing.warpStd2[1]               - rt2.warpStd2 + costing.warpStd2[2])
    return objVal
end

# -------- Formata apenas os campos internos do state --------
function format_state(state)
    T = typeof(state)
    state_name = nameof(T)

    parts = String[]
    for f in fieldnames(T)
        value = getfield(state, f)
        push!(parts, "$(f): $(value)")
    end

    return "$state_name: ($(join(parts, ", ")))"
end

# -------- Formata o label completo --------
function print_label(route_prefix, label)

    # 1) imprime rota parcial
    print("Partial label: ")
    println(join(route_prefix, " -> "))

    state_parts = String[]
    other_parts = String[]

    for fname in fieldnames(typeof(label))
        value = getfield(label, fname)

        if isstructtype(typeof(value))
            push!(state_parts, format_state(value))
        else
            push!(other_parts, "$(fname): $(value)")
        end
    end
    # 2) imprime states abaixo da rota
    line = "  States: $(join(state_parts, ", "))"

    if !isempty(other_parts)
        line *= ", $(join(other_parts, ", "))"
    end

    println(line)
    println()
end

function print_label(label)
    state_parts = String[]
    other_parts = String[]

    for fname in fieldnames(typeof(label))
        value = getfield(label, fname)

        if isstructtype(typeof(value))
            push!(state_parts, format_state(value))
        else
            push!(other_parts, "$(fname): $(value)")
        end
    end

    all_parts = vcat(
        ["States: $(join(state_parts, ", "))"],
        other_parts
    )

    println("  " * join(all_parts, ", "))
    println()
end


function printLabels(solver::Solver, sol::Union{Solution, UserSolution})
    computeLabels(solver, sol)
    println("#"^100)
    println("FORWARD LABELS:")
    println("#"^100)

    # -------- Seu loop --------
    for r = 1:length(sol.routes)
        rt = sol.routes[r]
        println("-"^100)
        println("Route $r: $(join(rt.visits, " -> "))")
        println("-"^100)

        for i = 1:length(rt.visits)
            route_prefix = rt.visits[1:i]
            label = rt.forwardLabels[i]
            print_label(route_prefix, label)
        end
    end
    println("#"^100)
    println("BACKWARD LABELS:")
    println("#"^100)

    for r = 1:length(sol.routes)
        rt = sol.routes[r]
        println("-"^100)
        println("Route $r: $(join(rt.visits, " -> "))")
        println("-"^100)
        n = length(rt.visits)
        for i = 1:n
            start = n - i + 1
            route_prefix = rt.visits[start:n]
            label = rt.backwardLabels[i]
            print_label(route_prefix, label)
        end
    end
end

function printConcatenations(solver::Solver, sol::Union{Solution, UserSolution})
    for r = 1:length(sol.routes)
        rt = sol.routes[r]
        println("-"^100)
        println("Route $r: $(join(rt.visits, " -> "))")
        println("-"^100)
        lenR = length(rt.visits)
        for i in 1:lenR
            prefix = rt.visits[1:i]
            suffix = rt.visits[i:end]
            concat = myConcatenationCost(
                solver.res,
                1,
                rt.forwardLabels[i],
                rt.backwardLabels[lenR - i + 1]
            )
            println("Concatenating $(join(prefix, " -> ")) with $(join(suffix, " -> "))")
            print_label(concat)
        end
    end
end

function createSolution(
    solver::Solver{N, AC, SC, R, PM, FL, BL},
    routes::Vector{Vector{Int}};
    dist::Union{Float64,Nothing}=nothing,
    cost::Union{Float64,Nothing}=nothing) where {N, AC, SC, R <: AbstractResources, PM, FL, BL}

    sol = UserSolution{FL, BL}(routes, 0.0, 0.0)

    sol.dist = isnothing(dist) ? manualCost(sol, solver.data.costMatrix) : dist

    computeLabels(solver, sol)

    sol.cost = isnothing(cost) ? sol.dist + sum(min(sol.routes[r].forwardLabels[end].cost, sol.routes[r].backwardLabels[end].cost) for r in 1:length(sol.routes)) : cost

    return sol
end

