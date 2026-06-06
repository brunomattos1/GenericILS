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
        solver.parameters.penaltyCustom,
        solver.parameters.penaltyStandard1, solver.parameters.penaltyStandard2,
        length(solver.route_storage), total_algorithm_time)

end



function manualCost(sol::Union{Solution, UserSolution}, costMatrix::Matrix{Float64})
    cost = 0.
    for r = 1:length(sol.routes)
        for i = 1:length(sol.routes[r])-1
            cost += costMatrix[sol.routes[r][i]+1, sol.routes[r][i+1]+1]
        end
    end
    return cost
end

function objectiveValue(solver::Solver, sol::Solution)
    objVal = sol.dist
    if isCostResource()
        objVal += sol.totalLabelCost
    end
    objVal += solver.parameters.penaltyCustom * (sol.totalInfeas)
    objVal += solver.parameters.penaltyStandard1 * (sol.totalWarpStd1)
    objVal += solver.parameters.penaltyStandard2 * (sol.totalWarpStd2)

    return objVal
end

function objectiveValue(solver::Solver, sol::Solution, r::Int, dist::Float64, infeas::Int, labelCost::Float64, warpStd1::Float64, warpStd2::Float64)
    objVal = dist
    if isCostResource()
        objVal += sol.totalLabelCost - sol.labelCosts[r] + labelCost
    end
    objVal += solver.parameters.penaltyCustom * (sol.totalInfeas - sol.infeas[r] + infeas)
    objVal += solver.parameters.penaltyStandard1 * (sol.totalWarpStd1 - sol.warpsStd1[r] + warpStd1)
    objVal += solver.parameters.penaltyStandard2 * (sol.totalWarpStd2 - sol.warpsStd2[r] + warpStd2)

    return objVal
end

function objectiveValue(solver::Solver, sol::Solution, costing::Cost)
    objVal = costing.dist
    if isCostResource()
        objVal += sol.totalLabelCost - sol.labelCosts[costing.route1] + costing.violInfo.firstRouteLabelCost - sol.labelCosts[costing.route2] + costing.violInfo.secondRouteLabelCost
    end
    objVal += solver.parameters.penaltyCustom * (sol.totalInfeas - sol.infeas[costing.route1]  + costing.violInfo.firstRouteInfeas - sol.infeas[costing.route2] + costing.violInfo.secondRouteInfeas)
    objVal += solver.parameters.penaltyStandard1 * (sol.totalWarpStd1 - sol.warpsStd1[costing.route1] + costing.warpStd1[1] - sol.warpsStd1[costing.route2] + costing.warpStd1[2])
    objVal += solver.parameters.penaltyStandard1 * (sol.totalWarpStd2 - sol.warpsStd2[costing.route1] + costing.warpStd2[1] - sol.warpsStd2[costing.route2] + costing.warpStd2[2])
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
        println("-"^100)
        println("Route $r: $(join(sol.routes[r], " -> "))")
        println("-"^100)

        for i = 1:length(sol.routes[r])
            route_prefix = sol.routes[r][1:i]
            label = sol.forwardLabels[r][i]
            print_label(route_prefix, label)
        end
    end
    println("#"^100)
    println("BACKWARD LABELS:")
    println("#"^100)

    for r = 1:length(sol.routes)
        println("-"^100)
        println("Route $r: $(join(sol.routes[r], " -> "))")
        println("-"^100)
        n = length(sol.routes[r])
        for i = 1:n
            start = n - i + 1
            route_prefix = sol.routes[r][start:n]
            label = sol.backwardLabels[r][i]
            print_label(route_prefix, label)
        end
    end
end

function printConcatenations(solver::Solver, sol::Union{Solution, UserSolution})
    for r = 1:length(sol.routes)
        println("-"^100)
        println("Route $r: $(join(sol.routes[r], " -> "))")
        println("-"^100)
        lenR = length(sol.routes[r])
        for i in 1:lenR
            prefix = sol.routes[r][1:i]
            suffix = sol.routes[r][i:end]
            concat = myConcatenationCost(
                solver.res,
                1,
                sol.forwardLabels[r][i],
                sol.backwardLabels[r][lenR - i + 1]
            )
            println("Concatenating $(join(prefix, " -> ")) with $(join(suffix, " -> "))")
            print_label(concat)
        end
    end
end

function createSolution(
    solver::Solver,
    routes::Vector{Vector{Int}};
    dist::Union{Float64,Nothing}=nothing,
    cost::Union{Float64,Nothing}=nothing)

    sol = UserSolution(
        routes,
        0.0,
        0.0,
        Vector{Vector{ForwardLabel}}(),
        Vector{Vector{BackwardLabel}}(),
    )

    sol.dist = isnothing(dist) ? manualCost(sol, solver.data.costMatrix) : dist

    computeLabels(solver, sol)

    sol.cost = isnothing(cost) ? sol.dist + sum(min(sol.forwardLabels[r][end].cost,sol.backwardLabels[r][end].cost) for r in 1:length(sol.routes)) : cost

    return sol
end

