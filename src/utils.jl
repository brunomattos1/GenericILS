"""
Exchange two blocks between routes: `k1` entries of `route1` starting at `i` go to `route2` at `j`, and
`k2` entries of `route2` starting at `j` go to `route1` at `i` (`k1` or `k2` may be 0; sizes may differ).
`buffer` is scratch space.
"""
function moveBlocks!(route1::Vector{Int}, i::Int, k1::Int,
                      route2::Vector{Int}, j::Int, k2::Int,
                      buffer::Vector{Int})
    n1 = length(route1)
    n2 = length(route2)

    if k1 > 0
        # the buffer may not have been sized yet (e.g. perturbation without InterShift/InterSwap neighborhoods)
        length(buffer) < k1 && resize!(buffer, k1)
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

"""
Move the block of `k` entries starting at `i` inside `route`. For `i > j` the block starts at `j`; for
`i < j` it ends right before the old entry `j`. `buffer` is scratch space.
"""
function moveBlocksIntra!(route::Vector{Int}, i::Int, k::Int, j::Int, buffer::Vector{Int})
    if i == j || k == 0
        return nothing
    end
    resize!(buffer, k)
    copyto!(buffer, 1, route, i, k)
    if i < j
        shiftLen = j - i - k
        if shiftLen > 0
            copyto!(route, i, route, i + k, shiftLen)
        end
        copyto!(route, j - k, buffer, 1, k)
    else
        shiftLen = i - j
        if shiftLen > 0
            copyto!(route, j + k, route, j, shiftLen)
        end
        copyto!(route, j, buffer, 1, k)
    end
    return nothing
end

"""Seconds spent so far by `algo`. Each algorithm defines its own method."""
totalTime(algo::Algorithm) = error("totalTime not implemented for $(typeof(algo))")

"""Print one line of progress information for the running algorithm."""
printInfo(solver::Solver) = printInfo(solver.algorithm, solver)
printInfo(algo::Algorithm, solver::Solver) = nothing  # default: no-op for algorithms that don't define progress reporting



"""Total distance of a solution, recomputed from its visit sequences."""
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

"""
Part of the penalized cost that does not change when routes `r1` and `r2` are modified. Used to prune
moves by distance alone.
"""
function pruningFixedPenalty(solver::Solver, sol::Solution, r1::Int, r2::Int)
    rt1 = sol.routes[r1]
    rt2 = sol.routes[r2]
    fixedPenalty = sol.cost - sol.dist
    fixedPenalty -= solver.penaltyManager.penaltyCustom    * (rt1.infeas   + rt2.infeas)
    fixedPenalty -= solver.penaltyManager.penaltyStandard1 * (rt1.warpStd1 + rt2.warpStd1)
    fixedPenalty -= solver.penaltyManager.penaltyStandard2 * (rt1.warpStd2 + rt2.warpStd2)
    return fixedPenalty
end

"""Same as above for a move that touches the single route `r`."""
function pruningFixedPenalty(solver::Solver, sol::Solution, r::Int)
    rt = sol.routes[r]
    fixedPenalty = sol.cost - sol.dist
    fixedPenalty -= solver.penaltyManager.penaltyCustom    * rt.infeas
    fixedPenalty -= solver.penaltyManager.penaltyStandard1 * rt.warpStd1
    fixedPenalty -= solver.penaltyManager.penaltyStandard2 * rt.warpStd2
    return fixedPenalty
end

"""Pruning moves by distance is only valid when the custom resource adds no cost."""
canPruneByDist(solver::Solver) = !isCostResource()

"""Penalized objective of `sol`: distance (+ custom cost) plus the penalty of each lane times its violation."""
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

"""
Objective of `sol` if route `r` were replaced by one with the given distance, infeasibility, label cost
and warps.
"""
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

"""Objective of `sol` after a move that changes two routes, described by `costing`."""
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

"""Format the fields of a state as `Name: (field: value, ...)`."""
function formatState(state)
    T = typeof(state)
    stateName = nameof(T)

    parts = String[]
    for f in fieldnames(T)
        value = getfield(state, f)
        push!(parts, "$(f): $(value)")
    end

    return "$stateName: ($(join(parts, ", ")))"
end

"""Print a label together with the partial route it summarizes."""
function printLabel(routePrefix, label)

    # 1) print the partial route
    print("Partial label: ")
    println(join(routePrefix, " -> "))

    stateParts = String[]
    otherParts = String[]

    for fname in fieldnames(typeof(label))
        value = getfield(label, fname)

        if isstructtype(typeof(value))
            push!(stateParts, formatState(value))
        else
            push!(otherParts, "$(fname): $(value)")
        end
    end
    # 2) print the states below the route
    line = "  States: $(join(stateParts, ", "))"

    if !isempty(otherParts)
        line *= ", $(join(otherParts, ", "))"
    end

    println(line)
    println()
end

"""Print a label (states and remaining fields)."""
function printLabel(label)
    stateParts = String[]
    otherParts = String[]

    for fname in fieldnames(typeof(label))
        value = getfield(label, fname)

        if isstructtype(typeof(value))
            push!(stateParts, formatState(value))
        else
            push!(otherParts, "$(fname): $(value)")
        end
    end

    allParts = vcat(
        ["States: $(join(stateParts, ", "))"],
        otherParts
    )

    println("  " * join(allParts, ", "))
    println()
end


"""Print forward and backward labels of every route of `sol`."""
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
            routePrefix = rt.visits[1:i]
            label = rt.forwardLabels[i]
            printLabel(routePrefix, label)
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
            routePrefix = rt.visits[start:n]
            label = rt.backwardLabels[i]
            printLabel(routePrefix, label)
        end
    end
end

"""Print the concatenation of the forward and backward labels at every position of every route."""
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
            printLabel(concat)
        end
    end
end

"""
Build a `UserSolution` from raw routes (vectors starting and ending at depot `0`), computing labels, distance
and cost unless `dist`/`cost` are given.
"""
function createSolution(
    solver::Solver{N, R, PM, FL, BL, AL},
    routes::Vector{Vector{Int}};
    dist::Union{Float64,Nothing}=nothing,
    cost::Union{Float64,Nothing}=nothing) where {N, R <: AbstractResources, PM, FL, BL, AL}

    sol = UserSolution{FL, BL}(routes, 0.0, 0.0)

    sol.dist = isnothing(dist) ? manualCost(sol, solver.data.costMatrix) : dist

    computeLabels(solver, sol)

    sol.cost = isnothing(cost) ? sol.dist + sum(min(sol.routes[r].forwardLabels[end].cost, sol.routes[r].backwardLabels[end].cost) for r in 1:length(sol.routes)) : cost

    return sol
end

