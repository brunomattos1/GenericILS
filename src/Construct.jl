"""Insert customer `customer` at position `pos` of route `route`."""
struct Insertion <: Move
    route::Int
    customer::Int
    pos::Int
end

"""Greedy construction of an initial solution (see `bestParallelInsertion`)."""
constructSol!(solver::Solver; r::Int = 0) = bestParallelInsertion(solver; r = r)

"""
Routes where a vertex of group `g` may still be inserted: the one its group is bound to when the group must
stay in a single route and already has a visit there, any of the first `nbRoutes` otherwise.
"""
allowedRoutes(data::ProblemData, groupRoute::Vector{Int}, g::Int, nbRoutes::Int) =
    data.groups[g].sameRoute && groupRoute[g] != 0 ? (groupRoute[g]:groupRoute[g]) : (1:nbRoutes)

"""
Account for the visit of vertex `id` in route `r`: one visit less is needed from its group (unless the vertex is
optional, which does not count), which is bound to `r` if it must stay in one route.
"""
function registerVisit!(data::ProblemData, need::Vector{Int}, groupRoute::Vector{Int}, id::Int, r::Int)
    g = data.groupOf[id]
    data.isOptional[id] || (need[g] -= 1)
    data.groups[g].sameRoute && (groupRoute[g] = r)
    return nothing
end

"""
Parallel greedy insertion: seed `nbRoutes` routes (at most `maxNbRoutes + r`) with random regular vertices, then
repeatedly insert the vertex/position with the lowest penalized cost, until every group has its `numVisits`
visits. Vertices of a group already satisfied are never inserted, and the visits of a `sameRoute` group all
go to the route of its first visit. Optional vertices are only inserted at the end, one by one, while the best
insertion reduces the objective (they are rewarded by the resource, for instance with a profit). Routes that
cannot be seeded with a regular vertex start empty. Also initializes the `lastEval` table.
"""
function bestParallelInsertion(solver::Solver; r::Int = 0)
    data = solver.data
    vertices = deepcopy(data.vertices)
    need = [g.numVisits for g in data.groups]
    groupRoute = zeros(Int, length(data.groups))
    remaining = sum(need; init = 0)
    # a route can be seeded by one visit of a sameRoute group, or by each of the visits of any other group
    maxSeeds = sum(g.numVisits == 0 ? 0 : (g.sameRoute ? 1 : g.numVisits) for g in data.groups; init = 0)
    nbRoutes = min(maxSeeds + length(data.optionalIds), data.maxNbRoutes + r)
    costMatrix = getCostMatrix(solver)
    sol = newSolution(solver)
    placed = 0                      # vertices[1:placed] are in the solution
    for r = 1:nbRoutes
        seeds = [i for i = placed+1:length(vertices)
                 if !data.isOptional[vertices[i].id] && need[data.groupOf[vertices[i].id]] > 0 &&
                    r in allowedRoutes(data, groupRoute, data.groupOf[vertices[i].id], nbRoutes)]
        if isempty(seeds)           # nothing left to start this route with
            push!(sol.routes, newRoute(solver, Int[0, 0]))
            continue
        end
        selectedIdx = seeds[rand(solver.seed, 1:length(seeds))]
        selected = vertices[selectedIdx]
        push!(sol.routes, newRoute(solver, Int[0, selected.id, 0]))
        sol.dist += costMatrix[1, selected.id+1] + costMatrix[selected.id+1, 1]
        sol.cost += costMatrix[1, selected.id+1] + costMatrix[selected.id+1, 1]
        placed += 1
        vertices[selectedIdx], vertices[placed] = vertices[placed], vertices[selectedIdx]
        registerVisit!(data, need, groupRoute, selected.id, r)
        remaining -= 1
    end
    computeLabels(solver, sol)
    for r = 1:length(sol.routes)
        rt = sol.routes[r]
        rt.labelCost = min(rt.forwardLabels[end].cost, rt.backwardLabels[end].cost)
    end
    sol.totalLabelCost = sum(rt.labelCost for rt in sol.routes)
    sol.cost = objectiveValue(solver, sol)
    k = placed + 1
    while remaining > 0
        bestI = 0
        bestInsertion = BestInsertion()
        for i = k:length(vertices)
            id = vertices[i].id
            data.isOptional[id] && continue
            g = data.groupOf[id]
            need[g] > 0 || continue
            for r in allowedRoutes(data, groupRoute, g, nbRoutes)
                for j = 2:length(sol.routes[r].visits)
                    dist, cost, infeas, warpStd1, warpStd2 = evalBestInsertion(solver, sol, Insertion(r, id, j))
                    if cost < bestInsertion.cost - 1e-6
                        bestInsertion = BestInsertion(cost, dist, r, id, j, infeas, warpStd1, warpStd2)
                        bestI = i
                    end
                end
            end
        end
        bestInsertion.pos > 0 || error("constructSol!: no insertion with finite cost for the $remaining remaining visits")
        applyMoveInsertion(solver, sol, bestInsertion)
        vertices[bestI], vertices[k] = vertices[k], vertices[bestI]
        registerVisit!(data, need, groupRoute, bestInsertion.customer, bestInsertion.route)
        k += 1
        remaining -= 1
    end
    while data.hasOptional
        bestI = 0
        bestInsertion = BestInsertion()
        for i = k:length(vertices)
            id = vertices[i].id
            data.isOptional[id] || continue
            for r in allowedRoutes(data, groupRoute, data.groupOf[id], nbRoutes)
                for j = 2:length(sol.routes[r].visits)
                    dist, cost, infeas, warpStd1, warpStd2 = evalBestInsertion(solver, sol, Insertion(r, id, j))
                    if cost < bestInsertion.cost - 1e-6
                        bestInsertion = BestInsertion(cost, dist, r, id, j, infeas, warpStd1, warpStd2)
                        bestI = i
                    end
                end
            end
        end
        bestInsertion.cost < sol.cost - 1e-6 || break       # no optional vertex pays off
        applyMoveInsertion(solver, sol, bestInsertion)
        vertices[bestI], vertices[k] = vertices[k], vertices[bestI]
        registerVisit!(data, need, groupRoute, bestInsertion.customer, bestInsertion.route)
        k += 1
    end
    R = length(sol.routes)
    nNeighborhoods = length(solver.neighborhoods)
    sol.timeStamp = 0
    sol.lastEval = zeros(Int, nNeighborhoods, R, R)

    for r in 1:R
        sol.lastEval[1, r, r] = 0
    end
    for move = 2:nNeighborhoods
        for r1 in 1:R-1
            for r2 in r1+1:R
                sol.lastEval[move, r1, r2] = 0
            end
        end
    end
    return sol
end

"""Apply an insertion to `solution`, updating labels and every bookkeeping field."""
function applyMoveInsertion(solver::Solver, solution::Solution, move::BestInsertion)
    r  = move.route
    i  = move.pos
    c  = move.customer
    rt = solution.routes[r]

    solution.dist = move.dist
    solution.totalInfeas   -= rt.infeas
    solution.totalInfeas   += move.infeas
    rt.infeas               = move.infeas

    solution.totalWarpStd1 -= rt.warpStd1
    solution.totalWarpStd1 += move.warpStd1
    rt.warpStd1             = move.warpStd1
    solution.totalWarpStd2 -= rt.warpStd2
    solution.totalWarpStd2 += move.warpStd2
    rt.warpStd2             = move.warpStd2

    insert!(rt.visits, i, c)
    computeLabels(solver, solution, r)

    solution.totalInfeas -= rt.infeas
    rt.infeas = length(rt.visits) - max(rt.feasibleF, rt.feasibleB) - 1
    solution.totalInfeas += rt.infeas

    solution.totalLabelCost -= rt.labelCost
    rt.labelCost = min(rt.forwardLabels[end].cost, rt.backwardLabels[end].cost)
    solution.totalLabelCost += rt.labelCost
    solution.cost = objectiveValue(solver, solution)
end

"""Distance of the route after inserting `customer` before position `j`, from the current distance."""
function bestInsertionCost(currCost::Float64, costMatrix::Matrix{Float64}, route::Vector{Int}, customer::Int, j::Int)
    newCost = currCost - costMatrix[route[j-1]+1, route[j]+1] + costMatrix[route[j-1]+1, customer+1] + costMatrix[customer+1, route[j]+1]
    return newCost
end

"""Evaluate an insertion: returns `(dist, cost, infeas, warpStd1, warpStd2)` of the resulting solution."""
function evalBestInsertion(solver::Solver, sol::Solution, insertion::Insertion)
    currCost = sol.dist
    r        = insertion.route
    customer = insertion.customer
    j        = insertion.pos
    dist = bestInsertionCost(currCost, solver.data.costMatrix, sol.routes[r].visits, customer, j)
    block = solver.bufferRoute
    resize!(block, 1)
    block[1] = customer
    infeas, labelCost, warpStd1, warpStd2 = infeasArcsInsertionK(solver, sol, r, block, j)
    cost = objectiveValue(solver, sol, r, dist, infeas, labelCost, warpStd1, warpStd2)
    return dist, cost, infeas, warpStd1, warpStd2
end
