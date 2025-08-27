function hash(route::Vector{Int})
    edges = Vector{Tuple{Int, Int}}()
    for i = 1:length(route) - 1
        push!(edges, (route[i], route[i+1]))
    end
    return hash(edges)
end


function push!(solver::Solver)
    for r = 1:length(solver.currSol.routes)
        if (solver.currSol.feasiblesF[r] == length(solver.currSol.routes[r]) - 2) || (solver.currSol.feasiblesB[r] == length(solver.currSol.routes[r]) - 2)
            routeHash = hash(solver.currSol.routes[r])
            if (!(routeHash in solver.hashes))
                push!(solver.hashes, routeHash)
                push!(solver.pool, copy(solver.currSol.routes[r]))
            end
        end
    end
end

function α(i::Int, route::Vector{Int})
    return i in route
end

function c(solver::Solver, r::Int)
    cost = 0.
    for i = 1:length(solver.pool[r])-1
        cost += solver.data.costMatrix[solver.pool[r][i]+1, solver.pool[r][i+1]+1]
    end
    return cost
end

function setPartitioning(solver::Solver)
    sp = Model(CPLEX.Optimizer)
    # set_silent(sp)
    @variable(sp, λ[r = 1:length(solver.pool)], Bin)
    @objective(sp, Min, sum(c(solver, r)*λ[r] for r = 1:length(solver.pool)))
    @constraint(sp, [i = 1:length(solver.data.vertices)], sum(α(i, solver.pool[r])λ[r] for r = 1:length(solver.pool)) == 1)
    @constraint(sp, sum(λ[r] for r = 1:length(solver.pool)) <= solver.data.maxNbRoutes)
    optimize!(sp)
    if termination_status(sp) == OPTIMAL
        # return Solution([solver.pool[r] for r = 1:length(solver.pool) if value(λ[r]) >= 0.9], objective_value(sp), [0 for i = 1:solver.data.maxNbRoutes], 0)
        solver.currSol.routes = [solver.pool[r] for r = 1:length(solver.pool) if value(λ[r]) >= 0.9]
        solver.currSol.cost = objective_value(sp)
        computeLabels(solver)
        RVND!(solver)
    else
        if objective_value(sp) < solver.bestSol.cost - 0.001
            solver.currSol.routes = [solver.pool[r] for r = 1:length(solver.pool) if value(λ[r]) >= 0.9]
            solver.currSol.cost = objective_value(sp)
            computeLabels(solver)
        end
    end
end