
function computeLabels(solver::Solver)
    sol = solver.currSol
    nbRoutes = length(sol.routes)
    forwLabels = Vector{Vector{CapacityState}}(undef, nbRoutes)
    backwLabels = Vector{Vector{CapacityState}}(undef, nbRoutes)
    sol.lastFeasibleF = Vector{Int}()
    sol.lastFeasibleB = Vector{Int}()
    sol.feasiblesF = Vector{Int}()
    sol.feasiblesB = Vector{Int}()
    for r = 1:nbRoutes
        push!(sol.feasiblesF, -1)
        push!(sol.feasiblesB, -1)
        forwLabels[r] = Vector{CapacityState}(undef, length(sol.routes[r]))
        backwLabels[r] = Vector{CapacityState}(undef, length(sol.routes[r]))
        lenR = length(sol.routes[r])
        forwLabels[r][1] = solver.initState()
        backwLabels[r][lenR] = solver.initState()
    end
    for r = 1:nbRoutes
        lenR = length(sol.routes[r])
        last = false
        for i = 2:lenR
            label = solver.extendAlongArc(solver.res, copy(forwLabels[r][i-1]), (sol.routes[r][i-1]+1, sol.routes[r][i]+1))
            if (!last && label.cost == Inf)# || (!last && length(sol.routes[r]) == 3)
                last = true
                push!(sol.lastFeasibleF, i - 1)
                sol.feasiblesF[r] = i - 2
            end
            forwLabels[r][i] = label
        end
        if !last
            push!(sol.lastFeasibleF, lenR)
            sol.feasiblesF[r] = lenR - 2
        end
    end
    for r = 1:nbRoutes
        lenR = length(sol.routes[r])
        last = false
        for i = lenR:-1:2
            label = extendAlongArc(solver.res, copy(backwLabels[r][i]), (sol.routes[r][i]+1, sol.routes[r][i-1]+1))
            if (!last && label.cost == Inf)# || length(sol.routes[r]) == 3
                last = true
                push!(sol.lastFeasibleB, lenR - i + 1)
                # sol.lastFeasibleB[r] = lenR - i + 1
                sol.feasiblesB[r] = lenR - i
            end
            backwLabels[r][i-1] = label
        end
        reverse!(backwLabels[r])
        if !last
            push!(sol.lastFeasibleB, lenR)
            sol.feasiblesB[r] = lenR - 2
        end
    end
    solver.forwardLabels = forwLabels
    solver.backwardLabels = backwLabels
end

function computeLabels(solver::Solver, routes::Vector{Int})
    sol = solver.currSol
    # nbRoutes = length(sol.routes)

    for r in routes
        sol.lastFeasibleF[r] = -1
        sol.lastFeasibleB[r] = -1
        sol.feasiblesF[r] = -1
        sol.feasiblesB[r] = -1
        lenR = length(sol.routes[r])
        solver.forwardLabels[r] = Vector{CapacityState}(undef, lenR)
        solver.backwardLabels[r] = Vector{CapacityState}(undef, lenR)
        solver.forwardLabels[r][1] = solver.initState()
        solver.backwardLabels[r][lenR] = solver.initState()
    end
    for r in routes
        lenR = length(sol.routes[r])
        last = false
        for i = 2:lenR
            label = solver.extendAlongArc(solver.res, copy(solver.forwardLabels[r][i-1]), (sol.routes[r][i-1]+1, sol.routes[r][i]+1))
            if (!last && label.cost == Inf)# || (!last && length(sol.routes[r]) == 3)
                last = true
                sol.lastFeasibleF[r] = i - 1
                sol.feasiblesF[r] = i - 2
            end
            solver.forwardLabels[r][i] = label
        end
        if !last
            sol.lastFeasibleF[r] = lenR - 1
            sol.feasiblesF[r] = lenR - 2
        end

    end
    for r in routes
        lenR = length(sol.routes[r])
        last = false
        for i = lenR:-1:2
            label = extendAlongArc(solver.res, copy(solver.backwardLabels[r][i]), (sol.routes[r][i]+1, sol.routes[r][i-1]+1))
            if (!last && label.cost == Inf)# || length(sol.routes[r]) == 3
                last = true
                sol.lastFeasibleB[r] = lenR - i + 1
                sol.feasiblesB[r] = lenR - i
            end
            solver.backwardLabels[r][i-1] = label
        end
        reverse!(solver.backwardLabels[r])
        if !last
            sol.lastFeasibleB[r] = lenR - 1
            sol.feasiblesB[r] = lenR - 2
        end
        # solver.backwardLabels[r] = backwLabels
    end
end



