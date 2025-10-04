
function computeLabels(solver::Solver, sol::Solution)
    # sol = solver.currSol
    nbRoutes = length(sol.routes)
    forwLabels = Vector{Vector{ForwardLabel}}(undef, nbRoutes)
    backwLabels = Vector{Vector{BackwardLabel}}(undef, nbRoutes)
    sol.lastFeasibleF = Vector{Int}()
    sol.lastFeasibleB = Vector{Int}()
    sol.feasiblesF = Vector{Int}()
    sol.feasiblesB = Vector{Int}()
    for r = 1:nbRoutes
        push!(sol.feasiblesF, -1)
        push!(sol.feasiblesB, -1)
        forwLabels[r] = Vector{ForwardLabel}(undef, length(sol.routes[r]))
        backwLabels[r] = Vector{BackwardLabel}(undef, length(sol.routes[r]))
        lenR = length(sol.routes[r])
        forwLabels[r][1] = myInitStateForward(solver.res.customResource)
        backwLabels[r][lenR] = myInitStateBackward(solver.res.customResource)
    end
    for r = 1:nbRoutes
        lenR = length(sol.routes[r])
        last = false
        for i = 2:lenR
            label = myExtendAlongArc(solver.res, forwLabels[r][i-1], (sol.routes[r][i-1]+1, sol.routes[r][i]+1))
            if (!last && label.cost == Inf)# || (!last && length(sol.routes[r]) == 3)
                last = true
                push!(sol.lastFeasibleF, i - 1)
                sol.feasiblesF[r] = i - 2
            end
            forwLabels[r][i] = label
        end
        if !last
            push!(sol.lastFeasibleF, lenR)
            sol.feasiblesF[r] = lenR - 1
        end
    end
    for r = 1:nbRoutes
        lenR = length(sol.routes[r])
        last = false
        for i = lenR:-1:2
            label = myExtendAlongArc(solver.res, backwLabels[r][i], (sol.routes[r][i]+1, sol.routes[r][i-1]+1))
            if (!last && label.cost == Inf)# || length(sol.routes[r]) == 3
                last = true
                push!(sol.lastFeasibleB, lenR - i + 1)# +1)
                # push!(sol.lastFeasibleB, i)
                sol.feasiblesB[r] = lenR - i# +1
            end
            backwLabels[r][i-1] = label
        end
        reverse!(backwLabels[r])
        if !last
            push!(sol.lastFeasibleB, lenR)
            sol.feasiblesB[r] = lenR - 1
        end
    end
    sol.forwardLabels = forwLabels
    sol.backwardLabels = backwLabels
end

function computeLabels(solver::Solver, sol::Solution, routes::Int...)
    for r in routes
        lenR = length(sol.routes[r])

        # Pré-alocação e redimensionamento dos vetores de labels
        if length(sol.forwardLabels[r]) != lenR
            resize!(sol.forwardLabels[r], lenR)
            resize!(sol.backwardLabels[r], lenR)
        end
        # O restante do seu código permanece, mas agora opera em vetores pré-alocados.
        sol.lastFeasibleF[r] = -1
        sol.lastFeasibleB[r] = -1
        sol.feasiblesF[r] = -1
        sol.feasiblesB[r] = -1
        sol.forwardLabels[r][1] = myInitStateForward(solver.res.customResource)
        sol.backwardLabels[r][lenR] = myInitStateBackward(solver.res.customResource)
    end
    for r in routes
        lenR = length(sol.routes[r])
        last = false
        for i = 2:lenR
            label = myExtendAlongArc(solver.res, sol.forwardLabels[r][i-1], (sol.routes[r][i-1]+1, sol.routes[r][i]+1))
            if (!last && label.cost == Inf)# || (!last && length(sol.routes[r]) == 3)
                last = true
                sol.lastFeasibleF[r] = i - 1
                sol.feasiblesF[r] = i - 2
            end
            sol.forwardLabels[r][i] = label
        end
        if !last
            sol.lastFeasibleF[r] = lenR
            sol.feasiblesF[r] = lenR - 1#2
        end

    end
    for r in routes
        lenR = length(sol.routes[r])
        last = false
        for i = lenR:-1:2
            label = myExtendAlongArc(solver.res, sol.backwardLabels[r][i], (sol.routes[r][i]+1, sol.routes[r][i-1]+1))
            if (!last && label.cost == Inf)# || length(sol.routes[r]) == 3
                last = true
                sol.lastFeasibleB[r] = lenR - i + 1# +1
                sol.feasiblesB[r] = lenR - i# +1
            end
            sol.backwardLabels[r][i-1] = label
        end
        reverse!(sol.backwardLabels[r])
        if !last
            sol.lastFeasibleB[r] = lenR
            sol.feasiblesB[r] = lenR - 1#2
        end
    end
end



