
function computeLabels(solver::Solver{N, AC, SC, R, FL, BL}, sol::Solution{FL, BL}) where {N, AC, SC, R <: AbstractResources, FL, BL}
    nbRoutes = length(sol.routes)
    forwLabels  = Vector{Vector{FL}}(undef, nbRoutes)
    backwLabels = Vector{Vector{BL}}(undef, nbRoutes)
    sol.lastFeasibleF = Vector{Int}()
    sol.lastFeasibleB = Vector{Int}()
    sol.feasiblesF = Vector{Int}()
    sol.feasiblesB = Vector{Int}()
    for r = 1:nbRoutes
        push!(sol.feasiblesF, -1)
        push!(sol.feasiblesB, -1)
        forwLabels[r]  = Vector{FL}(undef, length(sol.routes[r]))
        backwLabels[r] = Vector{BL}(undef, length(sol.routes[r]))
        lenR = length(sol.routes[r])
        forwLabels[r][1] = myInitStateForward(solver.res.customResource)
        backwLabels[r][1] = myInitStateBackward(solver.res.customResource)
    end
    for r = 1:nbRoutes
        lenR = length(sol.routes[r])
        last = false
        for i = 2:lenR
            label = myExtendAlongArc(solver.res, forwLabels[r][i-1], (sol.routes[r][i-1]+1, sol.routes[r][i]+1))
            if !last && label.cost == Inf
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
        for k = 2:lenR
            pos = lenR - k + 1
            label = myExtendAlongArc(solver.res, backwLabels[r][k-1], (sol.routes[r][pos+1]+1, sol.routes[r][pos]+1))
            if !last && label.cost == Inf
                last = true
                push!(sol.lastFeasibleB, k - 1)
                sol.feasiblesB[r] = k - 2
            end
            backwLabels[r][k] = label
        end
        if !last
            push!(sol.lastFeasibleB, lenR)
            sol.feasiblesB[r] = lenR - 1
        end
    end
    sol.forwardLabels  = forwLabels
    sol.backwardLabels = backwLabels
end

function computeLabels(solver::Solver{N, AC, SC, R, FL, BL}, sol::Solution{FL, BL}, routes::Int...) where {N, AC, SC, R <: AbstractResources, FL, BL}
    for r in routes
        lenR = length(sol.routes[r])

        if length(sol.forwardLabels[r]) != lenR
            resize!(sol.forwardLabels[r], lenR)
            resize!(sol.backwardLabels[r], lenR)
        end
        sol.lastFeasibleF[r] = -1
        sol.lastFeasibleB[r] = -1
        sol.feasiblesF[r] = -1
        sol.feasiblesB[r] = -1
        sol.forwardLabels[r][1] = myInitStateForward(solver.res.customResource)
        sol.backwardLabels[r][1] = myInitStateBackward(solver.res.customResource)
    end
    for r in routes
        lenR = length(sol.routes[r])
        last = false
        for i = 2:lenR
            label = myExtendAlongArc(solver.res, sol.forwardLabels[r][i-1], (sol.routes[r][i-1]+1, sol.routes[r][i]+1))
            if !last && label.cost == Inf
                last = true
                sol.lastFeasibleF[r] = i - 1
                sol.feasiblesF[r] = i - 2
            end
            sol.forwardLabels[r][i] = label
        end
        if !last
            sol.lastFeasibleF[r] = lenR
            sol.feasiblesF[r] = lenR - 1
        end
    end
    for r in routes
        lenR = length(sol.routes[r])
        last = false
        for k = 2:lenR
            pos = lenR - k + 1
            label = myExtendAlongArc(solver.res, sol.backwardLabels[r][k-1], (sol.routes[r][pos+1]+1, sol.routes[r][pos]+1))
            if !last && label.cost == Inf
                last = true
                sol.lastFeasibleB[r] = k - 1
                sol.feasiblesB[r] = k - 2
            end
            sol.backwardLabels[r][k] = label
        end
        if !last
            sol.lastFeasibleB[r] = lenR
            sol.feasiblesB[r] = lenR - 1
        end
    end
end

function computeRouteLabelCost(solver::Solver, route::Vector{Int})
    label = myInitStateForward(solver.res.customResource)
    for i = 2:length(route)
        label = myExtendAlongArc(solver.res, label, (route[i-1]+1, route[i]+1))
    end
    return label.cost
end

function computeLabels(solver::Solver{N, AC, SC, R, FL, BL}, sol::UserSolution{FL, BL}) where {N, AC, SC, R <: AbstractResources, FL, BL}
    nbRoutes = length(sol.routes)
    forwLabels  = Vector{Vector{FL}}(undef, nbRoutes)
    backwLabels = Vector{Vector{BL}}(undef, nbRoutes)
    for r = 1:nbRoutes
        forwLabels[r]  = Vector{FL}(undef, length(sol.routes[r]))
        backwLabels[r] = Vector{BL}(undef, length(sol.routes[r]))
        lenR = length(sol.routes[r])
        forwLabels[r][1] = myInitStateForward(solver.res.customResource)
        backwLabels[r][1] = myInitStateBackward(solver.res.customResource)
    end
    for r = 1:nbRoutes
        lenR = length(sol.routes[r])
        for i = 2:lenR
            forwLabels[r][i] = myExtendAlongArc(solver.res, forwLabels[r][i-1], (sol.routes[r][i-1]+1, sol.routes[r][i]+1))
        end
    end
    for r = 1:nbRoutes
        lenR = length(sol.routes[r])
        for k = 2:lenR
            pos = lenR - k + 1
            backwLabels[r][k] = myExtendAlongArc(solver.res, backwLabels[r][k-1], (sol.routes[r][pos+1]+1, sol.routes[r][pos]+1))
        end
    end
    sol.forwardLabels  = forwLabels
    sol.backwardLabels = backwLabels
end
