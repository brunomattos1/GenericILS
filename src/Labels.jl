
function computeLabels(solver::Solver{N, AC, SC, R, FL, BL}, sol::Solution{FL, BL}) where {N, AC, SC, R <: AbstractResources, FL, BL}
    for r = 1:length(sol.routes)
        rt = sol.routes[r]
        lenR = length(rt.visits)
        resize!(rt.forwardLabels, lenR)
        resize!(rt.backwardLabels, lenR)
        rt.feasibleF     = -1
        rt.feasibleB     = -1
        rt.lastFeasibleF = -1
        rt.lastFeasibleB = -1
        rt.forwardLabels[1]  = myInitStateForward(solver.res.customResource)
        rt.backwardLabels[1] = myInitStateBackward(solver.res.customResource)
    end
    for r = 1:length(sol.routes)
        rt = sol.routes[r]
        lenR = length(rt.visits)
        last = false
        for i = 2:lenR
            label = myExtendAlongArc(solver.res, rt.forwardLabels[i-1], (rt.visits[i-1]+1, rt.visits[i]+1))
            if !last && label.cost == Inf
                last = true
                rt.lastFeasibleF = i - 1
                rt.feasibleF     = i - 2
            end
            rt.forwardLabels[i] = label
        end
        if !last
            rt.lastFeasibleF = lenR
            rt.feasibleF     = lenR - 1
        end
    end
    for r = 1:length(sol.routes)
        rt = sol.routes[r]
        lenR = length(rt.visits)
        last = false
        for k = 2:lenR
            pos = lenR - k + 1
            label = myExtendAlongArc(solver.res, rt.backwardLabels[k-1], (rt.visits[pos+1]+1, rt.visits[pos]+1))
            if !last && label.cost == Inf
                last = true
                rt.lastFeasibleB = k - 1
                rt.feasibleB     = k - 2
            end
            rt.backwardLabels[k] = label
        end
        if !last
            rt.lastFeasibleB = lenR
            rt.feasibleB     = lenR - 1
        end
    end
end

function computeLabels(solver::Solver{N, AC, SC, R, FL, BL}, sol::Solution{FL, BL}, routes::Int...) where {N, AC, SC, R <: AbstractResources, FL, BL}
    for r in routes
        rt = sol.routes[r]
        lenR = length(rt.visits)
        if length(rt.forwardLabels) != lenR
            resize!(rt.forwardLabels, lenR)
            resize!(rt.backwardLabels, lenR)
        end
        rt.lastFeasibleF = -1
        rt.lastFeasibleB = -1
        rt.feasibleF     = -1
        rt.feasibleB     = -1
        rt.forwardLabels[1]  = myInitStateForward(solver.res.customResource)
        rt.backwardLabels[1] = myInitStateBackward(solver.res.customResource)
    end
    for r in routes
        rt = sol.routes[r]
        lenR = length(rt.visits)
        last = false
        for i = 2:lenR
            label = myExtendAlongArc(solver.res, rt.forwardLabels[i-1], (rt.visits[i-1]+1, rt.visits[i]+1))
            if !last && label.cost == Inf
                last = true
                rt.lastFeasibleF = i - 1
                rt.feasibleF     = i - 2
            end
            rt.forwardLabels[i] = label
        end
        if !last
            rt.lastFeasibleF = lenR
            rt.feasibleF     = lenR - 1
        end
    end
    for r in routes
        rt = sol.routes[r]
        lenR = length(rt.visits)
        last = false
        for k = 2:lenR
            pos = lenR - k + 1
            label = myExtendAlongArc(solver.res, rt.backwardLabels[k-1], (rt.visits[pos+1]+1, rt.visits[pos]+1))
            if !last && label.cost == Inf
                last = true
                rt.lastFeasibleB = k - 1
                rt.feasibleB     = k - 2
            end
            rt.backwardLabels[k] = label
        end
        if !last
            rt.lastFeasibleB = lenR
            rt.feasibleB     = lenR - 1
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
    for r = 1:length(sol.routes)
        rt = sol.routes[r]
        lenR = length(rt.visits)
        resize!(rt.forwardLabels, lenR)
        resize!(rt.backwardLabels, lenR)
        rt.forwardLabels[1]  = myInitStateForward(solver.res.customResource)
        rt.backwardLabels[1] = myInitStateBackward(solver.res.customResource)
    end
    for r = 1:length(sol.routes)
        rt = sol.routes[r]
        lenR = length(rt.visits)
        for i = 2:lenR
            rt.forwardLabels[i] = myExtendAlongArc(solver.res, rt.forwardLabels[i-1], (rt.visits[i-1]+1, rt.visits[i]+1))
        end
    end
    for r = 1:length(sol.routes)
        rt = sol.routes[r]
        lenR = length(rt.visits)
        for k = 2:lenR
            pos = lenR - k + 1
            rt.backwardLabels[k] = myExtendAlongArc(solver.res, rt.backwardLabels[k-1], (rt.visits[pos+1]+1, rt.visits[pos]+1))
        end
    end
end
