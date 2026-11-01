
"""
Compute forward and backward labels of every route of `sol` and set the feasibility markers
(`feasibleF/B`, `lastFeasibleF/B`).
"""
function computeLabels(solver::Solver{N, R, PM, FL, BL, AL}, sol::Solution{FL, BL}) where {N, R <: AbstractResources, PM, FL, BL, AL}
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

"""
Set the label cost of every route of `sol`, and its total, from the labels that `computeLabels` just computed. Needed
after routes are replaced wholesale (as the Set Partitioning does), because the moves keep these values up to date
themselves.
"""
function refreshLabelCosts!(sol::Solution)
    sol.totalLabelCost = 0.0
    for rt in sol.routes
        rt.labelCost = min(rt.forwardLabels[end].cost, rt.backwardLabels[end].cost)
        sol.totalLabelCost += rt.labelCost
    end
    return sol
end

"""Same as above, restricted to the routes whose indices are listed (used after a move)."""
function computeLabels(solver::Solver{N, R, PM, FL, BL, AL}, sol::Solution{FL, BL}, routes::Int...) where {N, R <: AbstractResources, PM, FL, BL, AL}
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

"""Custom-resource cost of a visit sequence (forward label cost at the end of the route)."""
function computeRouteLabelCost(solver::Solver, route::Vector{Int})
    label = myInitStateForward(solver.res.customResource)
    for i = 2:length(route)
        label = myExtendAlongArc(solver.res, label, (route[i-1]+1, route[i]+1))
    end
    return label.cost
end

"""Compute the labels of a `UserSolution` (no feasibility markers are kept)."""
function computeLabels(solver::Solver{N, R, PM, FL, BL, AL}, sol::UserSolution{FL, BL}) where {N, R <: AbstractResources, PM, FL, BL, AL}
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
