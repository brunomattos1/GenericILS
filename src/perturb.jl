
function perturb!(solver::Solver)
    rnd = rand(solver.seed)
    if rnd <= 0.4
        for _ = 1:solver.diversification.shift
            perturbed = randomInterShit10!(solver)
        end
    elseif rnd <= 0.8
        for _ = 1:solver.diversification.swap
            perturbed = randomInterSwap11!(solver)
        end
    else
        for _ = 1:solver.diversification.swap
            perturbed = split(solver)
        end
    end
end

function randomInterShit10!(solver::Solver)
    routes = getRoutes(getCurrSol(solver))
    r1 = rand(solver.seed, 1:length(routes))
    r2 = rand(solver.seed, 1:length(routes))
    counter = 0
    while r1 == r2 || length(routes[r1]) <= 2
        r1 = rand(solver.seed, 1:length(routes))
        r2 = rand(solver.seed, 1:length(routes))
        if counter > 20
           return false
        end
        counter += 1
    end
    i = rand(solver.seed, 2:length(routes[r1])-1)
    j = rand(solver.seed, 2:length(routes[r2]))
    cost, _ = evalInterShift10(solver.currSol.cost, routes, solver, r1, r2, i, j)
    # totalViol = solver.currSol.totalViolation - solver.currSol.resViolation[r1] - solver.currSol.resViolation[r2] + resViolR1 + resViolR2
    applyMoveInterShift10!(solver, cost, r1, r2, i, j)
    computeLabels(solver, [r1, r2])

    return true
end

function split(solver::Solver)
    routes = getRoutes(getCurrSol(solver))
    r = rand(solver.seed, 1:length(routes))
    counter = 0
    while length(routes[r]) <= 3
        r = rand(solver.seed, 1:length(routes))
        if counter > 20
           return false
        end
        counter += 1
    end
    i = rand(solver.seed, 2:length(routes[r])-1)
    cost = evalSplit!(solver.currSol.cost, routes, solver, r, i)
    applyMoveSplit!(solver, cost, r, i)
    computeLabels(solver)
    return true
end

function randomInterSwap11!(solver::Solver)
    routes = getRoutes(getCurrSol(solver))
    r1 = rand(solver.seed, 1:length(routes))
    r2 = rand(solver.seed, 1:length(routes))
    counter = 0
    while r1 == r2 || length(routes[r1]) <= 2 || length(routes[r2]) <= 2
        r1 = rand(solver.seed, 1:length(routes))
        r2 = rand(solver.seed, 1:length(routes))
        if counter > 20
            return false
        end
        counter += 1
    end
    i = rand(solver.seed, 2:length(routes[r1]) - 1)
    j = rand(solver.seed, 2:length(routes[r2]) - 1)
    
    cost, _ = evalInterSwap11(solver.currSol.cost, routes, solver, r1, r2, i, j)
    # totalViol = solver.currSol.totalViolation - solver.currSol.resViolation[r1] - solver.currSol.resViolation[r2] + resViolR1 + resViolR2
    applyMoveInterSwap11!(solver, cost, r1, r2, i, j)
    computeLabels(solver, [r1, r2])
    return true
end

function randomIntraShift10!(solver::Solver)
    routes = getRoutes(getCurrSol(solver))
    r = rand(solver.seed, 1:length(routes))
    counter = 0
    while length(routes[r]) <= 2
        r = rand(solver.seed, 1:length(routes))
        if counter > 20
            return 0 
        end
        counter += 1
    end
    i = rand(solver.seed, 2:length(routes[r])-1)
    j = rand(solver.seed, 2:length(routes[r]))
    while i == j || i == j+1 || j == i+1
        i = rand(solver.seed, 2:length(routes[r])-1)
        j = rand(solver.seed, 2:length(routes[r]))
        if counter > 20
            return 0 
        end
        counter += 1
    end
    cost, resViolR = evalIntraShift10(solver.currSol.cost, routes, solver, r, i, j)
    totalViol = solver.currSol.totalViolation - solver.currSol.resViolation[r] + resViolR
    applyMoveIntraShift10!(solver, cost, totalViol, resViolR, r, i, j)
end