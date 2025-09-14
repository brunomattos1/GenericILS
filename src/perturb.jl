
function perturb!(solver::Solver)
    rnd = rand(solver.seed)
    if rnd <= 1.5
        for _ = 1:solver.diversification.shift
            perturbed = randomInterShit10!(solver)
        end
    elseif rnd <= 1.
        for _ = 1:solver.diversification.swap
            perturbed = randomInterSwap11!(solver)
        end
    else
        for _ = 1:1
            perturbed = split!(solver)
        end
    end
end

function innerPerturb!(solver::Solver, solution::Solution)
    rnd = rand(solver.seed)
    if rnd <= 1.333
        for _ = 1:solver.diversification.innerShift
            perturbed = randomInterShit10!(solver, solution)
        end
    elseif rnd <= 0.666
        for _ = 1:solver.diversification.innerSwap
            perturbed = randomInterSwap11!(solver)
        end
    else
        for _ = 1:1
            perturbed = split(solver)
        end
    end
end

function outerPerturb!(solver::Solver, solution::Solution)
    rnd = rand(solver.seed)
    if rnd <= 1.333
        for _ = 1:solver.diversification.outerShift
            perturbed = randomInterShit10!(solver, solution)
        end
    elseif rnd <= 0.666
        for _ = 1:solver.diversification.outerSwap
            perturbed = randomInterSwap11!(solver)
        end
    else
        for _ = 1:1
            perturbed = split(solver)
        end
    end
end

# function randomInterShit10!(solver::Solver, sol::Solution)
#     routes = sol.routes
#     r1 = rand(solver.seed, 1:length(routes))
#     r2 = rand(solver.seed, 1:length(routes))
#     counter = 0
#     while r1 == r2 || length(routes[r1]) <= 2
#         r1 = rand(solver.seed, 1:length(routes))
#         r2 = rand(solver.seed, 1:length(routes))
#         if counter > 20
#            return false
#         end
#         counter += 1
#     end
#     i = rand(solver.seed, 2:length(routes[r1])-1)
#     j = rand(solver.seed, 2:length(routes[r2]))
#     cost, _ = evalInterShift10(sol.cost, sol, routes, solver, r1, r2, i, j)
#     applyMoveInterShift10!(solver, sol, cost, r1, r2, i, j)
#     computeLabels(solver, sol, [r1, r2])

#     return true
# end

function randomInterShit10!(solver::Solver, solution::Solution)
    routes = getRoutes(solution)
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
    # dist, cost, feasR1, feasR2, warpR1, warpR2 = evalInterShift10(solution.dist, solution, routes, solver, r1, r2, i, j)
    dist, cost, feasR1, feasR2, warpR1, warpR2 = evalInterShift10(solver, solution, Shift(r1, r2, i, j))

    infeasR1 = length(solution.routes[r1]) - 1 - feasR1 - 1
    infeasR2 = length(solution.routes[r2]) - 1 - feasR2 + 1
    # cost = objectiveValue(solver, solution, r1, r2, dist, infeasR1, infeasR2, warpR1, warpR2)
    infeas = (infeasR1, infeasR2)
    warp = (warpR1, warpR2)
    applyMoveInterShift10!(solver, solution, BestMove(cost, dist, r1, r2, i, j, infeas, warp))
    computeLabels(solver, solution, [r1, r2])
    solution.infeas[r1] = length(solution.routes[r1]) - max(solution.feasiblesF[r1], solution.feasiblesB[r1]) - 1
    solution.infeas[r2] = length(solution.routes[r2]) - max(solution.feasiblesF[r2], solution.feasiblesB[r2]) - 1
    solution.totalInfeas = sum(solution.infeas)
    return true
end

function split!(solver::Solver)
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