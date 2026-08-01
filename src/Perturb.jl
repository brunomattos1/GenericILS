function innerPerturb!(solver::Solver, solution::Solution)
    rnd = rand(solver.seed)
    if rnd <= 0.5
        for _ = 1:solver.diversification.innerShift
            perturbed = randomInterShift10!(solver, solution)
        end
    else
        for _ = 1:solver.diversification.innerSwap
            perturbed = randomInterSwap11!(solver, solution)
        end
    end
end

function outerPerturb!(solver::Solver, solution::Solution)
    rnd = rand(solver.seed)
    if rnd <= 0.5
        for _ = 1:solver.diversification.outerShift
            perturbed = randomInterShift10!(solver, solution)
        end
    else
        for _ = 1:solver.diversification.outerSwap
            perturbed = randomInterSwap11!(solver, solution)
        end
    end
end

function randomInterShift10!(solver::Solver, solution::Solution)
    routes = solution.routes
    r1 = rand(solver.seed, 1:length(routes))
    r2 = rand(solver.seed, 1:length(routes))
    counter = 0
    while r1 == r2 || length(routes[r1].visits) <= 2
        r1 = rand(solver.seed, 1:length(routes))
        r2 = rand(solver.seed, 1:length(routes))
        if counter > 20
           return false
        end
        counter += 1
    end
    i = rand(solver.seed, 2:length(routes[r1].visits)-1)
    j = rand(solver.seed, 2:length(routes[r2].visits))
    k = 1
    dist = interShiftCost(solution.dist, solver.data.costMatrix, routes[r1].visits, routes[r2].visits, i, j, k)
    block = routes[r1].visits[i:i]
    violInfo, warpR1s1, warpR2s1, warpR1s2, warpR2s2 =
        computeViolInterShiftK(solver, solution, r1, r2, i, j, k, block)
    cost = objectiveValue(solver, solution,
        Cost(dist, r1, r2, violInfo, (warpR1s1, warpR2s1), (warpR1s2, warpR2s2)))
    move = BestMove(cost, dist, r1, r2, i, j)
    apply!(InterShift{1}(), solver, solution, move)
    return true
end

function randomInterSwap11!(solver::Solver, solution::Solution)
    routes = solution.routes
    r1 = rand(solver.seed, 1:length(routes))
    r2 = rand(solver.seed, 1:length(routes))
    counter = 0
    while r1 == r2 || length(routes[r1].visits) <= 2 || length(routes[r2].visits) <= 2
        r1 = rand(solver.seed, 1:length(routes))
        r2 = rand(solver.seed, 1:length(routes))
        if counter > 20
            return false
        end
        counter += 1
    end
    i = rand(solver.seed, 2:length(routes[r1].visits) - 1)
    j = rand(solver.seed, 2:length(routes[r2].visits) - 1)
    k1 = 1
    k2 = 1
    dist = interSwapCost(solution.dist, solver.data.costMatrix, routes[r1].visits, routes[r2].visits, i, j, k1, k2)
    violInfo, warpR1s1, warpR2s1, warpR1s2, warpR2s2 =
        computeViolInterSwapK(solver, solution, r1, r2, i, j, k1, k2)
    cost = objectiveValue(solver, solution,
        Cost(dist, r1, r2, violInfo, (warpR1s1, warpR2s1), (warpR1s2, warpR2s2)))
    move = BestMove(cost, dist, r1, r2, i, j)
    apply!(InterSwap{1,1}(), solver, solution, move)
    return true
end