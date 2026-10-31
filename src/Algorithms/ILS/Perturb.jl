"""
Perturbation: with probability 1/2 apply `shift` random shifts, otherwise `swap` random swaps; then flip the visit
of a random optional vertex, if there are any.
"""
function perturb!(solver::Solver, solution::Solution)
    rnd = rand(solver.seed)
    if rnd <= 0.5
        for _ = 1:solver.algorithm.diversification.shift
            perturbed = randomInterShift10!(solver, solution)
        end
    else
        for _ = 1:solver.algorithm.diversification.swap
            perturbed = randomInterSwap11!(solver, solution)
        end
    end
    randomToggleOptional!(solver, solution)
end

"""
Position of a random customer of `visits` that can leave its route without splitting a group, or `0` if a few
draws find none.
"""
function randomMovable(solver::Solver, visits::Vector{Int})
    for _ = 1:20
        i = rand(solver.seed, 2:length(visits) - 1)
        canLeaveRoute(solver.data, visits, i, 1) && return i
    end
    return 0
end

"""Move one random customer to a random position of another route. Returns `false` if no valid pair is found."""
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
    i = randomMovable(solver, routes[r1].visits)
    i == 0 && return false
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

"""Swap one random customer of a route with one of another route. Returns `false` if no valid pair is found."""
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
    i = randomMovable(solver, routes[r1].visits)
    j = randomMovable(solver, routes[r2].visits)
    (i == 0 || j == 0) && return false
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
