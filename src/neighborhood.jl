
function intraShift10!(solver::Solver, sol::Solution)
    improved = false
    copy_solution!(solver.bufferSol, sol)
    oldSol = solver.bufferSol
    for r = 1:length(sol.routes)
        lastEval = sol.lastEval[1, r, r]

        if lastEval >= max(sol.lastModif[r], sol.lastModif[r])
            #continue  # pular pares que não mudaram
        end
        bestMove = BestMove(cost = sol.cost, dist = sol.dist)
        if (sol.feasiblesF[r] >= length(sol.routes[r]) - 1)
            for i = 2:length(sol.routes[r])-1
                for j = i+1:length(sol.routes[r])-1
                    dist, cost, resViol, warpR1Std1, warpR2Std1, warpR1Std2, warpR2Std2 = evalIntraShift10(solver, sol, Shift(r, r, i, j))

                    if resViol == 0 && (cost < bestMove.cost - 1e-6)
                        bestMove = BestMove(cost, dist, r, 0, i, j, (0, 0), (warpR1Std1, warpR1Std2), (warpR1Std2, warpR2Std2))
                    end
                end
            end
        end
        if (sol.feasiblesB[r] >= length(sol.routes[r]) - 1)
            for i = length(sol.routes[r])-1:-1:2
                for j = i-1:-1:2
                    # if j == i - 1
                    #     solver.prevLabelB = myExtendAlongArc(solver.res, sol.backwardLabels[r][length(sol.routes[r]) - i], (sol.routes[r][i+1]+1, sol.routes[r][j] + 1))
                    # end
                    # dist, resViol, warp, cost = evalIntraShift10(sol.dist, sol, sol.routes, solver, r, i, j)
                    dist, cost, resViol, warpR1Std1, warpR2Std1, warpR1Std2, warpR2Std2 = evalIntraShift10(solver, sol, Shift(r, r, i, j))

                    if resViol == 0 && (cost < bestMove.cost - 1e-6)
                        bestMove = BestMove(cost, dist, r, 0, i, j, (0, 0), (warpR1Std1, warpR1Std2), (warpR1Std2, warpR2Std2))
                    end
                end
            end
        end
        if bestMove.firstIdx > 0
            prevCost = oldSol.cost
            applyMoveIntraShift10!(solver, sol, bestMove)
            if sol.cost < prevCost - 1e-6
                improved = true
                copy_solution!(oldSol, sol)
            else
                copy_solution!(sol, oldSol)
            end
        end
    end
    return improved
end

function interShift10!(solver::Solver, sol::Solution)
    improved = false
    copy_solution!(solver.bufferSol, sol)
    oldSol = solver.bufferSol
    resize!(solver.buffer, length(sol.routes))

    copyto!(solver.buffer, 1:length(sol.routes))

    shuffle!(solver.buffer)

    routesIdx = solver.buffer   
    for r1 in routesIdx
        bestMove = BestMove(dist = sol.dist, cost = sol.cost)
        for r2 in routesIdx
            if r1 == r2
                continue
            end
            lastEval = sol.lastEval[2, r1, r2]
            if lastEval > max(sol.lastModif[r1], sol.lastModif[r2])
                continue  # pular pares que não mudaram
            end
            for i = 2:length(sol.routes[r1]) - 1
                for j = 2:length(sol.routes[r2])
                    # dist, cost, feasR1, feasR2, warpR1, warpR2 = evalInterShift10(sol.dist, sol, routes, solver, r1, r2, i, j)
                    dist, cost, infeasR1, infeasR2, warpR1Std1, warpR1Std2, warpR2Std1, warpR2Std2 = evalInterShift10(solver, sol, Shift(r1, r2, i, j))
                    if cost < bestMove.cost - 1e-6
                        bestMove = BestMove(cost, dist, r1, r2, i, j, (infeasR1, infeasR2), (warpR1Std1, warpR1Std2), (warpR2Std1, warpR2Std2))
                    end
                end
            end
        end
        if bestMove.firstIdx > 0
            prevCost = oldSol.cost
            applyMoveInterShift10!(solver, sol, bestMove)
            if sol.cost < prevCost - 1e-6
                improved = true
                copy_solution!(oldSol, sol)
            else
                copy_solution!(sol, oldSol)
            end
        end
    end
    return improved
end

function interShift20!(solver::Solver, sol::Solution)
    improved = false
    copy_solution!(solver.bufferSol, sol)
    oldSol = solver.bufferSol
    resize!(solver.buffer, length(sol.routes))
    copyto!(solver.buffer, 1:length(sol.routes))
    shuffle!(solver.buffer)

    routesIdx = solver.buffer
    for r1 in routesIdx
        bestMove = BestMove(dist = sol.dist, cost = sol.cost)
        if length(sol.routes[r1]) <= 3
            continue
        end
        for r2 in routesIdx
            if r1 == r2
                continue
            end
            lastEval = sol.lastEval[5, r1, r2]
            if lastEval > max(sol.lastModif[r1], sol.lastModif[r2])
                continue
            end
            for i = 2:length(sol.routes[r1]) - 2
                for j = 2:length(sol.routes[r2])
                    dist, cost, infeasR1, infeasR2, warpR1Std1, warpR1Std2, warpR2Std1, warpR2Std2 =
                        evalInterShift20(solver, sol, Shift(r1, r2, i, j))
                    if cost < bestMove.cost - 1e-6
                        bestMove = BestMove(cost, dist, r1, r2, i, j,
                            (infeasR1, infeasR2),
                            (warpR1Std1, warpR1Std2),
                            (warpR2Std1, warpR2Std2))
                    end
                end
            end
        end
        if bestMove.firstIdx > 0
            prevCost = oldSol.cost
            applyMoveInterShift20!(solver, sol, bestMove)
            if sol.cost < prevCost - 1e-6
                improved = true
                copy_solution!(oldSol, sol)
            else
                copy_solution!(sol, oldSol)
            end
        end
    end
    return improved
end

function interSwap11!(solver::Solver, sol::Solution)
    improved = false
    copy_solution!(solver.bufferSol, sol)
    oldSol = solver.bufferSol
    resize!(solver.buffer, length(sol.routes))

    copyto!(solver.buffer, 1:length(sol.routes))

    shuffle!(solver.buffer)

    routesIdx = solver.buffer   
    for r1 in routesIdx
        bestMove = BestMove(dist = sol.dist, cost = sol.cost)
        if length(sol.routes[r1]) <= 2
            continue
        end
        for r2 in routesIdx
            if r1 >= r2
                continue
            end
            if length(sol.routes[r2]) <= 2
                continue
            end
            # key = (:interSwap, r1, r2)
            # lastEval = get(sol.lastEval, key, -1)
            lastEval = sol.lastEval[3, r1, r2]

            if lastEval >= max(sol.lastModif[r1], sol.lastModif[r2])
                continue  # pular pares que não mudaram
            end
            for i = 2:length(sol.routes[r1]) - 1
                for j = 2:length(sol.routes[r2]) - 1
                    dist, cost, infeasR1, infeasR2, warpR1Std1, warpR1Std2, warpR2Std1, warpR2Std2 = evalInterSwap11(solver, sol, Swap(r1, r2, i, j))
                    if cost < bestMove.cost - 1e-6
                        bestMove = BestMove(cost, dist, r1, r2, i, j, (infeasR1, infeasR2), (warpR1Std1, warpR1Std2), (warpR2Std1, warpR2Std2))
                    end
                end
            end
        end
        if bestMove.firstIdx > 0
            prevCost = oldSol.cost
            applyMoveInterSwap11!(solver, sol, bestMove)
            if sol.cost < prevCost - 1e-6
                improved = true
                copy_solution!(oldSol, sol)
            else
                copy_solution!(sol, oldSol)
            end
        end
    end
    return improved
end

function twoOptStar!(solver::Solver, sol::Solution)
    improved = false
    copy_solution!(solver.bufferSol, sol)
    oldSol = solver.bufferSol
    resize!(solver.buffer, length(sol.routes))

    copyto!(solver.buffer, 1:length(sol.routes))

    shuffle!(solver.buffer)

    routesIdx = solver.buffer   
    for r1 in routesIdx
        bestMove = BestMove(cost = sol.cost, dist = sol.dist)
        for r2 in routesIdx
            if r1 == r2
                continue
            end
            # key = (:twoOptStar, r1, r2)
            # lastEval = get(sol.lastEval, key, -1)
            lastEval = sol.lastEval[4, r1, r2]

            if lastEval >= max(sol.lastModif[r1], sol.lastModif[r2])
                continue  # pular pares que não mudaram
            end
            for i = 1:length(sol.routes[r1]) - 2
                for j = 1:length(sol.routes[r2]) - 2
                    dist, cost, infeasR1, infeasR2, warpR1Std1, warpR1Std2, warpR2Std1, warpR2Std2 = evalTwoOptStar!(solver, sol, OptStar(r1, r2, i, j))
                    if cost < bestMove.cost - 1e-6
                        bestMove = BestMove(cost, dist, r1, r2, i, j, (infeasR1, infeasR2), (warpR1Std1, warpR1Std2), (warpR2Std1, warpR2Std2))
                    end
                end
            end
        end
        if bestMove.firstIdx > 0
            prevCost = oldSol.cost
            applyMoveTwoOptStar!(solver, sol, bestMove)
            if sol.cost < prevCost - 1e-6
                improved = true
                copy_solution!(oldSol, sol)
            else
                copy_solution!(sol, oldSol)
            end
        end
    end
    return improved
end