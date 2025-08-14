function applyMoveInsertion(solver::Solver, newCost::Float64, r::Int, customer::Int, j::Int)
    solver.currSol.cost = newCost
    # solver.currSol.resViolation[r] = newResViol
    # solver.currSol.totalViolation = newTotalViol
    insert!(solver.currSol.routes[r], j, customer)
end

function applyMoveIntraShift10!(solver::Solver, newCost::Float64, r::Int, i::Int, j::Int)
    solver.currSol.cost = newCost
    customerI = solver.currSol.routes[r][i]
    if i < j
        if j == i + 1
            deleteat!(solver.currSol.routes[r], i)
            insert!(solver.currSol.routes[r], j, customerI)
        else
            deleteat!(solver.currSol.routes[r], i)
            insert!(solver.currSol.routes[r], j-1, customerI)
        end
        # deleteat!(solver.currSol.routes[r], i)
        # insert!(solver.currSol.routes[r], j-1, customerI)
    else
        deleteat!(solver.currSol.routes[r], i)
        insert!(solver.currSol.routes[r], j, customerI)
    end
end

function applyMoveIntraShift20!(solver::Solver, newCost::Float64, r::Int, i::Int, j::Int)
    solver.currSol.cost = newCost
    customerI1 = solver.currSol.routes[r][i]
    customerI2 = solver.currSol.routes[r][i+1]
    if i < j
        deleteat!(solver.currSol.routes[r], [i, i+1])
        insert!(solver.currSol.routes[r], j-2, customerI2)
        insert!(solver.currSol.routes[r], j-2, customerI1)
    else
        deleteat!(solver.currSol.routes[r], [i, i+1])
        insert!(solver.currSol.routes[r], j, customerI2)
        insert!(solver.currSol.routes[r], j, customerI1)
    end
end

function applyMoveIntraSwap11!(solver::Solver, newCost::Float64, r::Int, i::Int, j::Int)
    solver.currSol.cost = newCost
    solver.currSol.routes[r][i], solver.currSol.routes[r][j] = solver.currSol.routes[r][j], solver.currSol.routes[r][i]
end

function applyMove2opt!(solver::Solver, newCost::Float64, r::Int, i::Int, j::Int)
    solver.currSol.cost = newCost
    reverse!(solver.currSol.routes[r], i, j)
end

function applyMoveInterShift10!(solver::Solver, newCost::Float64, r1::Int, r2::Int, i::Int, j::Int)
    solver.currSol.cost = newCost
    customerI = solver.currSol.routes[r1][i]
    deleteat!(solver.currSol.routes[r1], i)
    insert!(solver.currSol.routes[r2], j, customerI)
end

function applyMoveInterShift20!(solver::Solver, newCost::Float64, newTotalViol::Int, newResViolR1::Int, newResViolR2::Int, r1::Int, r2::Int, i::Int, j::Int)
    solver.currSol.cost = newCost
    solver.currSol.totalViolation = newTotalViol
    solver.currSol.resViolation[r1] = newResViolR1
    solver.currSol.resViolation[r2] = newResViolR2

    customerI1 = solver.currSol.routes[r1][i]
    customerI2 = solver.currSol.routes[r1][i+1]
    deleteat!(solver.currSol.routes[r1], [i, i+1])
    insert!(solver.currSol.routes[r2], j, customerI2)
    insert!(solver.currSol.routes[r2], j, customerI1)
end

function applyMoveInterSwap11!(solver::Solver, newCost::Float64, r1::Int, r2::Int, i::Int, j::Int)
    solver.currSol.cost = newCost

    customerI = solver.currSol.routes[r1][i]
    customerJ = solver.currSol.routes[r2][j]
    solver.currSol.routes[r1][i] = customerJ
    solver.currSol.routes[r2][j] = customerI
end

function applyMoveInterSwap22!(solver::Solver, newCost::Float64, newResViol::Int, r1::Int, r2::Int, i::Int, j::Int)
    solver.currSol.cost = newCost
    solver.currSol.resViolation = newResViol
    customerI1 = solver.currSol.routes[r1][i]
    customerI2 = solver.currSol.routes[r1][i]
    customerJ1 = solver.currSol.routes[r2][j]
    customerJ2 = solver.currSol.routes[r2][j]
    solver.currSol.routes[r1][i] = customerJ1
    solver.currSol.routes[r2][i+1] = customerJ2
    solver.currSol.routes[r1][j] = customerI1
    solver.currSol.routes[r2][j+1] = customerI2
end

function applyMoveTwoOptStar!(solver::Solver, newCost::Float64, r1::Int, r2::Int, i::Int, j::Int)
    solver.currSol.cost = newCost
    seg1 = solver.currSol.routes[r1][i+1:end]
    seg2 = solver.currSol.routes[r2][j+1:end]
    solver.currSol.routes[r1] = vcat(solver.currSol.routes[r1][1:i], seg2)
    solver.currSol.routes[r2] = vcat(solver.currSol.routes[r2][1:j], seg1)
end

function applyMoveSplit!(solver::Solver, newCost::Float64, r::Int, i::Int)
    solver.currSol.cost = newCost
    split1 = solver.currSol.routes[r][1:i-1]
    push!(split1, 0)
    split2 = solver.currSol.routes[r][i:end]
    pushfirst!(split2, 0)
    solver.currSol.routes[r] = split1
    push!(solver.currSol.routes, split2)
end