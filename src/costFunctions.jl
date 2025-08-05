function manualCost(sol::Solution, costMatrix::Matrix{Float64})
    cost = 0.
    for r = 1:length(sol.routes)
        for i = 1:length(sol.routes[r])-1
            cost += costMatrix[sol.routes[r][i]+1, sol.routes[r][i+1]+1]
        end
    end
    return cost
end

function bestInsertionCost(currCost::Float64, costMatrix::Matrix{Float64}, route::Vector{Int}, customer::Int, j::Int)
    newCost = currCost - costMatrix[route[j-1]+1, route[j]+1] + costMatrix[route[j-1]+1, customer+1] + costMatrix[customer+1, route[j]+1]
    return newCost
end

function intraShift10Cost2(currCost::Float64, costMatrix::Matrix{Float64}, route::Vector{Int}, i::Int, j::Int)
    if j > i + 1
        newCost = currCost - costMatrix[route[i-1]+1, route[i]+1] - costMatrix[route[i]+1, route[i+1]+1] - costMatrix[route[j]+1, route[j+1]+1]
        newCost += costMatrix[route[i-1]+1, route[i+1]+1] + costMatrix[route[j]+1, route[i]+1] + costMatrix[route[i]+1, route[j+1]+1]
    end
    if i > j + 1
        # @show route[i-1], route[i], route[i+1], route[j-1], route[j], route[j+1]
        newCost = currCost - costMatrix[route[i-1]+1, route[i]+1] - costMatrix[route[i]+1, route[i+1]+1] - costMatrix[route[j-1]+1, route[j]+1]
        newCost += costMatrix[route[i-1]+1, route[i+1]+1] + costMatrix[route[j-1]+1, route[i]+1] + costMatrix[route[i]+1, route[j]+1]
    end
    if j == i+1
        newCost = currCost - costMatrix[route[i-1]+1, route[i]+1] - costMatrix[route[i]+1, route[i+1]+1] - costMatrix[route[j]+1, route[j+1]+1]
        newCost += costMatrix[route[i-1]+1, route[i+1]+1] + costMatrix[route[j]+1, route[i]+1] + costMatrix[route[i]+1, route[j+1]+1]
    end
    if i == j+1
        newCost = currCost - costMatrix[route[i-1]+1, route[i]+1] - costMatrix[route[i]+1, route[i+1]+1] - costMatrix[route[j-1]+1, route[j]+1]
        newCost += costMatrix[route[j-1]+1, route[i]+1] + costMatrix[route[i]+1, route[j]+1] + costMatrix[route[i-1]+1, route[i+1]+1]
    end
    return newCost
end

function intraShift10Cost(currCost::Float64, costMatrix::Matrix{Float64}, route::Vector{Int}, i::Int, j::Int)
    newCost = currCost - costMatrix[route[i-1]+1, route[i]+1] - costMatrix[route[i]+1, route[i+1]+1] - costMatrix[route[j-1]+1, route[j]+1]
    newCost += costMatrix[route[i-1]+1, route[i+1]+1] + costMatrix[route[j-1]+1, route[i]+1] + costMatrix[route[i]+1, route[j]+1]
    return newCost
end

function intraShift20Cost(currCost::Float64, costMatrix::Matrix{Float64}, route::Vector{Int}, i::Int, j::Int)
    newCost = currCost - costMatrix[route[i-1]+1, route[i]+1] - costMatrix[route[i+1]+1, route[i+2]+1] - costMatrix[route[j-1]+1, route[j]+1]
    newCost += costMatrix[route[i-1]+1, route[i+2]+1] + costMatrix[route[j-1]+1, route[i]+1] + costMatrix[route[i+1]+1, route[j]+1]
    return newCost
end

function intraSwap11Cost(currCost::Float64, costMatrix::Matrix{Float64}, route::Vector{Int}, i::Int, j::Int)
    if i == j-1
        newCost = currCost - costMatrix[route[i-1]+1, route[i]+1] - costMatrix[route[i]+1, route[i+1]+1] - costMatrix[route[j]+1, route[j+1]+1]
        newCost += costMatrix[route[i-1]+1, route[j]+1] + costMatrix[route[j]+1, route[i]+1] + costMatrix[route[i]+1, route[j+1]+1]
    else
        newCost = currCost - costMatrix[route[i-1]+1, route[i]+1] - costMatrix[route[i]+1, route[i+1]+1] - costMatrix[route[j-1]+1, route[j]+1] - costMatrix[route[j]+1, route[j+1]+1]
        newCost += costMatrix[route[i-1]+1, route[j]+1] + costMatrix[route[j]+1, route[i+1]+1] + costMatrix[route[j-1]+1, route[i]+1] + costMatrix[route[i]+1, route[j+1]+1]
    end
    return newCost
end

function twoOptCost(currCost::Float64, costMatrix::Matrix{Float64}, route::Vector{Int}, i::Int, j::Int)
    newCost = currCost - costMatrix[route[i-1]+1, route[i]+1] - costMatrix[route[j]+1, route[j+1]+1]
    newCost += costMatrix[route[i]+1, route[j+1]+1] + costMatrix[route[i-1]+1, route[j]+1]
    return newCost
end

function interShift10Cost(currCost::Float64, costMatrix::Matrix{Float64}, route1::Vector{Int}, route2::Vector{Int}, i::Int, j::Int)
    newCost = currCost - costMatrix[route1[i-1]+1, route1[i]+1] - costMatrix[route1[i]+1, route1[i+1]+1] - costMatrix[route2[j-1]+1, route2[j]+1]
    newCost += costMatrix[route1[i-1]+1, route1[i+1]+1] + costMatrix[route2[j-1]+1, route1[i]+1] + costMatrix[route1[i]+1, route2[j]+1]
    return newCost
end

function interShift20Cost(currCost::Float64, costMatrix::Matrix{Float64}, route1::Vector{Int}, route2::Vector{Int}, i::Int, j::Int)
    newCost = currCost - costMatrix[route1[i-1]+1, route1[i]+1] - costMatrix[route1[i+1]+1, route1[i+2]+1] - costMatrix[route2[j-1]+1, route2[j]+1]
    newCost += costMatrix[route1[i-1]+1, route1[i+2]+1] + costMatrix[route2[j-1]+1, route1[i]+1] + costMatrix[route1[i+1]+1, route2[j]+1]
    return newCost
end

function interSwap11Cost(currCost::Float64, costMatrix::Matrix{Float64}, route1::Vector{Int}, route2::Vector{Int}, i::Int, j::Int)
    newCost = currCost - costMatrix[route1[i-1]+1, route1[i]+1] - costMatrix[route1[i]+1, route1[i+1]+1] - costMatrix[route2[j-1]+1, route2[j]+1] - costMatrix[route2[j]+1, route2[j+1]+1]
    newCost += costMatrix[route1[i-1]+1, route2[j]+1] + costMatrix[route2[j]+1, route1[i+1]+1] + costMatrix[route2[j-1]+1, route1[i]+1]+ costMatrix[route1[i]+1, route2[j+1]+1]
    return newCost
end

function interSwap22Cost(currCost::Float64, costMatrix::Matrix{Float64}, route1::Vector{Int}, route2::Vector{Int}, i::Int, j::Int)
    newCost = currCost - costMatrix[route1[i-1]+1, route1[i]+1] - costMatrix[route1[i+1]+1, route1[i+2]+1] - costMatrix[route2[j-1]+1, route2[j]+1] - costMatrix[route2[j+1]+1, route2[j+2]+1]
    newCost += costMatrix[route1[i-1]+1, route2[j]+1] + costMatrix[route2[j+1]+1, route1[i+2]+1] + costMatrix[route2[j-1]+1, route1[i]+1]+ costMatrix[route1[i+1]+1, route2[j+2]+1]
    return newCost
end

function splitCost(currCost::Float64, costMatrix::Matrix{Float64}, route::Vector{Int}, i::Int)
    newCost = currCost - costMatrix[route[i-1]+1, route[i]+1]
    newCost += costMatrix[route[i-1]+1, 1] + costMatrix[1, route[i]+1]
    return newCost
end

function twoOptStarCost(currCost::Float64, costMatrix::Matrix{Float64}, route1::Vector{Int}, route2::Vector{Int}, i::Int, j::Int)
    newCost = currCost - costMatrix[route1[i]+1, route1[i+1]+1] - costMatrix[route2[j]+1, route2[j+1]+1]
    newCost += costMatrix[route1[i]+1, route2[j+1]+1] + costMatrix[route2[j]+1, route1[i+1]+1]
    return newCost
end