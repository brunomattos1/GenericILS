mutable struct Parameters
    restarts::Int
    outerIterMax::Int
    innerIterMax::Int
    penaltyCustom::Float64
    penaltyCustomIncrease::Float64
    penaltyCustomDecrease::Float64
    penaltyStandard1::Float64
    penaltyStandard1Increase::Float64
    penaltyStandard1Decrease::Float64
    penaltyStandard2::Float64
    penaltyStandard2Increase::Float64
    penaltyStandard2Decrease::Float64
end
function Parameters(;
    restarts = 10,
    outerIterMax = 10,
    innerIterMax = 3,
    penaltyCustom = 100.0,
    penaltyCustomIncrease = 0.01,
    penaltyCustomDecrease = 0.01,
    penaltyStandard1 = 100.0,
    penaltyStandard1Increase = 0.01,
    penaltyStandard1Decrease = 0.01,
    penaltyStandard2 = 100.0,
    penaltyStandard2Increase = 0.01,
    penaltyStandard2Decrease = 0.01)
    return Parameters(restarts, outerIterMax, innerIterMax, penaltyCustom, penaltyCustomIncrease, penaltyCustomDecrease, 
    penaltyStandard1, penaltyStandard1Increase, penaltyStandard1Decrease, penaltyStandard2, penaltyStandard2Increase, penaltyStandard2Decrease)
end

function updatePenalty(parameters::Parameters, sol::Solution)
    if sol.totalInfeas == 0
        parameters.penaltyCustom = max((1 - parameters.penaltyCustomDecrease)*parameters.penaltyCustom, 0.1)
    else
        parameters.penaltyCustom = min((1 + parameters.penaltyCustomIncrease)*parameters.penaltyCustom, 10000.0)
    end
    if sol.totalWarpStd1 <= 1e-6
        parameters.penaltyStandard1 = max((1 - parameters.penaltyStandard1Decrease)*parameters.penaltyStandard1, 0.1)
    else
        parameters.penaltyStandard1 = min((1 + parameters.penaltyStandard1Increase)*parameters.penaltyStandard1, 10000.0)
    end
    if sol.totalWarpStd2 <= 1e-6
        parameters.penaltyStandard2 = max((1 - parameters.penaltyStandard2Decrease)*parameters.penaltyStandard2, 0.1)
    else
        parameters.penaltyStandard2 = min((1 + parameters.penaltyStandard2Increase)*parameters.penaltyStandard2, 10000.0)
    end
end