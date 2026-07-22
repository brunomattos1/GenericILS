abstract type PenaltyManager end

# Implementacao antiga: ajuste incremental a cada iteracao (sem janela).
mutable struct StandardPenaltyManager <: PenaltyManager
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
function StandardPenaltyManager(;
    penaltyCustom = 100.0,
    penaltyCustomIncrease = 0.01,
    penaltyCustomDecrease = 0.01,
    penaltyStandard1 = 100.0,
    penaltyStandard1Increase = 0.01,
    penaltyStandard1Decrease = 0.01,
    penaltyStandard2 = 100.0,
    penaltyStandard2Increase = 0.01,
    penaltyStandard2Decrease = 0.01)
    return StandardPenaltyManager(
        penaltyCustom, penaltyCustomIncrease, penaltyCustomDecrease,
        penaltyStandard1, penaltyStandard1Increase, penaltyStandard1Decrease,
        penaltyStandard2, penaltyStandard2Increase, penaltyStandard2Decrease
    )
end

function updatePenalty(pm::StandardPenaltyManager, sol::Solution)
    if sol.totalInfeas == 0
        pm.penaltyCustom = max((1 - pm.penaltyCustomDecrease)*pm.penaltyCustom, 0.1)
    else
        pm.penaltyCustom = min((1 + pm.penaltyCustomIncrease)*pm.penaltyCustom, 10000.0)
    end
    if sol.totalWarpStd1 <= 1e-6
        pm.penaltyStandard1 = max((1 - pm.penaltyStandard1Decrease)*pm.penaltyStandard1, 0.1)
    else
        pm.penaltyStandard1 = min((1 + pm.penaltyStandard1Increase)*pm.penaltyStandard1, 10000.0)
    end
    if sol.totalWarpStd2 <= 1e-6
        pm.penaltyStandard2 = max((1 - pm.penaltyStandard2Decrease)*pm.penaltyStandard2, 0.1)
    else
        pm.penaltyStandard2 = min((1 + pm.penaltyStandard2Increase)*pm.penaltyStandard2, 10000.0)
    end
    return nothing
end

mutable struct TargetRatePenaltyManager <: PenaltyManager
    penaltyCustom::Float64
    penaltyCustomIncrease::Float64
    penaltyCustomDecrease::Float64
    penaltyStandard1::Float64
    penaltyStandard1Increase::Float64
    penaltyStandard1Decrease::Float64
    penaltyStandard2::Float64
    penaltyStandard2Increase::Float64
    penaltyStandard2Decrease::Float64
    targetFeasRate::Float64
    feasRateTolerance::Float64
    updatePeriod::Int
    customFeasCount::Int
    standard1FeasCount::Int
    standard2FeasCount::Int
    windowCount::Int
end
function TargetRatePenaltyManager(;
    penaltyCustom = 1000.0,
    penaltyCustomIncrease = 0.4,
    penaltyCustomDecrease = 0.2,
    penaltyStandard1 = 1000.0,
    penaltyStandard1Increase = 0.4,
    penaltyStandard1Decrease = 0.2,
    penaltyStandard2 = 1000.0,
    penaltyStandard2Increase = 0.4,
    penaltyStandard2Decrease = 0.2,
    targetFeasRate = 0.7,
    feasRateTolerance = 0.05,
    updatePeriod = 30)
    return TargetRatePenaltyManager(
        penaltyCustom, penaltyCustomIncrease, penaltyCustomDecrease,
        penaltyStandard1, penaltyStandard1Increase, penaltyStandard1Decrease,
        penaltyStandard2, penaltyStandard2Increase, penaltyStandard2Decrease,
        targetFeasRate, feasRateTolerance, updatePeriod,
        0, 0, 0, 0
    )
end

function updatePenalty(pm::TargetRatePenaltyManager, sol::Solution)
    pm.customFeasCount    += sol.totalInfeas    <= 1e-6 ? 1 : 0
    pm.standard1FeasCount += sol.totalWarpStd1  <= 1e-6 ? 1 : 0
    pm.standard2FeasCount += sol.totalWarpStd2  <= 1e-6 ? 1 : 0
    pm.windowCount        += 1

    if pm.windowCount < pm.updatePeriod
        return nothing  # janela ainda nao encheu
    end

    ρ_custom    = pm.customFeasCount    / pm.windowCount
    ρ_standard1 = pm.standard1FeasCount / pm.windowCount
    ρ_standard2 = pm.standard2FeasCount / pm.windowCount

    if ρ_custom < pm.targetFeasRate - pm.feasRateTolerance
        pm.penaltyCustom = min(10000.0, pm.penaltyCustom * (1.0 + pm.penaltyCustomIncrease))
    elseif ρ_custom > pm.targetFeasRate + pm.feasRateTolerance
        pm.penaltyCustom = max(0.01,   pm.penaltyCustom / (1.0 + pm.penaltyCustomDecrease))
    end

    if ρ_standard1 < pm.targetFeasRate - pm.feasRateTolerance
        pm.penaltyStandard1 = min(10000.0, pm.penaltyStandard1 * (1.0 + pm.penaltyStandard1Increase))
    elseif ρ_standard1 > pm.targetFeasRate + pm.feasRateTolerance
        pm.penaltyStandard1 = max(0.01,   pm.penaltyStandard1 / (1.0 + pm.penaltyStandard1Decrease))
    end

    if ρ_standard2 < pm.targetFeasRate - pm.feasRateTolerance
        pm.penaltyStandard2 = min(10000.0, pm.penaltyStandard2 * (1.0 + pm.penaltyStandard2Increase))
    elseif ρ_standard2 > pm.targetFeasRate + pm.feasRateTolerance
        pm.penaltyStandard2 = max(0.01,   pm.penaltyStandard2 / (1.0 + pm.penaltyStandard2Decrease))
    end

    # Reseta a janela: "encher e zerar" (igual ao HGS), nao e buffer circular.
    pm.customFeasCount    = 0
    pm.standard1FeasCount = 0
    pm.standard2FeasCount = 0
    pm.windowCount        = 0
    return nothing
end
