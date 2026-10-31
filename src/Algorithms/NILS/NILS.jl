"""
Nested Iterated Local Search (`NILSAlgorithm`): an outer loop with accept/stop criteria and, for each candidate,
an inner loop of local search and light perturbation; finishes with a Set Partitioning step.
Its accept/stop criteria are specific to this module: use them qualified (`NILS.Metropolis`, ...).
"""
module NILS

using Random, JuMP, Printf
import ..GenericILS
using ..GenericILS:
    Solver, Solution, AbstractResources, Algorithm,
    newSolution, newRoute,
    objectiveValue, computeLabels, computeRouteLabelCost, refreshLabelCosts!,
    isSymmetric, isCostResource,
    apply!, InterShift, InterSwap, interShiftCost, interSwapCost,
    computeViolInterShiftK, computeViolInterSwapK,
    updateBestFeasSol!, updatePenalty, registerBestFeasibleBefSP!,
    Cost, BestMove,
    myInitStateForward, myInitStateBackward,
    constructSol!, search!, copySolution!, canLeaveRoute, addPartitioningConstraints!, manualCost, randomToggleOptional!
import ..GenericILS: run!, totalTime, printInfo

export NILSAlgorithm, run!, RVND!,
    Metropolis, MetropolisTimed, MetropolisTimedIter, AcceptBest, RandomWalk,
    ByTime, ByTemperature, ByTemperatureIter, ByIterMax,
    Diversification,
    getInnerIterMax, setInnerIterMax!,
    getDiversification, setDiversification!,
    getAcceptCriteria, setAcceptCriteria!,
    getStopCriteria, setStopCriteria!,
    getTimeLimits, setTimeLimits!,
    setTimeLimitILS, setTimeLimitSP, aggressivePool

include("NILSAlgorithm.jl")
include("AcceptCriteria.jl")
include("StopCriteria.jl")
include("Perturb.jl")
include("SetPartitioning.jl")
include("LocalSearch.jl")
include("innerLoop.jl")
include("run.jl")

end # module NILS
