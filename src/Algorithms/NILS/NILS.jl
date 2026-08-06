module NILS

using Random, JuMP, Printf
import ..GenericILS
using ..GenericILS:
    Solver, Solution, AbstractResources, Algorithm,
    new_solution, new_route,
    objectiveValue, computeLabels, computeRouteLabelCost,
    isSymmetric, isCostResource,
    apply!, InterShift, InterSwap, interShiftCost, interSwapCost,
    computeViolInterShiftK, computeViolInterSwapK,
    updateBestFeasSol!, updatePenalty, registerBestFeasibleBefSP!,
    Cost, BestMove,
    myInitStateForward, myInitStateBackward,
    constructSol!, search!, copy_solution!
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
