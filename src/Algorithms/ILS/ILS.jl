module ILS

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

export ILSAlgorithm, run!, RVND!,
    Metropolis, MetropolisTimed, MetropolisTimedIter, AcceptBest, RandomWalk,
    ByTime, ByTemperature, ByTemperatureIter, ByIterMax,
    Diversification,
    getDiversification, setDiversification!,
    getAcceptCriteria, setAcceptCriteria!,
    getStopCriteria, setStopCriteria!,
    getTimeLimits, setTimeLimits!,
    setTimeLimitILS, setTimeLimitSP, aggressivePool

include("ILSAlgorithm.jl")
include("AcceptCriteria.jl")
include("StopCriteria.jl")
include("Perturb.jl")
include("SetPartitioning.jl")
include("LocalSearch.jl")
include("run.jl")

end # module ILS
