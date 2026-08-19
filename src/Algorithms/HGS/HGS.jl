module HGS

using Random, JuMP, Printf
import ..GenericILS
using ..GenericILS:
    Solver, Solution, Route, AbstractResources, Algorithm,
    new_solution, new_route,
    objectiveValue, computeLabels, isCostResource, computeRouteLabelCost,
    myInitStateForward, myInitStateBackward, myExtendAlongArc,
    updateBestFeasSol!, updatePenalty, registerBestFeasibleBefSP!,
    search!, apply!,
    BestMove, Cost, ViolationInfo, Shift, OptStar,
    canPruneByDist, pruningFixedPenalty,
    TwoOptStar, IntraShift, InterShift, InterSwap,
    twoOptStarCost, computeViolTwoOptStar, evalTwoOptStar!,
    intraShift10Cost, computeViolIntraShift10, evalIntraShift10,
    interShiftCost, computeViolInterShiftK,
    interSwapCost, computeViolInterSwapK
import ..GenericILS: run!, totalTime, printInfo

export HGSAlgorithm, run!,
    ByGenerations, ByTime,
    getStopCriteria, setStopCriteria!,
    getMuMax, setMuMax!,
    getLambda, setLambda!,
    getNClosest, setNClosest!,
    getNElite, setNElite!,
    getNbIterNonProd, setNbIterNonProd!,
    getGranularK, setGranularK!,
    getTimeLimitSP, setTimeLimitSP!

include("Individual.jl")
include("Split.jl")
include("Diversity.jl")
include("Population.jl")
include("Selection.jl")
include("Crossover.jl")
include("HGSAlgorithm.jl")
include("StopCriteria.jl")
include("Granularity.jl")
include("RouteIndex.jl")
include("LocalSearch.jl")
include("SetPartitioning.jl")
include("run.jl")

end # module HGS
