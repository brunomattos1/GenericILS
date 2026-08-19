module GenericILS

using Random, JuMP, HiGHS, Printf
import Base: split

include("LabelsFunctions.jl")
include("Resources.jl")
include("Solution.jl")
include("PenaltyManager.jl")
include("Solver.jl")
include("Construct.jl")

include("Labels.jl")
include("Infeasibility.jl")
include("utils.jl")

include("neighborhoods/IntraShift.jl")
include("neighborhoods/TwoOptStar.jl")
include("neighborhoods/InterShift.jl")
include("neighborhoods/InterSwap.jl")
include("Neighborhoods.jl")

export
    # resources / labels (user extends these via `import GenericILS: ...`)
    AbstractResource, AbstractResources, Label,
    StandardResource, StandardState,
    ForwardLabel, BackwardLabel,
    Resources,
    isSymmetric, isCostResource,
    initStateForward, initStateBackward,
    extendAlongArc, concatenationCost,

    # problem / solver setup
    Vertex, ProblemData,
    StandardPenaltyManager, TargetRatePenaltyManager,
    Solver,

    # neighborhoods
    TwoOptStar, IntraShift, InterShift, InterSwap,

    # algorithms (abstract contract; each Algorithm submodule -- NILS, ILS, ... --
    # provides its own concrete Algorithm type + criteria vocabulary + run! method)
    Algorithm, run!, solve!,

    # solution / running
    Solution, UserSolution, Route,
    getCurrSol, getBestSol, getBestRoutes, getRoutes, getCost, getDistance, getCostMatrix,
    createSolution, computeLabels,
    new_solution, new_route, copy_solution!, updateBestFeasSol!,

    # reusable building blocks for custom algorithms (construction, local search)
    constructSol!, search!,

    # Solver getters/setters
    getSeed, setSeed!,
    getAlgorithm, setAlgorithm!,
    getPenaltyManager, setPenaltyManager!,
    getRes, setRes!,
    getData, setData!,
    getNeighborhoods, setNeighborhoods!,
    getMIPSolver, setMIPSolver!,
    getStatistics,

    # misc utilities used by applications
    printInfo, printLabels, printConcatenations, manualCost,
    objectiveValue, canPruneByDist, totalTime

include("Algorithms/NILS/NILS.jl")
include("Algorithms/ILS/ILS.jl")
include("Algorithms/HGS/HGS.jl")

# Only the concrete Algorithm types (unique names) and the run!/solve! entry
# points are re-exported without qualification. Each submodule's criteria
# vocabulary (Metropolis, ByTemperature, Diversification, ...) is duplicated
# independently per algorithm and must be accessed qualified, e.g.
# NILS.Metropolis(...) or ILS.Metropolis(...), to avoid name ambiguity.
using .NILS: NILSAlgorithm
using .ILS: ILSAlgorithm
using .HGS: HGSAlgorithm
export NILSAlgorithm, ILSAlgorithm, HGSAlgorithm, NILS, ILS, HGS

solve!(solver::Solver) = run!(solver.algorithm, solver)

end # module GenericILS
