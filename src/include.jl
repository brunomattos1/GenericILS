using Random, JuMP, HiGHS, Printf
import Base: push!
import Base: hash

include("resources.jl")
include("structs.jl")
include("setPartitioning.jl")
include("subsequences.jl")
include("computeViolation.jl")
include("acceptCriteria.jl")
include("algorithms.jl")
include("applyMoves.jl")
include("construct.jl")
include("costFunctions.jl")
include("evalMoves.jl")
include("localSearch.jl")
include("neighborhood.jl")
include("perturb.jl")
