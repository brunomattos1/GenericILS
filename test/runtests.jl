using Test

include("helpers.jl")

@testset "GenericILS" begin
    include("testPenaltyManager.jl")
    include("testCriteria.jl")
    include("testLabels.jl")
    include("testSolutionConstruct.jl")
    include("testInfeasibility.jl")
    include("testNeighborhoods.jl")
    include("testSolverApi.jl")
    include("testAlgorithms.jl")
    include("testCostResource.jl")
    include("testUtils.jl")
end
