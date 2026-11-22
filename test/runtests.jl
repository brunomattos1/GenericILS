using Test

include("helpers.jl")

@testset "GenericILS" begin
    include("testPenaltyManager.jl")
    include("testCriteria.jl")
    include("testLabels.jl")
    include("testExtendToVertex.jl")
    include("testResourceFlags.jl")
    include("testCppResource.jl")
    # needs VrpSolver.jl, which is not a dependency of the package, and a C++ compiler
    if Base.find_package("VrpSolver") === nothing
        @info "VrpSolver.jl is not installed: the adapter tests are skipped"
    elseif GenericILS.CppResources.cppCompiler() !== nothing
        include("testVrpSolverAdapter.jl")
    end
    include("testSolutionConstruct.jl")
    include("testGroups.jl")
    include("testInfeasibility.jl")
    include("testNeighborhoods.jl")
    include("testIntraGroupExchange.jl")
    include("testGroupMoves.jl")
    include("testSetPartitioning.jl")
    include("testOptionalVertices.jl")
    include("testOptionalResource.jl")
    include("testSolverApi.jl")
    include("testAlgorithms.jl")
    include("testCostResource.jl")
    include("testUtils.jl")
end
