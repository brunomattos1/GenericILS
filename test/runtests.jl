using Test

include("helpers.jl")

@testset "GenericILS" begin
    include("test_penalty_manager.jl")
    include("test_criteria.jl")
    include("test_labels.jl")
    include("test_solution_construct.jl")
    include("test_infeasibility.jl")
    include("test_neighborhoods.jl")
    include("test_solver_api.jl")
    include("test_algorithms.jl")
    include("test_cost_resource.jl")
    include("test_utils.jl")
end
