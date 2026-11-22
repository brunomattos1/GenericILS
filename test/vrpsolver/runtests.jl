# Only the tests of the adapter, in an environment that has VrpSolver.jl (see Project.toml). From the root of the package:
#   julia --project=test/vrpsolver -e 'using Pkg; Pkg.instantiate()'
#   julia --project=test/vrpsolver test/vrpsolver/runtests.jl
# The whole suite (test/runtests.jl) also runs them when it finds VrpSolver.jl.
using Test

include("../helpers.jl")
include("../cppHelpers.jl")

if GenericILS.CppResources.cppCompiler() === nothing
    error("the adapter tests need a C++ compiler (install g++ or clang++, or set CXX)")
end
@testset "adapter" begin
    include("../testVrpSolverAdapter.jl")
end
