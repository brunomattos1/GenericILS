@testset "Optional vertices rewarded by a resource (isCostResource() == true)" begin
    # Separate process: the resource flags are global, see optionalResourceScript.jl
    script = joinpath(@__DIR__, "optionalResourceScript.jl")
    cmd = `$(Base.julia_cmd()) --project=$(Base.active_project()) $script`
    out = read(cmd, String)
    @test occursin("OPTIONAL_RESOURCE_OK", out)
end
