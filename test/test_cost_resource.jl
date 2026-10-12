@testset "Cost resource (isCostResource() == true)" begin
    # Separate process: the resource flags are global, see cost_resource_script.jl
    script = joinpath(@__DIR__, "cost_resource_script.jl")
    cmd = `$(Base.julia_cmd()) --project=$(Base.active_project()) $script`
    out = read(cmd, String)
    @test occursin("COST_RESOURCE_OK", out)
end
