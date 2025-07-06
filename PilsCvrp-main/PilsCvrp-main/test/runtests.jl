using Test, PilsCvrp

cfg = PilsCvrp.Config(max_nb_closest = 150)
data = PilsCvrp.readCVRPData("../data/M/M-n151-k12.vrp")
solver = PilsCvrp.build_solver(data, cfg)
PilsCvrp.construct!(solver)

@testset "Intra-route neighborhoods" begin
    @test 1 == 1
end
