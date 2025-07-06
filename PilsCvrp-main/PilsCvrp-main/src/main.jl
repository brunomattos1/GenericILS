using Profile
module PilsCvrp

using Random
using Unicode: Unicode
using Parameters

include("data.jl")
include("solver.jl")

end # module PilsCvrp
cfg = PilsCvrp.Config(max_nb_closest = 5)
# data = PilsCvrp.readCVRPData("data/M/M-n151-k12.vrp")
# sol = PilsCvrp.solve(data, cfg)

# Profile.clear_malloc_data()

data = PilsCvrp.readCVRPData(raw"C:\Users\bruno.mattos\OneDrive - americanas s.a\Documentos\GitHub\GMHVRP\PilsCvrp-main\PilsCvrp-main\data\M\M-n101-k10.vrp")
println("Solving CVRP with $(length(data.G.V)) vertices")
sol = PilsCvrp.solve(data, cfg)
@show sol.cost

