# Runs every .fcvrp instance of a folder on several seeds and reports the gap to the best known solutions.
# Usage: julia --project=. runBatchFCVRP.jl [seeds] [outerIterMax] [innerIterMax] [folder]
# The folder defaults to data/. A `bks.csv` (`instance,bks`) in it gives the best known costs.
include("mainFCVRP.jl")

"""Best known costs from `bks.csv` in `folder` (instance name without extension -> cost); empty if there is none."""
function readBKS(folder::String)
    file = joinpath(folder, "bks.csv")
    isfile(file) || return Dict{String, Float64}()
    return Dict(String(split(l, ',')[1]) => parse(Float64, split(l, ',')[2]) for l in readlines(file)[2:end] if !isempty(strip(l)))
end

"""
Check the best solution of `solver` against the instance, independently of the labels: distance recomputed
from the cost matrix, capacity of every route, visits per family, one visit per vertex and number of routes.
Returns the list of problems found (empty when the solution is valid) and the recomputed distance.
"""
function verify(solver::Solver, fcvrp, dist)
    problems = String[]
    routes = [rt.visits for rt in solver.bestFeasSol.routes if length(rt.visits) > 2]
    total = sum(sum(dist[v[k]+1, v[k+1]+1] for k in 1:length(v)-1) for v in routes; init = 0.0)
    demand = vertexDemands(fcvrp.families, length(solver.data.vertices))
    for (r, v) in enumerate(routes)
        load = sum(demand[x+1] for x in v[2:end-1])
        load > fcvrp.capacity + 1e-6 && push!(problems, "route $r has load $load > $(fcvrp.capacity)")
    end
    length(routes) > fcvrp.vehicles && push!(problems, "$(length(routes)) routes for $(fcvrp.vehicles) vehicles")
    visited = [x for v in routes for x in v[2:end-1]]
    length(unique(visited)) == length(visited) || push!(problems, "a vertex is visited more than once")
    for (f, family) in enumerate(fcvrp.families)
        count(x -> x in family.nodes, visited) == family.visits || push!(problems, "family $f is not visited $(family.visits) times")
    end
    abs(total - solver.bestFeasSol.cost) > 1e-6 && push!(problems, "cost $(solver.bestFeasSol.cost) differs from recomputed distance $total")
    return problems, total
end

function runBatch(seeds::Int, outerIterMax::Int, innerIterMax::Int, folder::String)
    files = sort([f for f in readdir(folder) if endswith(f, ".fcvrp")])
    bks = readBKS(folder)
    println("folder = $folder, seeds = 1:$seeds, outer = $outerIterMax, inner = $innerIterMax\n")
    @printf("%-20s %8s | %8s %8s %8s | %8s %7s | %4s | %8s\n",
        "instance", "bks", "best", "avg", "worst", "gap%best", "gap%avg", "hits", "avg t(s)")
    for file in files
        name = splitext(file)[1]
        fcvrp = readFCVRP(joinpath(folder, file))
        dist = euclideanMatrix(fcvrp.coords)
        opt = get(bks, name, NaN)
        costs, times = Float64[], Float64[]
        for seed in 1:seeds
            solver = nothing
            t = @elapsed redirect_stdout(devnull) do
                solver = main(joinpath(folder, file), outerIterMax, innerIterMax, seed)
            end
            problems, total = verify(solver, fcvrp, dist)
            isempty(problems) || println("  !! $name seed $seed INVALID: ", join(problems, "; "))
            push!(costs, isempty(problems) ? total : Inf)
            push!(times, t)
        end
        gap(c) = 100 * (c - opt) / opt
        @printf("%-20s %8.0f | %8.0f %8.1f %8.0f | %8.2f %7.2f | %2d/%d | %8.1f\n", name, opt,
            minimum(costs), sum(costs) / seeds, maximum(costs), gap(minimum(costs)), gap(sum(costs) / seeds),
            count(c -> abs(c - opt) < 1e-6, costs), seeds, sum(times) / seeds)
        println("    costs: ", join(Int.(costs), ", "))
    end
end

if abspath(PROGRAM_FILE) == @__FILE__
    seeds = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 5
    outerIterMax = length(ARGS) >= 2 ? parse(Int, ARGS[2]) : 1000
    innerIterMax = length(ARGS) >= 3 ? parse(Int, ARGS[3]) : 10
    folder = length(ARGS) >= 4 ? ARGS[4] : joinpath(@__DIR__, "data")
    runBatch(seeds, outerIterMax, innerIterMax, folder)
end
