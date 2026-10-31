# Runs the PCVRP instances of a folder on several seeds and reports the profit (prizes - distance) collected.
# The minimum demand `Q` of the instances is not enforced (see `main`).
# Usage: julia --project=. runBatchPCVRP.jl [seeds] [seconds] [folder] [name filter]
# The folder defaults to data/. A `bks.csv` (`instance,bks`, the best known profit) in it enables the gaps.
include("mainPCVRP.jl")

"""Best known profits from `bks.csv` in `folder` (instance name without extension -> profit); empty if there is none."""
function readBKS(folder::String)
    file = joinpath(folder, "bks.csv")
    isfile(file) || return Dict{String, Float64}()
    return Dict(String(split(l, ',')[1]) => parse(Float64, split(l, ',')[2]) for l in readlines(file)[2:end] if !isempty(strip(l)))
end

"""
Check the best solution of `solver` against the instance, independently of the labels and of the cost matrix: the
distance of every route is recomputed from the coordinates, and the capacity, the number of routes and the
customers visited are checked. Returns the problems found (empty when valid), the profit (prizes minus distance)
and the demand satisfied.
"""
function verify(solver::Solver, pc)
    problems = String[]
    routes = [rt.visits for rt in solver.bestFeasSol.routes if length(rt.visits) > 2]
    travel = 0.0
    for (r, v) in enumerate(routes)
        stops = vcat(0, v[2:end-1], 0)
        travel += sum(hypot(pc.coords[stops[k] + 1][1] - pc.coords[stops[k + 1] + 1][1],
                            pc.coords[stops[k] + 1][2] - pc.coords[stops[k + 1] + 1][2]) for k in 1:length(stops)-1)
        load = sum(pc.demand[x + 1] for x in v[2:end-1])
        load > pc.capacity + 1e-6 && push!(problems, "route $r has load $load > $(pc.capacity)")
    end
    length(routes) > pc.vehicles && push!(problems, "$(length(routes)) routes for $(pc.vehicles) vehicles")
    visited = [x for v in routes for x in v[2:end-1]]
    length(unique(visited)) == length(visited) || push!(problems, "a customer is visited more than once")
    all(x -> 1 <= x < length(pc.coords), visited) || push!(problems, "unknown customer")
    return problems, sum(pc.prize[x + 1] for x in visited; init = 0.0) - travel, sum(pc.demand[x + 1] for x in visited; init = 0.0)
end

function runBatch(seeds::Int, seconds::Float64, folder::String, filter::String)
    files = sort([f for f in readdir(folder) if endswith(f, ".txt") && occursin(Regex(filter), f)])
    bks = readBKS(folder)
    println("folder = $folder, seeds = 1:$seeds, cooling time = $(seconds)s\n")
    @printf("%-15s %4s %3s %5s | %9s %9s %9s | %7s | %8s\n", "instance", "n", "m", "Q", "bks", "best", "avg", "served", "avg t(s)")
    for file in files
        name = splitext(file)[1]
        pc = readPCVRP(joinpath(folder, file))
        known = get(bks, name, NaN)
        profits, servedDemand, times = Float64[], Float64[], Float64[]
        for seed in 1:seeds
            solver = nothing
            t = @elapsed redirect_stdout(devnull) do
                solver = main(joinpath(folder, file), 1000, 5, seed; seconds = seconds)
            end
            problems, profit, demandServed = verify(solver, pc)
            isempty(problems) || println("  !! $name seed $seed INVALID: ", join(problems, "; "))
            push!(profits, isempty(problems) ? profit : -Inf)
            push!(servedDemand, demandServed)
            push!(times, t)
        end
        best = argmax(profits)
        @printf("%-15s %4d %3d %5.0f | %9.2f %9.2f %9.2f | %7.0f | %8.1f\n", name, length(pc.coords) - 1, pc.vehicles, pc.quota,
            known, maximum(profits), sum(profits) / seeds, servedDemand[best], sum(times) / seeds)
        println("    profits: ", join(round.(profits; digits = 2), ", "))
    end
end

if abspath(PROGRAM_FILE) == @__FILE__
    seeds = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 3
    seconds = length(ARGS) >= 2 ? parse(Float64, ARGS[2]) : 10.0
    folder = length(ARGS) >= 3 ? ARGS[3] : joinpath(@__DIR__, "data")
    filter = length(ARGS) >= 4 ? ARGS[4] : ""
    runBatch(seeds, seconds, folder, filter)
end
