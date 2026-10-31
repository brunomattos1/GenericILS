# Runs the TOP instances of a folder on several seeds and reports the profit (and the gap to the best known one).
# Usage: julia --project=. runBatchTOP.jl [seeds] [outerIterMax] [innerIterMax] [folder] [name filter]
# The folder defaults to data/. A `bks.csv` (`instance,bks`, the best known profit) in it enables the gaps.
include("mainTOP.jl")

"""Best known profits from `bks.csv` in `folder` (instance name without extension -> profit); empty if there is none."""
function readBKS(folder::String)
    file = joinpath(folder, "bks.csv")
    isfile(file) || return Dict{String, Float64}()
    return Dict(String(split(l, ',')[1]) => parse(Float64, split(l, ',')[2]) for l in readlines(file)[2:end] if !isempty(strip(l)))
end

"""
Check the best solution of `solver` against the instance, independently of the labels and of the cost matrix: the
length of every path is recomputed from the coordinates (start and end included), and the number of paths and the
customers visited are checked. Returns the problems found (empty when valid) and the profit collected.
"""
function verify(solver::Solver, top)
    problems = String[]
    routes = [rt.visits for rt in solver.bestFeasSol.routes if length(rt.visits) > 2]
    point(i) = top.coords[i == 0 ? 1 : i + 1]
    dist(a, b) = hypot(point(a)[1] - point(b)[1], point(a)[2] - point(b)[2])
    for (r, v) in enumerate(routes)
        stops = vcat(1, v[2:end-1] .+ 1, top.n + 2)             # start, customers, end (indices of top.coords)
        len = sum(hypot(top.coords[stops[k]][1] - top.coords[stops[k+1]][1], top.coords[stops[k]][2] - top.coords[stops[k+1]][2])
                  for k in 1:length(stops)-1)
        len > top.tmax + 1e-6 && push!(problems, "path $r has length $len > $(top.tmax)")
    end
    length(routes) > top.vehicles && push!(problems, "$(length(routes)) paths for $(top.vehicles) vehicles")
    visited = [x for v in routes for x in v[2:end-1]]
    length(unique(visited)) == length(visited) || push!(problems, "a customer is visited more than once")
    all(x -> 1 <= x <= top.n, visited) || push!(problems, "unknown customer")
    return problems, sum(top.profit[x + 1] for x in visited; init = 0.0)
end

function runBatch(seeds::Int, outerIterMax::Int, innerIterMax::Int, folder::String, filter::String)
    files = sort([f for f in readdir(folder) if endswith(f, ".txt") && occursin(Regex(filter), f)])
    bks = readBKS(folder)
    println("folder = $folder, seeds = 1:$seeds, outer = $outerIterMax, inner = $innerIterMax\n")
    @printf("%-10s %4s %6s %7s | %6s %8s %6s | %8s %7s | %4s | %8s\n",
        "instance", "n", "m", "Tmax", "bks", "best", "avg", "gap%best", "gap%avg", "hits", "avg t(s)")
    for file in files
        name = splitext(file)[1]
        top = readTOP(joinpath(folder, file))
        known = get(bks, name, NaN)
        profits, times = Float64[], Float64[]
        for seed in 1:seeds
            solver = nothing
            t = @elapsed redirect_stdout(devnull) do
                solver = main(joinpath(folder, file), outerIterMax, innerIterMax, seed)
            end
            problems, collected = verify(solver, top)
            isempty(problems) || println("  !! $name seed $seed INVALID: ", join(problems, "; "))
            push!(profits, isempty(problems) ? collected : -Inf)
            push!(times, t)
        end
        gap(p) = 100 * (known - p) / known
        @printf("%-10s %4d %6d %7.1f | %6.0f %8.0f %6.1f | %8.2f %7.2f | %2d/%d | %8.1f\n", name, top.n, top.vehicles, top.tmax, known,
            maximum(profits), sum(profits) / seeds, gap(maximum(profits)), gap(sum(profits) / seeds),
            count(p -> p >= known - 1e-6, profits), seeds, sum(times) / seeds)
        println("    profits: ", join(Int.(profits), ", "))
    end
end

if abspath(PROGRAM_FILE) == @__FILE__
    seeds = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 3
    outerIterMax = length(ARGS) >= 2 ? parse(Int, ARGS[2]) : 1000
    innerIterMax = length(ARGS) >= 3 ? parse(Int, ARGS[3]) : 10
    folder = length(ARGS) >= 4 ? ARGS[4] : joinpath(@__DIR__, "data")
    filter = length(ARGS) >= 5 ? ARGS[5] : ""
    runBatch(seeds, outerIterMax, innerIterMax, folder, filter)
end
