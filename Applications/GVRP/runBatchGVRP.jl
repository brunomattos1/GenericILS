# Runs every instance of data/ on several seeds and reports the gap to the known optimum.
# Usage: julia --project=. runBatchGVRP.jl [seeds] [outerIterMax] [innerIterMax]
include("mainGVRP.jl")

# optimal costs that have no .sol file next to the instance
const EXTRA_OPTIMA = Dict("M-n101-k10-C34-V4" => 458.0)

"""Known optimal cost: the `Cost:` line of the `.sol` file of the instance, or `EXTRA_OPTIMA`."""
function knownOptimum(name::String)
    sol = joinpath(@__DIR__, "data", name * ".sol")
    if isfile(sol)
        for line in eachline(sol)
            startswith(line, "Cost:") && return parse(Float64, split(line, ':')[2])
        end
    end
    return get(EXTRA_OPTIMA, name, NaN)
end

"""
Check the best solution of `solver` against the instance, independently of the labels: distance recomputed
from the cost matrix, capacity of every route, one visit per cluster and number of routes. Returns the list
of problems found (empty when the solution is valid) and the recomputed distance.
"""
function verify(solver::Solver, gvrp, dist)
    problems = String[]
    routes = [rt.visits for rt in solver.bestFeasSol.routes if length(rt.visits) > 2]
    total = sum(sum(dist[v[k]+1, v[k+1]+1] for k in 1:length(v)-1) for v in routes; init = 0.0)
    for (r, v) in enumerate(routes)
        load = sum(gvrp.demand[x+1] for x in v[2:end-1])
        load > gvrp.capacity + 1e-6 && push!(problems, "route $r has load $load > $(gvrp.capacity)")
    end
    length(routes) > gvrp.vehicles && push!(problems, "$(length(routes)) routes for $(gvrp.vehicles) vehicles")
    append!(problems, groupViolations(solver.data, routes))
    abs(total - solver.bestFeasSol.cost) > 1e-6 && push!(problems, "cost $(solver.bestFeasSol.cost) differs from recomputed distance $total")
    return problems, total
end

function runBatch(seeds::Int, outerIterMax::Int, innerIterMax::Int)
    names = sort([splitext(f)[1] for f in readdir(joinpath(@__DIR__, "data")) if endswith(f, ".vrp")])
    println("seeds = 1:$seeds, outer = $outerIterMax, inner = $innerIterMax\n")
    @printf("%-20s %8s | %8s %8s %8s | %7s %7s | %4s | %8s\n",
        "instance", "optimum", "best", "avg", "worst", "gap%best", "gap%avg", "hits", "avg t(s)")
    for name in names
        gvrp = readGVRP(joinpath(@__DIR__, "data", name * ".vrp"))
        dist = euclideanMatrix(gvrp.coords)
        opt = knownOptimum(name)
        costs, times = Float64[], Float64[]
        for seed in 1:seeds
            solver = nothing
            t = @elapsed redirect_stdout(devnull) do
                solver = main(name * ".vrp", outerIterMax, innerIterMax, seed)
            end
            problems, total = verify(solver, gvrp, dist)
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
    runBatch(seeds, outerIterMax, innerIterMax)
end
