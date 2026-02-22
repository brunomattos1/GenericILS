
using ArgParse

dest_dir = joinpath("..", "..", "src")

old = joinpath(dest_dir, "resources.jl")
bak = joinpath(dest_dir, "resources_backup.jl")

old_alg = joinpath(dest_dir, "algorithms.jl")
bak_alg = joinpath(dest_dir, "algorithms_backup.jl")

if isfile(old)
    mv(old, bak; force=true)
end

if isfile(old_alg)
    mv(old_alg, bak_alg; force=true)
end

src_file = joinpath("src", "resources.jl")
if isfile(src_file)
    cp(src_file, old; force=true)
end

src_file_alg = joinpath("src", "algorithms.jl")
if isfile(src_file_alg)
    cp(src_file_alg, old_alg; force=true)
end

include("model.jl")
include("data.jl")


function parse_commandline(args_array::Vector{String}, appfolder::String)
    s = ArgParseSettings(
        usage="##### Generic ILS #####\n\n" *
              "  On interactive mode, call main([\"arg1\", ..., \"argn\"])", exit_after_help=false)
    @add_arg_table s begin
        "instance"
        help = "Instance file path"

        "--lilim", "-l"
        help = "Li&Lim instance."
        action = :store_true

        "--round", "-R"
        help = "Round the distance matrix"
        action = :store_true

        "--seed", "-s"
        help = "Random seed"
        arg_type = Int
        default = 1

        "--restarts", "-r"
        help = "Number of restarts"
        arg_type = Int
        default = 1

        "--outerIterMax", "-o"
        help = "Maximum number of outer iterations"
        arg_type = Int
        default = 500

        "--innerIterMax", "-i"
        help = "Maximum number of inner iterations"
        arg_type = Int
        default = 10
    end
    return parse_args(args_array, s)
end


appfolder = dirname(@__FILE__)
app = parse_commandline(ARGS, appfolder)

# Reading the instance file
if app["lilim"]
    data = readLiLimData(app["instance"], app["round"])
else
    data = readRopkeData(app["instance"], app["round"])
end



###########################################################################
main(data, app["restarts"], app["outerIterMax"], app["innerIterMax"], app["seed"])
####################################################################

old = joinpath(dest_dir, "resources.jl")
bak = joinpath(dest_dir, "resources_backup.jl")

if isfile(old)
    rm(old)
end

if isfile(bak)
    mv(bak, old; force=true)
end


old_alg = joinpath(dest_dir, "algorithms.jl")
bak_alg = joinpath(dest_dir, "algorithms_backup.jl")

if isfile(old_alg)
    rm(old_alg)
end

if isfile(bak_alg)
    mv(bak_alg, old_alg; force=true)
end



cd(normpath(joinpath(@__DIR__, "..", "..",".."))) do
    run(`git restore src/resources.jl`)
    run(`git restore src/algorithms.jl`)
end




