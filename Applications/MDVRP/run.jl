using ArgParse
include("model.jl")
include("data.jl")


function parse_commandline(args_array::Vector{String}, appfolder::String)
    s = ArgParseSettings(
        usage="##### Generic ILS #####\n\n" *
              "  On interactive mode, call main([\"arg1\", ..., \"argn\"])", exit_after_help=false)
    @add_arg_table s begin
        "instance"
        help = "Instance file path"

        "--round","-R"
        help = "Does round the distance matrix?"
        action = :store_true

        "--VRPSolver","-V"
        help = "Run the VRPSolver version (multiple graphs)"
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
data = readMDVRPData(app)
###########################################################################
main(data, app["restarts"], app["outerIterMax"], app["innerIterMax"], app["seed"])
####################################################################


