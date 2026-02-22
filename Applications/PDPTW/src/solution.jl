include("../../../src/include.jl")
function printSol(sol::Solution)
    R = sol.routes
    for k=1:length(R)
        print("route $(k) : ")
        for i ∈ R[k]
            print("$i ")
        end
        println()
    end
end
