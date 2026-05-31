abstract type AbstractSolution
end

mutable struct Solution
    routes::Vector{Vector{Int}} # routes of the solution
    dist::Float64 # total distance
    cost::Float64 # total cost
    totalLabelCost::Float64
    labelCosts::Vector{Float64}
    totalInfeas::Int
    infeas::Vector{Int}

    totalWarpStd1::Float64
    warpsStd1::Vector{Float64}
    totalWarpStd2::Float64
    warpsStd2::Vector{Float64}

    feasiblesF::Vector{Int} # number of feasible customers per route forward sense
    feasiblesB::Vector{Int} # number of feasible customers per route backward sense
    lastFeasibleF::Vector{Int} # last feasible position for each route forward sense
    lastFeasibleB::Vector{Int} # last feasible position for each route backward sense
    forwardLabels::Vector{Vector{ForwardLabel}}
    backwardLabels::Vector{Vector{BackwardLabel}}
    lastEval::Array{Int, 3}#Vector{Vector{Vector{Int}}}#Dict{Tuple{Symbol, Int, Int}, Int}
    lastModif::Vector{Int}
end

mutable struct UserSolution
    routes::Vector{Vector{Int}} # routes of the solution
    dist::Float64 # total distance
    cost::Float64 # total cost
    forwardLabels::Vector{Vector{ForwardLabel}}
    backwardLabels::Vector{Vector{BackwardLabel}}
end

# Solution() = Solution(Vector{Vector{Int}}(), 0.0, 0.0, 0, Vector{Int}(), 0.0, Vector{Float64}(), Vector{Int}(), Vector{Int}(), Vector{Int}(), Vector{Int}(), Vector{ForwardLabel}[], Vector{BackwardLabel}[], Dict{Tuple{Symbol, Int, Int}, Int}(), Vector{Int}())
# Solution() = Solution(Vector{Vector{Int}}(), 0.0, 0.0, 0, Vector{Int}(), 0.0, Vector{Float64}(), Vector{Int}(), Vector{Int}(), Vector{Int}(), Vector{Int}(), Vector{ForwardLabel}[], Vector{BackwardLabel}[], Vector{Vector{Vector{Int}}}(), Vector{Int}())
Solution() = Solution(Vector{Vector{Int}}(),
                0.0, 
                0.0, 
                0.0, 
                Vector{Int}(), 
                0, 
                Vector{Int}(), 
                0.0, 
                Vector{Float64}(), 
                0.0, 
                Vector{Float64}(), 
                Vector{Int}(), 
                Vector{Int}(),
                Vector{Int}(), 
                Vector{Int}(), 
                Vector{ForwardLabel}[], 
                Vector{BackwardLabel}[], 
                Array{Int,3}(undef, 5, 10, 10),
                Vector{Int}())


function copy_solution!(dest::Solution, src::Solution)
    # Copy scalar fields
    dest.dist = src.dist
    dest.cost = src.cost
    dest.totalInfeas = src.totalInfeas
    dest.totalWarpStd1 = src.totalWarpStd1
    dest.totalWarpStd2 = src.totalWarpStd2
    dest.totalLabelCost = src.totalLabelCost
    
    # Copy vector fields using resize! and copyto!
    resize!(dest.infeas, length(src.infeas))
    copyto!(dest.infeas, src.infeas)

    resize!(dest.warpsStd1, length(src.warpsStd1))
    copyto!(dest.warpsStd1, src.warpsStd1)

    resize!(dest.warpsStd2, length(src.warpsStd2))
    copyto!(dest.warpsStd2, src.warpsStd2)

    resize!(dest.labelCosts, length(src.infeas))
    copyto!(dest.labelCosts, src.labelCosts)

    resize!(dest.feasiblesF, length(src.feasiblesF))
    copyto!(dest.feasiblesF, src.feasiblesF)

    resize!(dest.feasiblesB, length(src.feasiblesB))
    copyto!(dest.feasiblesB, src.feasiblesB)

    resize!(dest.lastFeasibleF, length(src.lastFeasibleF))
    copyto!(dest.lastFeasibleF, src.lastFeasibleF)

    resize!(dest.lastFeasibleB, length(src.lastFeasibleB))
    copyto!(dest.lastFeasibleB, src.lastFeasibleB)

    # Copy nested vectors (routes)
    resize!(dest.routes, length(src.routes))
    resize!(dest.forwardLabels, length(src.forwardLabels))
    resize!(dest.backwardLabels, length(src.backwardLabels))
    for i in 1:length(src.routes)
        # Check if the inner vector is undefined or has a different size
        if !isassigned(dest.routes, i)
            dest.routes[i] = similar(src.routes[i])  # aloca uma vez só
        end
        resize!(dest.routes[i], length(src.routes[i]))
        copyto!(dest.routes[i], src.routes[i])

        if !isassigned(dest.forwardLabels, i)
            dest.forwardLabels[i] = similar(src.forwardLabels[i])
        end
        resize!(dest.forwardLabels[i], length(src.forwardLabels[i]))
        copyto!(dest.forwardLabels[i], src.forwardLabels[i])

        if !isassigned(dest.backwardLabels, i)
            dest.backwardLabels[i] = similar(src.backwardLabels[i])
        end
        resize!(dest.backwardLabels[i], length(src.backwardLabels[i]))
        copyto!(dest.backwardLabels[i], src.backwardLabels[i])
    end

    # if dest.lastEval === nothing
    #     dest.lastEval = Dict{Tuple,Int}()
    # else
    #     empty!(dest.lastEval)
    # end
    # for (k,v) in src.lastEval
    #     dest.lastEval[k] = v
    # end
    if dest.lastEval === nothing || size(dest.lastEval) != size(src.lastEval)
        dest.lastEval = similar(src.lastEval)  # cria novo array do mesmo tamanho
    end
    copyto!(dest.lastEval, src.lastEval)

    resize!(dest.lastModif, length(src.lastModif))
    copyto!(dest.lastModif, src.lastModif)

    # Copy nested vectors of labels (forwardLabels, backwardLabels)
    # resize!(dest.forwardLabels, length(src.forwardLabels))
    # resize!(dest.backwardLabels, length(src.backwardLabels))

    # for i in 1:length(src.forwardLabels)
    #     # Check if the inner vector is undefined or needs resizing
    #     if !isassigned(dest.forwardLabels, i)
    #         dest.forwardLabels[i] = similar(src.forwardLabels[i])
    #     end
    #     resize!(dest.forwardLabels[i], length(src.forwardLabels[i]))
    #     copyto!(dest.forwardLabels[i], src.forwardLabels[i])

    #     if !isassigned(dest.backwardLabels, i)
    #         dest.backwardLabels[i] = similar(src.backwardLabels[i])
    #     end
    #     resize!(dest.backwardLabels[i], length(src.backwardLabels[i]))
    #     copyto!(dest.backwardLabels[i], src.backwardLabels[i])
    # end

    # resize!(dest.backwardLabels, length(src.backwardLabels))
    # for i in 1:length(src.backwardLabels)
    #     if !isassigned(dest.forwardLabels, i)
    #         dest.forwardLabels[i] = similar(src.forwardLabels[i])
    #     end
    #     resize!(dest.forwardLabels[i], length(src.forwardLabels[i]))
    #     copyto!(dest.forwardLabels[i], src.forwardLabels[i])
    # end
end
