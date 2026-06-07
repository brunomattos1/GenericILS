abstract type AbstractSolution
end

mutable struct Solution{FL, BL}
    routes::Vector{Vector{Int}}
    dist::Float64
    cost::Float64
    totalLabelCost::Float64
    labelCosts::Vector{Float64}
    totalInfeas::Int
    infeas::Vector{Int}

    totalWarpStd1::Float64
    warpsStd1::Vector{Float64}
    totalWarpStd2::Float64
    warpsStd2::Vector{Float64}

    feasiblesF::Vector{Int}
    feasiblesB::Vector{Int}
    lastFeasibleF::Vector{Int}
    lastFeasibleB::Vector{Int}
    forwardLabels::Vector{Vector{FL}}
    backwardLabels::Vector{Vector{BL}}
    lastEval::Array{Int, 3}
    lastModif::Vector{Int}
    timeStamp::Int
end

mutable struct UserSolution{FL, BL}
    routes::Vector{Vector{Int}}
    dist::Float64
    cost::Float64
    forwardLabels::Vector{Vector{FL}}
    backwardLabels::Vector{Vector{BL}}
end

function Solution{FL, BL}() where {FL, BL}
    Solution{FL, BL}(
        Vector{Vector{Int}}(),
        0.0,
        0.0,
        0.0,
        Vector{Float64}(),
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
        Vector{Vector{FL}}(),
        Vector{Vector{BL}}(),
        Array{Int,3}(undef, 5, 10, 10),
        Vector{Int}(),
        0
    )
end

function copy_solution!(dest::Solution{FL, BL}, src::Solution{FL, BL}) where {FL, BL}
    dest.dist = src.dist
    dest.cost = src.cost
    dest.totalInfeas = src.totalInfeas
    dest.totalWarpStd1 = src.totalWarpStd1
    dest.totalWarpStd2 = src.totalWarpStd2
    dest.totalLabelCost = src.totalLabelCost

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

    resize!(dest.routes, length(src.routes))
    resize!(dest.forwardLabels, length(src.forwardLabels))
    resize!(dest.backwardLabels, length(src.backwardLabels))
    for i in 1:length(src.routes)
        if !isassigned(dest.routes, i)
            dest.routes[i] = similar(src.routes[i])
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

    if dest.lastEval === nothing || size(dest.lastEval) != size(src.lastEval)
        dest.lastEval = similar(src.lastEval)
    end
    copyto!(dest.lastEval, src.lastEval)

    resize!(dest.lastModif, length(src.lastModif))
    copyto!(dest.lastModif, src.lastModif)

    dest.timeStamp = src.timeStamp
end
