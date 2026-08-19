# HGS-CVRP-style mu+lambda population: two independently ranked subpopulations
# (feasible/infeasible), each allowed to grow to muMax+lambda before a burst
# cull prunes it back to muMax. Ported from the reference HGS-VRP
# implementation (which is already representation-agnostic -- it operates on
# giant tours/routes, not on AbstractSegment). Individual carries no labels,
# so neither Subpopulation nor Population need to be parametrized by FL/BL.

mutable struct Subpopulation
    individuals::Vector{Individual}
    slotOf::Vector{Int}
    individualAtSlot::Vector{Int}
    validSlots::Vector{Int}
    freeSlots::Vector{Int}
    distances::Matrix{Float64}
    costRank::Vector{Int}
    diversityRank::Vector{Int}
    order::Vector{Int}
    diversities::Vector{Float64}
    distBuffer::Vector{Float64}
end

function Subpopulation(muMax::Int, lambda::Int)
    capacity = muMax + lambda + 1
    return Subpopulation(
        Individual[],
        Int[],
        zeros(Int, capacity),
        Int[],
        collect(capacity:-1:1),
        Matrix{Float64}(undef, capacity, capacity),
        Int[],
        Int[],
        Int[],
        Vector{Float64}(undef, capacity),
        Vector{Float64}(undef, capacity)
    )
end

Base.length(sp::Subpopulation) = length(sp.individuals)

function clear!(sp::Subpopulation)
    empty!(sp.individuals)
    empty!(sp.slotOf)
    capacity = length(sp.individualAtSlot)
    resize!(sp.freeSlots, capacity)
    for i in 1:capacity
        sp.freeSlots[i] = capacity - i + 1
    end
    empty!(sp.validSlots)
    empty!(sp.costRank)
    empty!(sp.diversityRank)
    empty!(sp.order)
    return sp
end

function insertIndividual!(sp::Subpopulation, ind::Individual)
    slot = pop!(sp.freeSlots)
    push!(sp.individuals, ind)
    push!(sp.slotOf, slot)
    idx = length(sp.individuals)
    sp.individualAtSlot[slot] = idx

    for otherSlot in sp.validSlots
        other = sp.individuals[sp.individualAtSlot[otherSlot]]
        d = brokenPairsDistance(ind, other)
        sp.distances[slot, otherSlot] = d
        sp.distances[otherSlot, slot] = d
    end
    push!(sp.validSlots, slot)
    return sp
end

function removeAt!(sp::Subpopulation, idx::Int)
    slot = sp.slotOf[idx]
    deleteat!(sp.individuals, idx)
    deleteat!(sp.slotOf, idx)
    filter!(!=(slot), sp.validSlots)
    push!(sp.freeSlots, slot)

    for i in idx:length(sp.individuals)
        sp.individualAtSlot[sp.slotOf[i]] = i
    end
    return sp
end

function averageDistanceToClosest(sp::Subpopulation, idx::Int, nClosest::Int)
    slot = sp.slotOf[idx]
    m = length(sp.validSlots)
    nClosest = min(nClosest, m - 1)
    nClosest <= 0 && return 0.0

    dists = sp.distBuffer
    c = 0
    for otherSlot in sp.validSlots
        otherSlot == slot && continue
        c += 1
        dists[c] = sp.distances[slot, otherSlot]
    end
    total = 0.0
    for _ in 1:nClosest
        best = Inf
        bestPos = 0
        for i in 1:(m - 1)
            if dists[i] < best
                best = dists[i]
                bestPos = i
            end
        end
        dists[bestPos] = Inf
        total += best
    end
    return total / nClosest
end

function updateBiasedFitness!(sp::Subpopulation, nClosest::Int, nElite::Int)
    m = length(sp.individuals)
    m == 0 && return sp

    resize!(sp.costRank, m)
    resize!(sp.diversityRank, m)
    resize!(sp.order, m)
    for i in 1:m
        sp.order[i] = i
    end

    sort!(sp.order; by = i -> sp.individuals[i].cost)
    for (rank, i) in enumerate(sp.order)
        sp.costRank[i] = rank
    end

    diversities = sp.diversities
    for i in 1:m
        diversities[i] = averageDistanceToClosest(sp, i, nClosest)
    end
    sort!(sp.order; by = i -> -diversities[i])
    for (rank, i) in enumerate(sp.order)
        sp.diversityRank[i] = rank
    end

    diversityWeight = 1.0 - min(nElite, m) / m
    for i in 1:m
        sp.individuals[i].biasedFitness = sp.costRank[i] + diversityWeight * sp.diversityRank[i]
    end
    return sp
end

const CLONE_EPSILON = 1e-5

function worstBiasedFitnessIndex(sp::Subpopulation)
    m = length(sp.individuals)
    m == 0 && return 0
    worst = 1
    worstIsClone = averageDistanceToClosest(sp, 1, 1) < CLONE_EPSILON
    for i in 2:m
        isClone = averageDistanceToClosest(sp, i, 1) < CLONE_EPSILON
        if (isClone && !worstIsClone) ||
           (isClone == worstIsClone && sp.individuals[i].biasedFitness > sp.individuals[worst].biasedFitness)
            worst = i
            worstIsClone = isClone
        end
    end
    return worst
end

mutable struct Population
    feasible::Subpopulation
    infeasible::Subpopulation
    muMax::Int
    lambda::Int
    nClosest::Int
    nElite::Int
    # Individuals evicted by a burst cull or a restart! are pushed here
    # instead of being discarded, so the next individual needed (see
    # getPooledIndividual!) can reuse its already-allocated giantTour/routes/
    # successor/predecessor buffers via storeIndividual!'s resize!+copyto!
    # in-place pattern, instead of the whole object graph being freshly
    # allocated (and the old one left for the GC) every single generation.
    pool::Vector{Individual}
end

function Population(muMax::Int, lambda::Int; nClosest::Int = 5, nElite::Int = 4)
    return Population(
        Subpopulation(muMax, lambda),
        Subpopulation(muMax, lambda),
        muMax, lambda, nClosest, nElite,
        Individual[]
    )
end

# Returns a recycled Individual from pop's pool if one is available, else
# allocates a fresh one. The caller is expected to fully overwrite it
# (giantTour + storeIndividual!) before it is inserted anywhere.
function getPooledIndividual!(pop::Population, solver::Solver)
    isempty(pop.pool) && return Individual(solver)
    return pop!(pop.pool)
end

function restart!(pop::Population)
    append!(pop.pool, pop.feasible.individuals)
    append!(pop.pool, pop.infeasible.individuals)
    clear!(pop.feasible)
    clear!(pop.infeasible)
    return pop
end

function addIndividual!(pop::Population, ind::Individual)
    sp = isFeasible(ind) ? pop.feasible : pop.infeasible
    insertIndividual!(sp, ind)
    updateBiasedFitness!(sp, pop.nClosest, pop.nElite)
    if length(sp) > pop.muMax + pop.lambda
        while length(sp) > pop.muMax
            idx = worstBiasedFitnessIndex(sp)
            push!(pop.pool, sp.individuals[idx])
            removeAt!(sp, idx)
            updateBiasedFitness!(sp, pop.nClosest, pop.nElite)
        end
    end
    return pop
end
