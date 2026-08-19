abstract type AbstractSelection end

struct BinaryTournament <: AbstractSelection end

function select(::BinaryTournament, pop::Population, seed)
    return binaryTournament(pop, seed)
end

function binaryTournament(pop::Population, seed)
    nFeas = length(pop.feasible)
    nInfeas = length(pop.infeasible)
    total = nFeas + nInfeas
    total > 0 || error("Cannot select from an empty population")

    a = pickRandomIndividual(pop, seed, nFeas, total)
    b = pickRandomIndividual(pop, seed, nFeas, total)
    return a.biasedFitness <= b.biasedFitness ? a : b
end

function pickRandomIndividual(pop::Population, seed, nFeas::Int, total::Int)
    r = rand(seed, 1:total)
    return r <= nFeas ? pop.feasible.individuals[r] : pop.infeasible.individuals[r - nFeas]
end
