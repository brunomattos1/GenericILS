# Classic Order Crossover (OX) on the giant tour: copies a random contiguous
# slice from parent1 verbatim, fills the rest (starting right after the
# slice, wrapping around) with parent2's clients in their relative order.
# Always produces a valid permutation of 1:n. Operates purely on
# giantTour::Vector{Int}, no dependency on labels/resources.
function orderCrossover!(child::Individual, parent1::Individual, parent2::Individual,
    visited::BitVector, freeSlots::Vector{Int}, seed)

    n = length(parent1.giantTour)
    fill!(visited, false)

    i = rand(seed, 1:n)
    j = rand(seed, 1:n)
    i, j = i <= j ? (i, j) : (j, i)

    childTour = child.giantTour
    for pos in i:j
        c = parent1.giantTour[pos]
        childTour[pos] = c
        visited[c] = true
    end

    nFree = n - (j - i + 1)
    length(freeSlots) >= nFree || resize!(freeSlots, nFree)
    slotCount = 0
    for offset in 1:n
        pos = j + offset
        pos > n && (pos -= n)
        (pos >= i && pos <= j) && continue
        slotCount += 1
        freeSlots[slotCount] = pos
    end

    cursor = 1
    for pos in 1:n
        c = parent2.giantTour[pos]
        visited[c] && continue
        childTour[freeSlots[cursor]] = c
        visited[c] = true
        cursor += 1
    end

    return child
end
