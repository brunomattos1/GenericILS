# Normalized broken-pairs distance between two individuals: the fraction of
# per-client successor/predecessor relationships that differ (0.0 = identical
# route structure, up to 1.0 = completely different). Relies on
# a.successor/a.predecessor (and b's) being up to date -- see
# rebuildSuccessorsPredecessors!.
function brokenPairsDistance(a::Individual, b::Individual)
    n = length(a.successor)
    diff = 0
    for c in 1:n
        a.successor[c] != b.successor[c] && (diff += 1)
        a.predecessor[c] != b.predecessor[c] && (diff += 1)
    end
    return diff / (2n)
end
