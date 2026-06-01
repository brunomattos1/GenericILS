
# IterationLimit — para após N iterações consecutivas sem melhora
mutable struct IterationLimit
    maxIter::Int
    iter::Int
    IterationLimit(n::Int) = new(n, 0)
end

stop!(c::IterationLimit)     = c.iter >= c.maxIter
tick!(c::IterationLimit)     = (c.iter += 1; nothing)
reset!(c::IterationLimit)    = (c.iter  = 0; nothing)
improved!(c::IterationLimit) = (c.iter  = 0; nothing)
