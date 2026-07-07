mutable struct Parameters
    restarts::Int
    outerIterMax::Int
    innerIterMax::Int
end
function Parameters(;
    restarts = 10,
    outerIterMax = 10,
    innerIterMax = 3)
    return Parameters(restarts, outerIterMax, innerIterMax)
end
