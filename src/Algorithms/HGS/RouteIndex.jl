# customerRoute[c]/customerPos[c]: reverse index (customer id -> current route
# index + position within that route's visits) that the route-centric
# Solution/Route structs don't otherwise provide -- needed by the granular
# RVND! to find "where is candidate V right now" in O(1). Rebuilt fully once
# per generation right after splitGiantTour! (which rebuilds the whole route
# layout anyway), and incrementally for just the touched route(s) after each
# accepted move.
function rebuildRouteIndex!(solver::Solver, algo::HGSAlgorithm)
    sol = algo.ws
    for r in 1:length(sol.routes)
        rebuildRouteIndex!(solver, algo, r)
    end
    return algo
end

function rebuildRouteIndex!(solver::Solver, algo::HGSAlgorithm, r::Int)
    visits = algo.ws.routes[r].visits
    m = length(visits)
    for pos in 2:(m - 1)
        c = visits[pos]
        algo.customerRoute[c] = r
        algo.customerPos[c] = pos
    end
    return algo
end
