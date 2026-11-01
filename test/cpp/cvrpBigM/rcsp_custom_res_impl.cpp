/**
 * Resource Constrained Shortest Path Solver
 * Designed to be used by Branch-Cut-and-Price algorithms
 *
 * Custom resource of the CVRP: the vehicle capacity.
 * The same as `cvrp`, but an extension or a concatenation that exceeds the capacity returns 1e12, as the files of the
 * MetaSolver do to forbid it, instead of infinity. Otherwise it returns 0 because the resource adds no cost.
 */

#include <iostream>
#include <limits>

#include "rcsp_custom_res_impl.hpp"

namespace rcsp_custom_res
{

namespace
{
const double INF = 1e12;
const double EPS = 1e-5;
} // namespace

void CustomResParameters::setDimensions(int n, int m)
{
    d.resize(m);
}

void CustomResParameters::setArcParameter(int a, const CustomResArcParameters &value)
{
    d[a] = value.d;
}

void CustomResParameters::setVertexParameter(int v, const CustomResVertexParameters &value) {}

void CustomResParameters::setConstParameter(const CustomResConstParameters &value)
{
    Q = value.Q;
}

bool symmetric(const CustomResParameters &Rcc)
{
    return false;
}

void initState(const CustomResParameters &Rcc, ForwardState &state)
{
    state.q = 0.0;
}

void initState(const CustomResParameters &Rcc, BackwardState &state)
{
    state.q = 0.0;
}

double extendToVertex(const CustomResParameters &Rcc, ForwardState &state, int v)
{
    return 0.0;
}

// The arc (i, j) brings the demand of j to the prefix.
double extendAlongArc(const CustomResParameters &Rcc, ForwardState &state, int a)
{
    state.q += Rcc.d[a];
    return state.q > Rcc.Q + EPS ? INF : 0.0;
}

double extendToVertex(const CustomResParameters &Rcc, BackwardState &state, int v)
{
    return 0.0;
}

// The suffix does not include its first vertex, so prepending the arc (i, j) adds the demand of j.
double extendAlongArc(const CustomResParameters &Rcc, BackwardState &state, int a)
{
    state.q += Rcc.d[a];
    return state.q > Rcc.Q + EPS ? INF : 0.0;
}

// A prefix with less accumulated demand dominates one with more; otherwise it cannot dominate.
double dominationCost(const CustomResParameters &Rcc, int v, const ForwardState &dominating,
                      const ForwardState &dominated)
{
    return dominating.q <= dominated.q ? 0.0 : INF;
}

double dominationCost(const CustomResParameters &Rcc, int v, const BackwardState &dominating,
                      const BackwardState &dominated)
{
    return dominating.q <= dominated.q ? 0.0 : INF;
}

// The prefix includes the junction vertex v and the suffix excludes it, so its demand is counted once.
double concatenationCost(const CustomResParameters &Rcc, int v, const ForwardState &fwd,
                         const BackwardState &bwd)
{
    return fwd.q + bwd.q > Rcc.Q + EPS ? INF : 0.0;
}

double concatenationCost(const CustomResParameters &Rcc, int v, const ForwardState &fwd,
                         const ForwardState &bwd)
{
    return 0.0;
}

bool isCostResource()
{
    return false;
}

} // namespace rcsp_custom_res
