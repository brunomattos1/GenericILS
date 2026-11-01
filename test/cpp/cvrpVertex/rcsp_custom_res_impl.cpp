/**
 * Resource Constrained Shortest Path Solver
 * Designed to be used by Branch-Cut-and-Price algorithms
 *
 * Custom resource of the CVRP written with the vertex step: the arc does nothing and the vertex that enters the
 * label brings its demand. Exceeding the capacity returns infinity (infeasible), otherwise 0.
 */

#include <iostream>
#include <limits>

#include "rcsp_custom_res_impl.hpp"

namespace rcsp_custom_res
{

namespace
{
const double INF = std::numeric_limits<double>::infinity();
const double EPS = 1e-5;
} // namespace

void CustomResParameters::setDimensions(int n, int m)
{
    d.resize(n);
}

void CustomResParameters::setArcParameter(int a, const CustomResArcParameters &value) {}

void CustomResParameters::setVertexParameter(int v, const CustomResVertexParameters &value)
{
    d[v] = value.d;
}

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

double extendAlongArc(const CustomResParameters &Rcc, ForwardState &state, int a)
{
    return 0.0;
}

double extendAlongArc(const CustomResParameters &Rcc, BackwardState &state, int a)
{
    return 0.0;
}

// The vertex that enters the prefix brings its demand.
double extendToVertex(const CustomResParameters &Rcc, ForwardState &state, int v)
{
    state.q += Rcc.d[v];
    return state.q > Rcc.Q + EPS ? INF : 0.0;
}

// The vertex that joins the suffix brings its demand (the suffix does not include the vertex where it starts).
double extendToVertex(const CustomResParameters &Rcc, BackwardState &state, int v)
{
    state.q += Rcc.d[v];
    return state.q > Rcc.Q + EPS ? INF : 0.0;
}

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
