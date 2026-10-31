/**
 * Custom resource with cost, for testing: the whole route cost lives in the resource.
 * See the header.
 */

#include <algorithm>

#include "rcsp_custom_res_impl.hpp"

namespace rcsp_custom_res
{

void CustomResParameters::setDimensions(int n, int m)
{
    w.resize(m);
    c.resize(n);
}

void CustomResParameters::setArcParameter(int a, const CustomResArcParameters &value)
{
    w[a] = value.w;
}

void CustomResParameters::setVertexParameter(int v, const CustomResVertexParameters &value)
{
    c[v] = value.c;
}

void CustomResParameters::setConstParameter(const CustomResConstParameters &value) {}

bool symmetric(const CustomResParameters &Rcc)
{
    return false;
}

void initState(const CustomResParameters &Rcc, ForwardState &state)
{
    state.D = 0.0;
}

void initState(const CustomResParameters &Rcc, BackwardState &state)
{
    state.D = 0.0;
}

// The returned value is the cost of the step; the state keeps the accumulated cost.
double extendAlongArc(const CustomResParameters &Rcc, ForwardState &state, int a)
{
    state.D += Rcc.w[a];
    return Rcc.w[a];
}

double extendAlongArc(const CustomResParameters &Rcc, BackwardState &state, int a)
{
    state.D += Rcc.w[a];
    return Rcc.w[a];
}

double extendToVertex(const CustomResParameters &Rcc, ForwardState &state, int v)
{
    state.D += Rcc.c[v];
    return Rcc.c[v];
}

double extendToVertex(const CustomResParameters &Rcc, BackwardState &state, int v)
{
    state.D += Rcc.c[v];
    return Rcc.c[v];
}

double dominationCost(const CustomResParameters &Rcc, int v, const ForwardState &dominating,
                      const ForwardState &dominated)
{
    return dominating.D - dominated.D;
}

double dominationCost(const CustomResParameters &Rcc, int v, const BackwardState &dominating,
                      const BackwardState &dominated)
{
    return dominating.D - dominated.D;
}

// Nothing to add: both halves already hold their cost.
double concatenationCost(const CustomResParameters &Rcc, int v, const ForwardState &fwd,
                         const BackwardState &bwd)
{
    return 0.0;
}

double concatenationCost(const CustomResParameters &Rcc, int v, const ForwardState &fwd,
                         const ForwardState &bwd)
{
    return 0.0;
}

bool isCostResource()
{
    return true;
}

} // namespace rcsp_custom_res
