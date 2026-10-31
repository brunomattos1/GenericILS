/**
 * Custom resource for testing, with a state that owns heap memory. See the header.
 */

#include <limits>

#include "rcsp_custom_res_impl.hpp"

namespace rcsp_custom_res
{

long liveStates = 0;

namespace
{
const double INF = std::numeric_limits<double>::infinity();
const double EPS = 1e-5;

// The vertex v enters a state: its demand is added and its color is marked; infeasible if the load is over the
// capacity or the color was already there.
template <class State>
double enter(const CustomResParameters &Rcc, State &state, int v)
{
    state.q += Rcc.d[v];
    bool bad = state.q > Rcc.Q + EPS;
    int c = Rcc.color[v];
    if (c > 0)
    {
        if (state.used[c])
            bad = true;
        state.used[c] = 1;
    }
    return bad ? INF : 0.0;
}
} // namespace

void CustomResParameters::setDimensions(int n, int m)
{
    d.resize(n);
    color.resize(n);
}

void CustomResParameters::setArcParameter(int a, const CustomResArcParameters &value) {}

void CustomResParameters::setVertexParameter(int v, const CustomResVertexParameters &value)
{
    d[v] = value.d;
    color[v] = value.color;
}

void CustomResParameters::setConstParameter(const CustomResConstParameters &value)
{
    Q = value.Q;
    ncolors = value.ncolors;
}

bool symmetric(const CustomResParameters &Rcc)
{
    return false;
}

void initState(const CustomResParameters &Rcc, ForwardState &state)
{
    state.q = 0.0;
    state.used.assign(Rcc.ncolors + 1, 0);
}

void initState(const CustomResParameters &Rcc, BackwardState &state)
{
    state.q = 0.0;
    state.used.assign(Rcc.ncolors + 1, 0);
}

double extendAlongArc(const CustomResParameters &Rcc, ForwardState &state, int a)
{
    return 0.0;
}

double extendAlongArc(const CustomResParameters &Rcc, BackwardState &state, int a)
{
    return 0.0;
}

double extendToVertex(const CustomResParameters &Rcc, ForwardState &state, int v)
{
    return enter(Rcc, state, v);
}

double extendToVertex(const CustomResParameters &Rcc, BackwardState &state, int v)
{
    return enter(Rcc, state, v);
}

double dominationCost(const CustomResParameters &Rcc, int v, const ForwardState &dominating,
                      const ForwardState &dominated)
{
    return 0.0;
}

double dominationCost(const CustomResParameters &Rcc, int v, const BackwardState &dominating,
                      const BackwardState &dominated)
{
    return 0.0;
}

// The two halves are infeasible together if they exceed the capacity or use a color in common.
double concatenationCost(const CustomResParameters &Rcc, int v, const ForwardState &fwd,
                         const BackwardState &bwd)
{
    if (fwd.q + bwd.q > Rcc.Q + EPS)
        return INF;
    for (int c = 1; c <= Rcc.ncolors; ++c)
        if (fwd.used[c] && bwd.used[c])
            return INF;
    return 0.0;
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

// How many states are alive, for the tests.
#ifdef _WIN32
extern "C" __declspec(dllexport) long colors_liveStates() { return rcsp_custom_res::liveStates; }
#else
extern "C" __attribute__((visibility("default"))) long colors_liveStates() { return rcsp_custom_res::liveStates; }
#endif
