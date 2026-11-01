/**
 * Custom resource for testing, with a state that owns heap memory: a route may not have two customers of the same
 * color (a "set of colors used" kept in a std::vector) and has a capacity. Like the incompatibilities of the
 * VRPTWRDIC resource. The states count how many of them are alive, so the tests can check that they are freed.
 * Following the interface of the MetaSolver.
 */

#ifndef RCSP_CUSTOM_RES_IMPL_H_
#define RCSP_CUSTOM_RES_IMPL_H_

#include <vector>

namespace rcsp_custom_res
{

extern long liveStates;

struct CustomResArcParameters
{
};

struct CustomResVertexParameters
{
    double d;  // demand
    int color; // 0 for none

    CustomResVertexParameters()
    {
        d = 0.0;
        color = 0;
    }

    CustomResVertexParameters(double d_, int color_)
    {
        d = d_;
        color = color_;
    }
};

struct CustomResConstParameters
{
    double Q;    // capacity
    int ncolors; // colors are 1..ncolors

    CustomResConstParameters()
    {
        Q = 0.0;
        ncolors = 0;
    }
};

struct CustomResParameters
{
    std::vector<double> d;
    std::vector<int> color;
    double Q;
    int ncolors;

    void setDimensions(int n, int m);
    void setArcParameter(int a, const CustomResArcParameters &value);
    void setVertexParameter(int v, const CustomResVertexParameters &value);
    void setConstParameter(const CustomResConstParameters &value);
};

struct CustomResSolution
{

    int compare(const CustomResSolution &custom_res_solution) const
    {
        return 0; // no fields to compare
    }
};

// Load and colors used by the prefix. Not trivially copyable: it has a vector and counts its instances.
struct ForwardState
{
    double q;
    std::vector<unsigned char> used;

    ForwardState() : q(0.0) { ++liveStates; }
    ForwardState(const ForwardState &o) : q(o.q), used(o.used) { ++liveStates; }
    ForwardState &operator=(const ForwardState &) = default;
    ~ForwardState() { --liveStates; }
};

// Same for the suffix.
struct BackwardState
{
    double q;
    std::vector<unsigned char> used;

    BackwardState() : q(0.0) { ++liveStates; }
    BackwardState(const BackwardState &o) : q(o.q), used(o.used) { ++liveStates; }
    BackwardState &operator=(const BackwardState &) = default;
    ~BackwardState() { --liveStates; }
};

bool symmetric(const CustomResParameters &Rcc);

void initState(const CustomResParameters &Rcc, ForwardState &state);

void initState(const CustomResParameters &Rcc, BackwardState &state);

double extendToVertex(const CustomResParameters &Rcc, ForwardState &state, int v);

double extendAlongArc(const CustomResParameters &Rcc, ForwardState &state, int a);

double extendToVertex(const CustomResParameters &Rcc, BackwardState &state, int v);

double extendAlongArc(const CustomResParameters &Rcc, BackwardState &state, int a);

double dominationCost(const CustomResParameters &Rcc, int v, const ForwardState &dominating,
                      const ForwardState &dominated);

double dominationCost(const CustomResParameters &Rcc, int v, const BackwardState &dominating,
                      const BackwardState &dominated);

double concatenationCost(const CustomResParameters &Rcc, int v, const ForwardState &fwd,
                         const BackwardState &bwd);

double concatenationCost(const CustomResParameters &Rcc, int v, const ForwardState &fwd,
                         const ForwardState &bwd);

bool isCostResource();

} // namespace rcsp_custom_res

#endif // RCSP_CUSTOM_RES_IMPL_H_
