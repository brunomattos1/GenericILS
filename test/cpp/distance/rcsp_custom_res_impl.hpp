/**
 * Custom resource with cost, for testing: the whole route cost lives in the resource.
 * Every arc costs its distance (arc parameter) and every vertex that enters a label costs a fixed amount (vertex
 * parameter), which is paid by extendToVertex. The route cost is the sum of the arcs and of the vertices entered,
 * the first vertex of the route excluded. Following the interface of the MetaSolver.
 */

#ifndef RCSP_CUSTOM_RES_IMPL_H_
#define RCSP_CUSTOM_RES_IMPL_H_

#include <vector>

namespace rcsp_custom_res
{

struct CustomResArcParameters
{
    double w; // distance

    CustomResArcParameters() { w = 0.0; }

    CustomResArcParameters(double w_) { w = w_; }
};

struct CustomResVertexParameters
{
    double c; // cost of entering the vertex

    CustomResVertexParameters() { c = 0.0; }

    CustomResVertexParameters(double c_) { c = c_; }
};

struct CustomResConstParameters
{
};

struct CustomResParameters
{
    std::vector<double> w;
    std::vector<double> c;

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

// Cost accumulated by the prefix.
struct ForwardState
{
    double D;
};

// Cost accumulated by the suffix.
struct BackwardState
{
    double D;
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
