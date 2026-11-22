/**
 * Resource Constrained Shortest Path Solver
 * Designed to be used by Branch-Cut-and-Price algorithms
 *
 * Custom resource of the CVRP written with the vertex step: the demand is a parameter of the vertex, the arcs have
 * none, and the capacity is checked by extendToVertex.
 * Following the interface of the MetaSolver (see VRPSolverDemos/CCVRP/src/meta_solver).
 */

#ifndef RCSP_CUSTOM_RES_IMPL_H_
#define RCSP_CUSTOM_RES_IMPL_H_

#include <vector>

namespace rcsp_custom_res
{

struct CustomResArcParameters
{
};

// Demand of the vertex.
struct CustomResVertexParameters
{
    double d;

    CustomResVertexParameters() { d = 0.0; }

    CustomResVertexParameters(double d_) { d = d_; }
};

// Vehicle capacity.
struct CustomResConstParameters
{
    double Q;

    CustomResConstParameters(double q) { Q = q; }
    CustomResConstParameters() { Q = 0.0; }
};

struct CustomResParameters
{
    std::vector<double> d;
    double Q;

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

// Demand accumulated from the source to the current vertex (included).
struct ForwardState
{
    double q;
};

// Demand accumulated from the sink back to the current vertex (excluded).
struct BackwardState
{
    double q;
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
