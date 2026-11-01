# Custom resources written in C++

A custom resource can be written in C++, in the format of the **MetaSolver** of VRPSolver (the
`rcsp_custom_res_impl.hpp` / `rcsp_custom_res_impl.cpp` pair), instead of in Julia. The same files that define the
resource of a VRPSolver model then define the resource of the heuristic. With the
[adapter](#from-a-vrpsolver-model) the whole heuristic is derived from the VRPSolver model (see the
[complete example](#complete-example-ccvrp)).

```julia
using GenericILS

res = loadCppResource("rcsp_custom_res_impl.cpp", "rcsp_custom_res_impl.hpp";
    nv = n + 1,                                          # vertices of the matrices, depot included
    arcParams = (i, j) -> (d = demand[j],),              # fields of CustomResArcParameters
    constParams = (Q = capacity,))                       # fields of CustomResConstParameters

resources = Resources(res, stdRes1, stdRes2)             # like any custom resource
```

The result is used as the custom resource of `Resources`, and nothing else has to be defined for it: its
`isCostResource` and `isSymmetric` are the ones of the C++ functions of the same name (per resource, so they do not
need the global definitions of the [tutorial](tutorial.md)).

## What it does

`loadCppResource`:

1. compiles the `.cpp` together with a small generic file (`src/cpp/shim.cpp`) into a shared library, with the
   compiler found in `CXX`, or `g++`, `clang++`, `c++`. The library is cached in the temporary directory by the hash of
   the sources, so it is compiled once;
2. decides how each state (`ForwardState`, `BackwardState`) travels in a label, with no action from you:
   - **plain data** (trivially copyable: numbers, `std::bitset`, fixed arrays): the label carries the bytes by value.
     When the fields of the struct can be read from the `.hpp` (`double`/`int`/`bool`... members) it is a Julia struct
     with the same field names, so `printLabels` shows them; otherwise it is an opaque block of bytes;
   - **anything else** (a `std::vector`, a set, a pointer, ...): the state lives in the C++ heap and the label holds a
     pointer that is freed when the label is collected. An extension copies the state with its copy constructor and
     updates the copy, as the hand-written resources do with a `Vector`. The state needs a default and a copy
     constructor;
3. creates the `CustomResParameters` object in C++ and fills it by calling `setDimensions`, `setArcParameter`,
   `setVertexParameter` and `setConstParameter`.

The labels are still the labels of GenericILS. Each step of the search calls the C++ functions to update the state
and the cost.

## How the functions map

| MetaSolver (C++) | GenericILS |
|---|---|
| `initState` | `initStateForward` / `initStateBackward` |
| `extendAlongArc(Rcc, state, a)` | `extendAlongArc`; `a` is the id of the arc |
| `extendToVertex(Rcc, state, v)` | `extendToVertex`, applied right after the arc |
| `concatenationCost` | `concatenationCost` |
| `isCostResource()`, `symmetric()` | `isCostResource(res)`, `isSymmetric(res)` |
| `dominationCost` | not used: the local search has no dominance |

Conventions to write the C++ side:

- **Vertices and arcs.** Vertex `i` of the Julia matrices (depot is 1) is the vertex `i - 1` of the interface. The arc
  `(i, j)` has the id `(i - 1) * nv + (j - 1)`, which is the `a` the functions receive; its parameters are what
  `arcParams(i, j)` returned. The backward extension is also called with the arc in the direction of travel.
- **Costs.** Every function returns the cost of its step, and the label cost is the sum of the steps. `Inf` means
  that the extension or the concatenation is infeasible. A resource that adds no cost returns `0.0`.
- **Forward and backward.** The forward state includes the vertex where it ends and the backward state does not
  include the vertex where it starts. `extendToVertex` is called with the vertex that enters the state: the head of
  the arc going forward, the vertex `next` going backward. The first vertex of a route is never entered. Because of this,
  `concatenationCost` has nothing to add for the vertices.

## Any state, with a cost

A resource can carry any structure in its state, such as the set of visited customers of the VRPTWRDIC resource. The
price is speed. Measured on a forward extension (arc plus vertex) of a "no two customers of the same color in a route"
resource, with a Julia resource of the same state as reference:

| state | Julia | C++ |
|---|---|---|
| plain data (bit mask / `std::bitset`) | 0.7 ns | 5.7 ns |
| heap (`Vector{UInt8}` / `std::vector`) | 31 ns | 199 ns |

The whole heuristic is as fast as the Julia one for plain states (0.95x to 1.3x in the tests) and about 5x slower than
a Julia resource with a bit mask, 2.6x slower than a Julia resource with a `Vector` state, when the C++ state is in the
heap. Allocation and release in C++ is what costs.

Because Julia does not see the memory of the C++ states, their number is counted and a garbage collection is forced
when 200,000 are alive (and again when their number doubles), which runs the finalizers. This keeps memory bounded
in long searches.

## Limits

- **The three parameter structs are plain data.** `CustomResArcParameters`, `CustomResVertexParameters` and
  `CustomResConstParameters` cross as bytes, from Julia, exactly as the MetaSolver passes them (a Julia `isbits` struct by
  value), so they cannot hold a `std::vector`. Data of that kind belongs in `CustomResParameters`, which lives in C++
  and is filled by the `set*Parameter` functions. If the header cannot be read, give an `isbits` Julia value with the
  same layout instead of a `NamedTuple`.
- **Exceptions** thrown by the C++ code are not caught; they end the process.
- **A state needs a default and a copy constructor** when it is not plain data. `ownedStates = true` forces every state
  into the heap (useful to test a resource in both ways).
- The compiler must be installed. Without it `loadCppResource` fails with a message, and the tests that need it are
  skipped.
- The resource owns a C++ object that is released when the resource is garbage collected, or at once by
  `freeCppResource!(res)` (a freed resource cannot be used again). The states of the labels are separate objects
  with their own release. The shared library stays loaded until the process ends.
- The data of the arcs and vertices come from Julia callbacks, or from a VRPSolver model with the adapter below.

The examples used by the tests are in [`test/cpp`](../test/cpp): the CVRP capacity written in the arc step and in
the vertex step, a resource with cost, and two resources with a set of colors in the state (a `std::bitset` and a
`std::vector`).

## From a VRPSolver model

`heuristicModelFromVRPSolver` builds the data of the heuristic from the `VrpGraph` of a VRPSolver model, so the arcs,
costs and resources do not have to be written twice. It needs VrpSolver.jl (loaded next to GenericILS, it is not a
dependency of the package):

```julia
using GenericILS, VrpSolver

h = heuristicModelFromVRPSolver(model; cppDir = "src/meta_solver")   # the folder with rcsp_custom_res_impl.{hpp,cpp}
solver = Solver(res = h.res, data = h.data, neighborhoods = (TwoOptStar(), IntraShift()),
    algorithm = NILSAlgorithm(h.res))
```

It returns `data` (`ProblemData`), `res` (`Resources`), `maxNbRoutes` and `userIds` (`userIds[i + 1]` is the id in the
model of vertex `i`; the depot is vertex 0 and the customers follow in the order of the graph). The model does not store
the path of the C++ files, so `cppDir` says where they are.

| In the VRPSolver model | In the heuristic |
|---|---|
| the custom resource | `loadCppResource` with the parameters stored in the model: the arc, vertex and constant structs are passed with their layout, which is the order of the fields of the C++ struct, whatever names the model used to build them |
| standard resources (up to two) | `StandardResource`: arc consumption, and the interval of the arcs that enter each vertex |
| a cost variable on the custom resource | the cost is the cost of the label (zero cost matrix); `isCostResource()` of the C++ must agree |
| no cost variable | the cost of the arc is the sum, over its mapped variables, of the coefficient in the objective |
| packing sets | groups that need one visit: a customer alone in its set is an ordinary customer, a set with several customers means one of them |
| multiplicity `(L, U)` | `maxNbRoutes = U`; `L` is not used |

What has no equivalent is an `ArgumentError`: more than one graph, a source different from the sink, a graph that is
not complete or has parallel arcs, binary or non-disposable resources, more than two standard resources or other than one
custom resource, bounds that depend on the arc, a vertex outside the packing sets, and an objective that is neither the
cost variable of the custom resource nor made of variables mapped to arcs (a mix of both is not supported yet). Master
constraints, cuts and branching are not read: every packing set is assumed to need exactly one visit.

## Complete example: CCVRP

[`Applications/CCVRP/src/mainCCVRP_cpp_full.jl`](../Applications/CCVRP/src/mainCCVRP_cpp_full.jl) is the example from
end to end. It writes the VRPSolver model of the CCVRP (the one of VRPSolverDemos: the edge variables `x`, the cost
variable `z` of the custom resource, the capacity resource, the packing sets and the cuts of the BCP) and solves it
with the heuristic. The model is the only description of the problem; the heuristic is built with
`heuristicModelFromVRPSolver(model; cppDir)` from the graph and the C++ files of `Applications/CCVRP/src/meta_solver`.
The BaPCod library is not needed: the model is only read, never optimized.

After the search the example checks the solution against the model: the values of `x` that the routes define give
degree 2 to every customer, and the cost of the heuristic is the cost of the routes by definition (the sum, over the
customers, of the demand times the time of service).

The application environment has `VrpSolver` (from its GitHub repository, through `[sources]`) and `JuMP`. From
`Applications/CCVRP/src`:

```
julia --project=.. mainCCVRP_cpp_full.jl
```

This is not the same problem as `mainCCVRP.jl` and `mainCCVRP_cpp.jl`, which give weight 1 to every customer: the
model of the demo gives the weight of the demand, so their costs are not comparable.

## Tests of the adapter

`test/testVrpSolverAdapter.jl` builds small VRPSolver models and compares the data and the solution of the heuristic
with the same problem built by hand, and checks every error of the adapter. It needs VrpSolver.jl in the test
environment and a C++ compiler; `test/runtests.jl` skips it when either is missing, since VrpSolver.jl is not a
dependency of the package.
