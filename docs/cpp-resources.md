# Custom resources written in C++

A custom resource can be written in C++, in the format of the **MetaSolver** of VRPSolver (the
`rcsp_custom_res_impl.hpp` / `rcsp_custom_res_impl.cpp` pair), instead of in Julia. The same files that define the
resource of a VRPSolver model then define the resource of the heuristic. This is the first step towards solving a
VRPSolver model with GenericILS.

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
- The data of the arcs and vertices come from Julia callbacks. They are not read from a VRPSolver model yet.

The examples used by the tests are in [`test/cpp`](../test/cpp): the CVRP capacity written in the arc step and in
the vertex step, a resource with cost, and two resources with a set of colors in the state (a `std::bitset` and a
`std::vector`).
