// Generic C interface over the custom resource of a rcsp_custom_res_impl.hpp/.cpp pair.
//
// The MetaSolver interface is C++ (namespace, overloads, std::vector inside CustomResParameters), which Julia
// cannot call directly. This file does not depend on the problem: it only uses the names that the interface
// prescribes, so it is compiled together with any user implementation.
//
// A state (ForwardState/BackwardState) reaches Julia in one of two ways, chosen per type by rcc_isPlain:
//   - plain data (trivially copyable): its bytes are copied into the Julia label, and the functions below work on a
//     pointer to that copy;
//   - anything else (a std::vector, a set, ...): the state lives in the C++ heap. Julia only holds a pointer that it
//     frees when the label is collected; an extension makes a copy with rcc_copy* and updates the copy.
// The three parameter structs (arc, vertex, constants) always cross as bytes, as the MetaSolver does.

#include <cstddef>
#include <cstring>
#include <type_traits>

#include "rcsp_custom_res_impl.hpp"

using namespace rcsp_custom_res;

#ifdef _WIN32
#define EXPORT extern "C" __declspec(dllexport)
#else
#define EXPORT extern "C" __attribute__((visibility("default")))
#endif

static CustomResParameters &params(void *p)
{
    return *static_cast<CustomResParameters *>(p);
}

// Copies at most sizeof(T) bytes: a struct without fields is empty in C++ (1 byte) but has size 0 in Julia.
template <class T>
static T fromBytes(const void *bytes, int n)
{
    T value;
    std::memcpy(&value, bytes, n < (int)sizeof(T) ? (size_t)n : sizeof(T));
    return value;
}

template <class T>
static int isPlainType()
{
    return std::is_trivially_copyable<T>::value && alignof(T) <= 8 ? 1 : 0;
}

EXPORT void *rcc_new() { return new CustomResParameters(); }

EXPORT void rcc_free(void *p) { delete static_cast<CustomResParameters *>(p); }

// Types: 0 forward state, 1 backward state, 2 arc parameters, 3 vertex parameters, 4 constants.
EXPORT int rcc_sizeof(int which)
{
    switch (which)
    {
        case 0: return (int)sizeof(ForwardState);
        case 1: return (int)sizeof(BackwardState);
        case 2: return (int)sizeof(CustomResArcParameters);
        case 3: return (int)sizeof(CustomResVertexParameters);
        default: return (int)sizeof(CustomResConstParameters);
    }
}

// Whether the type can be copied byte by byte (and needs no more alignment than Julia gives).
EXPORT int rcc_isPlain(int which)
{
    switch (which)
    {
        case 0: return isPlainType<ForwardState>();
        case 1: return isPlainType<BackwardState>();
        case 2: return isPlainType<CustomResArcParameters>();
        case 3: return isPlainType<CustomResVertexParameters>();
        default: return isPlainType<CustomResConstParameters>();
    }
}

EXPORT void rcc_setDimensions(void *p, int n, int m) { params(p).setDimensions(n, m); }

EXPORT void rcc_setArc(void *p, int a, const void *bytes, int n)
{
    params(p).setArcParameter(a, fromBytes<CustomResArcParameters>(bytes, n));
}

EXPORT void rcc_setVertex(void *p, int v, const void *bytes, int n)
{
    params(p).setVertexParameter(v, fromBytes<CustomResVertexParameters>(bytes, n));
}

EXPORT void rcc_setConst(void *p, const void *bytes, int n)
{
    params(p).setConstParameter(fromBytes<CustomResConstParameters>(bytes, n));
}

EXPORT int rcc_symmetric(void *p) { return symmetric(params(p)) ? 1 : 0; }

EXPORT int rcc_isCostResource() { return isCostResource() ? 1 : 0; }

// ---- states in the C++ heap (the general case) ----

// A new state, initialized by initState; null if the type cannot be default constructed.
template <class S>
static void *makeState(const CustomResParameters &p)
{
    if constexpr (std::is_default_constructible<S>::value)
    {
        S *s = new S();
        initState(p, *s);
        return s;
    }
    else
    {
        return nullptr;
    }
}

// A copy of a state; null if the type cannot be copied.
template <class S>
static void *copyState(const void *src)
{
    if constexpr (std::is_copy_constructible<S>::value)
    {
        return new S(*static_cast<const S *>(src));
    }
    else
    {
        return nullptr;
    }
}

EXPORT void *rcc_makeFwd(void *p) { return makeState<ForwardState>(params(p)); }

EXPORT void *rcc_makeBwd(void *p) { return makeState<BackwardState>(params(p)); }

EXPORT void *rcc_copyFwd(const void *s) { return copyState<ForwardState>(s); }

EXPORT void *rcc_copyBwd(const void *s) { return copyState<BackwardState>(s); }

EXPORT void rcc_freeFwd(void *s) { delete static_cast<ForwardState *>(s); }

EXPORT void rcc_freeBwd(void *s) { delete static_cast<BackwardState *>(s); }

// ---- the functions of the interface, on a pointer to a state ----

EXPORT void rcc_initFwd(void *p, void *state) { initState(params(p), *static_cast<ForwardState *>(state)); }

EXPORT void rcc_initBwd(void *p, void *state) { initState(params(p), *static_cast<BackwardState *>(state)); }

// Extension along the arc `a` (in the direction of travel); the vertex that enters the label is extended afterwards
// with the functions below.
EXPORT double rcc_extendFwd(void *p, void *state, int a)
{
    return extendAlongArc(params(p), *static_cast<ForwardState *>(state), a);
}

EXPORT double rcc_extendBwd(void *p, void *state, int a)
{
    return extendAlongArc(params(p), *static_cast<BackwardState *>(state), a);
}

EXPORT double rcc_vertexFwd(void *p, void *state, int v)
{
    return extendToVertex(params(p), *static_cast<ForwardState *>(state), v);
}

EXPORT double rcc_vertexBwd(void *p, void *state, int v)
{
    return extendToVertex(params(p), *static_cast<BackwardState *>(state), v);
}

// The arc and then the vertex that enters the label, in one call (what GenericILS does at every extension).
EXPORT double rcc_stepFwd(void *p, void *state, int a, int v)
{
    ForwardState &s = *static_cast<ForwardState *>(state);
    double cost = extendAlongArc(params(p), s, a);
    return cost + extendToVertex(params(p), s, v);
}

EXPORT double rcc_stepBwd(void *p, void *state, int a, int v)
{
    BackwardState &s = *static_cast<BackwardState *>(state);
    double cost = extendAlongArc(params(p), s, a);
    return cost + extendToVertex(params(p), s, v);
}

EXPORT double rcc_concatFB(void *p, int v, const void *fwd, const void *bwd)
{
    return concatenationCost(params(p), v, *static_cast<const ForwardState *>(fwd),
                             *static_cast<const BackwardState *>(bwd));
}

EXPORT double rcc_concatFF(void *p, int v, const void *fwd, const void *bwd)
{
    return concatenationCost(params(p), v, *static_cast<const ForwardState *>(fwd),
                             *static_cast<const ForwardState *>(bwd));
}

EXPORT double rcc_dominationFwd(void *p, int v, const void *dominating, const void *dominated)
{
    return dominationCost(params(p), v, *static_cast<const ForwardState *>(dominating),
                          *static_cast<const ForwardState *>(dominated));
}

EXPORT double rcc_dominationBwd(void *p, int v, const void *dominating, const void *dominated)
{
    return dominationCost(params(p), v, *static_cast<const BackwardState *>(dominating),
                          *static_cast<const BackwardState *>(dominated));
}
