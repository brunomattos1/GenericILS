"""
Custom resources written in C++, in the format of the MetaSolver of VRPSolver (`rcsp_custom_res_impl.hpp/.cpp`),
used as the custom resource of GenericILS.

The labels (`ForwardLabel`/`BackwardLabel`) stay the ones of GenericILS; only the update of the custom state and cost
is done by the C++ functions:

  1. `shim.cpp` (generic) and the user `.cpp` are compiled into a shared library, cached by content hash;
  2. a state can be anything the C++ code defines. If it is plain data (trivially copyable) the label carries its
     bytes by value, as a hand-written resource does, as a Julia struct with the fields of the `.hpp` when they can be
     read, or as opaque bytes. Otherwise (a `std::vector`, a set, ...) the state lives in the C++ heap and the label
     holds a pointer that is freed when the label is collected: slower, but it works for any state that can be
     copied. The choice is automatic;
  3. `CppResource` implements `initState*`, `extendAlongArc`, `extendToVertex` and `concatenationCost` with `ccall`;
     as in the MetaSolver, `extendToVertex` runs right after `extendAlongArc`.

The cost that the C++ functions return is a per-step amount: the label cost is the sum of the amounts (`Inf` means
infeasible and stays `Inf`). The flags `isCostResource` and `isSymmetric` come from the C++ implementation and belong
to the resource, so they need no global definition. See `loadCppResource`.
"""
module CppResources

using Libdl
import ..GenericILS
import ..GenericILS: AbstractResource, ForwardLabel, BackwardLabel,
    initStateForward, initStateBackward, extendAlongArc, extendToVertex, concatenationCost,
    isCostResource, isSymmetric, customExtension

export CppResource, loadCppResource, freeCppResource!

# ---------------------------------------------------------------------------------------------------------
# Compilation
# ---------------------------------------------------------------------------------------------------------

const SHIM = joinpath(@__DIR__, "shim.cpp")

"""
Path of the C++ compiler: the `CXX` environment variable if it is set, otherwise the first of `g++`, `clang++` and
`c++` that is installed. `nothing` when there is none.
"""
function cppCompiler()
    haskey(ENV, "CXX") && !isempty(ENV["CXX"]) && return Sys.which(ENV["CXX"])
    for name in ("g++", "clang++", "c++")
        path = Sys.which(name)
        path === nothing || return path
    end
    return nothing
end

"""
Compile `shim.cpp` and the user `cpp` into a shared library and return its path. The library name has the hash of
the sources, so an unchanged resource is not compiled again.
"""
function compileCppResource(cpp::String, hpp::String; compiler::Union{Nothing, String} = nothing,
                            cacheDir::String = joinpath(tempdir(), "genericils_cpp"), flags::Vector{String} = String[])
    isfile(cpp) || throw(ArgumentError("C++ file not found: $cpp"))
    isfile(hpp) || throw(ArgumentError("header not found: $hpp"))
    compilerPath = compiler === nothing ? cppCompiler() : Sys.which(compiler)
    compilerPath === nothing &&
        error("no C++ compiler found" * (compiler === nothing ? ": install g++ or clang++, or set the CXX environment variable" : ": `$compiler`"))
    mkpath(cacheDir)
    key = string(hash(read(SHIM, String) * read(cpp, String) * read(hpp, String) * join(flags)); base = 16)
    ext = Sys.iswindows() ? "dll" : Sys.isapple() ? "dylib" : "so"
    lib = joinpath(cacheDir, "cppres_$key.$ext")
    isfile(lib) && return lib
    extra = Sys.iswindows() ? ["-static-libgcc", "-static-libstdc++"] : ["-fPIC"]
    partial = "$lib.$(getpid()).tmp"          # another process may build the same library
    cmd = `$compilerPath -std=c++17 -O2 -shared $extra $flags -I$(dirname(abspath(hpp))) -I$(@__DIR__) $SHIM $cpp -o $partial`
    try
        run(cmd)
        mv(partial, lib; force = true)
    catch e
        rm(partial; force = true)
        isfile(lib) && return lib
        error("could not compile the C++ resource: $cmd")
    end
    return lib
end

# ---------------------------------------------------------------------------------------------------------
# Reading the structs of the hpp
# ---------------------------------------------------------------------------------------------------------

const CTYPES = Dict{String, DataType}(
    "double" => Float64, "float" => Float32,
    "int" => Cint, "unsigned" => Cuint, "unsigned int" => Cuint, "short" => Cshort,
    "long" => Clong, "long long" => Clonglong, "unsigned long long" => Culonglong,
    "bool" => Bool, "char" => Cchar, "size_t" => Csize_t,
    "signed char" => Cchar, "unsigned char" => Cuchar, "unsigned short" => Cushort, "unsigned long" => Culong,
    "int8_t" => Int8, "int16_t" => Int16, "int32_t" => Int32, "int64_t" => Int64,
    "uint8_t" => UInt8, "uint16_t" => UInt16, "uint32_t" => UInt32, "uint64_t" => UInt64,
)

"Text of `source` without `//` and `/* */` comments."
stripComments(source::AbstractString) = replace(source, r"//[^\n]*" => "", r"/\*.*?\*/"s => "")

"Body of `struct name { ... }` in `source` (comments removed), without the outer braces."
function structBody(source::AbstractString, name::AbstractString)
    src = stripComments(source)
    m = match(Regex("struct\\s+$name\\s*\\{"), src)
    m === nothing && error("struct $name not found in the hpp")
    start = m.offset + length(m.match)
    depth = 1
    i = start
    while depth > 0
        i > lastindex(src) && error("unbalanced braces in struct $name")
        src[i] == '{' && (depth += 1)
        src[i] == '}' && (depth -= 1)
        i = nextind(src, i)
    end
    return src[start:prevind(src, prevind(src, i))]
end

"""
Fields `name => type` of the plain data members of `struct name` in the hpp, in declaration order. Constructors and
member functions are ignored.
"""
function hppStructFields(source::AbstractString, name::AbstractString)
    body = structBody(source, name)
    # a block (constructor or function body) becomes a separator, so the declarations around it stay apart
    while occursin('{', body)
        body = replace(body, r"\{[^{}]*\}" => ";")
    end
    fields = Pair{Symbol, DataType}[]
    for stmt in split(body, ';')
        stmt = strip(replace(stmt, r"=[^,]*" => ""))
        (isempty(stmt) || occursin('(', stmt)) && continue
        occursin(r"[\[\*&]", stmt) && error("unsupported member `$stmt` in struct $name (only plain scalars)")
        parts = split(stmt, ',')
        first = split(strip(parts[1]))
        typ = join(filter(!=("const"), first[1:end-1]), " ")
        haskey(CTYPES, typ) || error("unsupported type `$typ` in struct $name")
        for field in [first[end]; strip.(parts[2:end])]
            push!(fields, Symbol(field) => CTYPES[typ])
        end
    end
    return fields
end

"Julia `isbits` struct `name` with the given fields (reused when it already exists)."
function defineStruct!(name::Symbol, fields::Vector{Pair{Symbol, DataType}})
    isdefined(CppResources, name) && return getfield(CppResources, name)
    block = Expr(:block, [:($f::$t) for (f, t) in fields]...)
    Core.eval(CppResources, Expr(:struct, false, name, block))
    return getfield(CppResources, name)
end

"""
Julia type for the plain-data struct `name` of the header: a struct with its fields when they can be read and the size
agrees with the C++ one, `nothing` otherwise.
"""
function plainType(source::AbstractString, name::AbstractString, key::AbstractString, cSize::Integer)
    fields = try
        hppStructFields(source, name)
    catch e
        e isa ErrorException || rethrow()
        return nothing
    end
    T = defineStruct!(Symbol(name, "_", key), fields)
    return (sizeof(T) == cSize || (sizeof(T) == 0 && cSize == 1)) ? T : nothing
end

"The bytes of a plain state that the header does not let us read field by field, in blocks of 8 for the alignment."
struct OpaqueState{K}
    data::NTuple{K, UInt64}
end

"""
Value of the parameter struct `name` (C++ size `cSize`) for `value`: a `NamedTuple` with its fields (read from the
header, `T`) or a Julia `isbits` value with the same layout, as the MetaSolver passes them.
"""
function paramValue(T::Union{Nothing, DataType}, name::AbstractString, value, cSize::Integer)
    if value isa NamedTuple
        T === nothing && throw(ArgumentError("the fields of $name cannot be read from the header: pass an isbits Julia value with its layout instead of a NamedTuple"))
        value isa T && return value
        missingFields = [f for f in fieldnames(T) if !hasproperty(value, f)]
        isempty(missingFields) || throw(ArgumentError("missing $(join(missingFields, ", ")) for $name, which has fields $(fieldnames(T))"))
        vals = map(f -> convert(fieldtype(T, f), getproperty(value, f)), fieldnames(T))
        return Base.invokelatest(T, vals...)   # the constructor of a type created at run time is of a newer world
    end
    (T !== nothing && value isa T) && return value
    (isbitstype(typeof(value)) && (sizeof(value) == cSize || (sizeof(value) == 0 && cSize == 1))) ||
        throw(ArgumentError("$name takes a NamedTuple with its fields or an isbits value of $cSize bytes, not $(typeof(value))"))
    return value
end

# ---------------------------------------------------------------------------------------------------------
# The resource
# ---------------------------------------------------------------------------------------------------------

"Pointers to the functions of the shared library."
struct CppLib
    makeFwd::Ptr{Cvoid}
    makeBwd::Ptr{Cvoid}
    copyFwd::Ptr{Cvoid}
    copyBwd::Ptr{Cvoid}
    freeFwd::Ptr{Cvoid}
    freeBwd::Ptr{Cvoid}
    initFwd::Ptr{Cvoid}
    initBwd::Ptr{Cvoid}
    extendFwd::Ptr{Cvoid}
    extendBwd::Ptr{Cvoid}
    vertexFwd::Ptr{Cvoid}
    vertexBwd::Ptr{Cvoid}
    stepFwd::Ptr{Cvoid}
    stepBwd::Ptr{Cvoid}
    concatFB::Ptr{Cvoid}
    concatFF::Ptr{Cvoid}
    dominationFwd::Ptr{Cvoid}
    dominationBwd::Ptr{Cvoid}
end

"Number of C++ objects released so far, by `freeCppResource!` or by the garbage collector (for tests)."
const RELEASED = Threads.Atomic{Int}(0)

"""
Owner of the `CustomResParameters` object of the library. It is the only mutable part of a `CppResource`, so the
finalizer is attached to it: the object is released when the last copy of the resource is collected, or earlier by
`freeCppResource!`. The shared library itself stays loaded for the whole process, as other resources may use it.
"""
mutable struct CppOwner
    handle::Ptr{Cvoid}
    const free::Ptr{Cvoid}
end

"Release the C++ object of `owner` (once)."
function release!(owner::CppOwner)
    if owner.handle != C_NULL
        ccall(owner.free, Cvoid, (Ptr{Cvoid},), owner.handle)
        owner.handle = C_NULL
        Threads.atomic_add!(RELEASED, 1)
    end
    return nothing
end

"""
A state that lives in the C++ heap, for the states that cannot be copied byte by byte. The label holds this object;
the C++ state is freed when the object is collected. A state is never changed after it is made (an extension updates
a copy), so labels, and solutions that copy labels, can share it.
"""
mutable struct CppState
    ptr::Ptr{Cvoid}
    const free::Ptr{Cvoid}
end

"""
States in the C++ heap that are alive. The garbage collector of Julia does not see that memory, and a search makes
millions of states, so when their number passes `STATE_LIMIT` a collection is forced, which runs the finalizers.
"""
const LIVE_STATES = Threads.Atomic{Int}(0)
const MIN_STATE_LIMIT = 200_000
const STATE_LIMIT = Threads.Atomic{Int}(MIN_STATE_LIMIT)

"Release the C++ state (once)."
function releaseState!(state::CppState)
    if state.ptr != C_NULL
        ccall(state.free, Cvoid, (Ptr{Cvoid},), state.ptr)
        state.ptr = C_NULL
        Threads.atomic_sub!(LIVE_STATES, 1)
    end
    return nothing
end

"Collect garbage because too many C++ states are alive; the next limit is twice what survives."
function collectStates()
    GC.gc(false)
    LIVE_STATES[] > STATE_LIMIT[] ÷ 2 && GC.gc(true)      # the young generation was not enough
    STATE_LIMIT[] = max(MIN_STATE_LIMIT, 2 * LIVE_STATES[])
    return nothing
end

"Take ownership of the C++ state at `ptr`, which `free` releases."
function ownState(ptr::Ptr{Cvoid}, free::Ptr{Cvoid})
    ptr == C_NULL && throw(ArgumentError("the state of this C++ resource cannot be created: its type needs a default and a copy constructor"))
    state = CppState(ptr, free)
    finalizer(releaseState!, state)
    Threads.atomic_add!(LIVE_STATES, 1) >= STATE_LIMIT[] && collectStates()
    return state
end

# `deepcopy` of a solution must not duplicate the pointer (it would be freed twice); the state is never modified, so sharing is safe
Base.deepcopy_internal(state::CppState, ::IdDict) = state

# a CppState goes to C as the pointer it owns, and stays alive during the call
Base.cconvert(::Type{Ptr{Cvoid}}, state::CppState) = state
Base.unsafe_convert(::Type{Ptr{Cvoid}}, state::CppState) = state.ptr

"""
Custom resource implemented in C++. `FS` and `BS` are the Julia types that carry the forward and backward states: a
generated struct with the fields of the header, `OpaqueState` or `CppState`. Vertex `i` of the matrices of GenericILS
is the vertex `i - 1` of the interface, and the arc `(i, j)` has the id `(i - 1) * nv + (j - 1)`. Build it with
`loadCppResource`.
"""
struct CppResource{FS, BS} <: AbstractResource
    lib::CppLib
    owner::CppOwner
    nv::Int
    isCost::Bool
    isSym::Bool
end

"Pointer to the C++ object of `owner`; fails once it was freed instead of crashing."
@inline function handleOf(owner::CppOwner)
    h = owner.handle
    h == C_NULL && throw(ArgumentError("this C++ resource was freed with freeCppResource!"))
    return h
end

"Id of the arc `(i, j)` (matrix indices) in the interface."
@inline arcId(res::CppResource, i::Int, j::Int) = (i - 1) * res.nv + (j - 1)

"""
    freeCppResource!(res)

Release the C++ object of `res` now instead of when it is collected. Calling it again is harmless; using the resource
afterwards throws an error. Copies of `res` share the object. The states of the labels are separate objects and are
not affected.
"""
freeCppResource!(res::CppResource) = release!(res.owner)

setArc!(f::Ptr{Cvoid}, h::Ptr{Cvoid}, a::Int, value::T) where T = ccall(f, Cvoid, (Ptr{Cvoid}, Cint, Ref{T}, Cint), h, a, Ref(value), sizeof(T))
setVertex!(f::Ptr{Cvoid}, h::Ptr{Cvoid}, v::Int, value::T) where T = ccall(f, Cvoid, (Ptr{Cvoid}, Cint, Ref{T}, Cint), h, v, Ref(value), sizeof(T))
setConst!(f::Ptr{Cvoid}, h::Ptr{Cvoid}, value::T) where T = ccall(f, Cvoid, (Ptr{Cvoid}, Ref{T}, Cint), h, Ref(value), sizeof(T))

"""
    loadCppResource(cpp, hpp; nv, constParams = (;), arcParams = nothing, vertexParams = nothing,
                    ownedStates = false, compiler = nothing, flags = String[])

Compile (if it was not compiled before) and load the custom resource written in `cpp`/`hpp`, in the format of the
`rcsp_custom_res_impl` files of the MetaSolver.

- `nv` is the number of vertices of the matrices of the problem, depot included.
- `arcParams(i, j)` returns the parameters of the arc with matrix indices `(i, j)` as a `NamedTuple` with the fields of
  `CustomResArcParameters`; `vertexParams(v)` those of the vertex with matrix index `v`, fields of
  `CustomResVertexParameters`; `constParams` is a `NamedTuple` with the fields of `CustomResConstParameters`. A
  function is required only for the parameters that have fields. As in the MetaSolver, these three structs must be
  plain data; instead of a `NamedTuple` a function may return an `isbits` Julia value with the same layout.
- The states can be of any type that has a default and a copy constructor. Plain data (trivially copyable) is kept in
  the label by value, which is fast; any other state (with a `std::vector`, a set, ...) lives in the C++ heap and is
  freed when its label is collected, which is slower. `ownedStates = true` uses the second way for every state.
- `compiler` is the C++ compiler (default: `CXX`, or `g++`, `clang++`, `c++`); `flags` are extra compiler options.

The result goes in `Resources` like any custom resource, with no further definition: its `isCostResource` and
`isSymmetric` come from the C++ functions of the same name. It owns a C++ object that is released by `freeCppResource!`
or by the garbage collector.
"""
function loadCppResource(cpp::AbstractString, hpp::AbstractString; nv::Int, constParams = (;), arcParams = nothing,
                         vertexParams = nothing, ownedStates::Bool = false,
                         compiler::Union{Nothing, AbstractString} = nothing, flags::Vector{String} = String[])
    cpp, hpp = String(cpp), String(hpp)
    lib = Libdl.dlopen(compileCppResource(cpp, hpp; compiler = compiler === nothing ? nothing : String(compiler), flags))
    sym(name) = Libdl.dlsym(lib, name)
    cppLib = CppLib(sym(:rcc_makeFwd), sym(:rcc_makeBwd), sym(:rcc_copyFwd), sym(:rcc_copyBwd), sym(:rcc_freeFwd), sym(:rcc_freeBwd),
        sym(:rcc_initFwd), sym(:rcc_initBwd), sym(:rcc_extendFwd), sym(:rcc_extendBwd),
        sym(:rcc_vertexFwd), sym(:rcc_vertexBwd), sym(:rcc_stepFwd), sym(:rcc_stepBwd), sym(:rcc_concatFB), sym(:rcc_concatFF), sym(:rcc_dominationFwd), sym(:rcc_dominationBwd))

    source = read(hpp, String)
    key = string(hash(source); base = 16)
    names = ["ForwardState", "BackwardState", "CustomResArcParameters", "CustomResVertexParameters", "CustomResConstParameters"]
    sizes = [Int(ccall(sym(:rcc_sizeof), Cint, (Cint,), k)) for k in 0:4]
    plain = [ccall(sym(:rcc_isPlain), Cint, (Cint,), k) != 0 for k in 0:4]
    fieldTypes = [plainType(source, names[k], key, sizes[k]) for k in 1:5]

    # states: by value when they are plain data, in the C++ heap otherwise (or when it was asked)
    stateType(k) = (ownedStates || !plain[k]) ? CppState : (fieldTypes[k] === nothing ? OpaqueState{cld(sizes[k], 8)} : fieldTypes[k])
    FS, BS = stateType(1), stateType(2)
    for k in 3:5
        plain[k] || throw(ArgumentError("$(names[k]) is not plain data (trivially copyable): the MetaSolver passes it by value from Julia, so it cannot hold a vector"))
    end
    AT, VT, CT = fieldTypes[3], fieldTypes[4], fieldTypes[5]
    hasFields(k) = fieldTypes[k] === nothing ? sizes[k] > 1 : sizeof(fieldTypes[k]) > 0
    hasFields(3) && arcParams === nothing && throw(ArgumentError("the arc parameters have fields: pass `arcParams`"))
    hasFields(4) && vertexParams === nothing && throw(ArgumentError("the vertex parameters have fields: pass `vertexParams`"))
    constValue = paramValue(CT, names[5], constParams, sizes[5])        # checked before anything is allocated in C++

    handle = ccall(sym(:rcc_new), Ptr{Cvoid}, ())
    owner = CppOwner(handle, sym(:rcc_free))
    finalizer(release!, owner)
    ccall(sym(:rcc_setDimensions), Cvoid, (Ptr{Cvoid}, Cint, Cint), handle, nv, nv * nv)
    setConst!(sym(:rcc_setConst), handle, constValue)
    if arcParams !== nothing
        setArcFn = sym(:rcc_setArc)
        for i in 1:nv, j in 1:nv
            setArc!(setArcFn, handle, (i - 1) * nv + (j - 1), paramValue(AT, names[3], arcParams(i, j), sizes[3]))
        end
    end
    if vertexParams !== nothing
        setVertexFn = sym(:rcc_setVertex)
        for v in 1:nv
            setVertex!(setVertexFn, handle, v - 1, paramValue(VT, names[4], vertexParams(v), sizes[4]))
        end
    end

    isCost = ccall(sym(:rcc_isCostResource), Cint, ()) != 0
    isSym = ccall(sym(:rcc_symmetric), Cint, (Ptr{Cvoid},), handle) != 0
    return CppResource{FS, BS}(cppLib, owner, nv, isCost, isSym)
end

# ---------------------------------------------------------------------------------------------------------
# The contract of GenericILS
# ---------------------------------------------------------------------------------------------------------

# the flags come from the C++ implementation, so they belong to the resource and not to the process
isCostResource(res::CppResource) = res.isCost
isSymmetric(res::CppResource) = res.isSym

# How a state travels to C++. Plain data: a copy on the stack, which C++ updates in place. A C++ state: it is
# copied in C++ first, and the copy is updated, because the original may be shared by other labels.
@inline writable(state::T, ::Ptr{Cvoid}, ::Ptr{Cvoid}) where T = Ref{T}(state)
@inline writable(state::CppState, copy::Ptr{Cvoid}, free::Ptr{Cvoid}) = ownState(ccall(copy, Ptr{Cvoid}, (Ptr{Cvoid},), state.ptr), free)
@inline readable(state::T) where T = Ref{T}(state)
@inline readable(state::CppState) = state
@inline stateOf(ref::Ref) = ref[]
@inline stateOf(state::CppState) = state

# the first state of a prefix or a suffix
@inline function firstState(::Type{S}, init::Ptr{Cvoid}, make::Ptr{Cvoid}, free::Ptr{Cvoid}, handle::Ptr{Cvoid}) where S
    ref = Ref{S}()
    ccall(init, Cvoid, (Ptr{Cvoid}, Ptr{Cvoid}), handle, ref)
    return ref[]
end
@inline firstState(::Type{CppState}, ::Ptr{Cvoid}, make::Ptr{Cvoid}, free::Ptr{Cvoid}, handle::Ptr{Cvoid}) =
    ownState(ccall(make, Ptr{Cvoid}, (Ptr{Cvoid},), handle), free)

function initStateForward(res::CppResource{FS, BS}) where {FS, BS}
    owner = res.owner
    state = GC.@preserve owner firstState(FS, res.lib.initFwd, res.lib.makeFwd, res.lib.freeFwd, handleOf(owner))
    return (state, 0.0)
end

function initStateBackward(res::CppResource{FS, BS}) where {FS, BS}
    owner = res.owner
    state = GC.@preserve owner firstState(BS, res.lib.initBwd, res.lib.makeBwd, res.lib.freeBwd, handleOf(owner))
    return (state, 0.0)
end

# GenericILS extends a label by the arc and then by the vertex that enters it. For a C++ resource that is a single
# call, which also makes a single copy of a state that lives in the C++ heap.
function customExtension(res::CppResource{FS, BS}, label::ForwardLabel, a::Tuple{Int, Int}) where {FS, BS}
    owner = res.owner
    state = writable(label.state, res.lib.copyFwd, res.lib.freeFwd)
    c = GC.@preserve owner ccall(res.lib.stepFwd, Cdouble, (Ptr{Cvoid}, Ptr{Cvoid}, Cint, Cint), handleOf(owner), state, arcId(res, a[1], a[2]), a[2] - 1)
    return (stateOf(state), label.cost + c)
end

function customExtension(res::CppResource{FS, BS}, label::BackwardLabel, a::Tuple{Int, Int}) where {FS, BS}
    owner = res.owner
    state = writable(label.state, res.lib.copyBwd, res.lib.freeBwd)
    c = GC.@preserve owner ccall(res.lib.stepBwd, Cdouble, (Ptr{Cvoid}, Ptr{Cvoid}, Cint, Cint), handleOf(owner), state, arcId(res, a[2], a[1]), a[1] - 1)
    return (stateOf(state), label.cost + c)
end

"Forward extension along `a = (i, j)`: appends `j`."
function extendAlongArc(res::CppResource{FS, BS}, label::ForwardLabel, a::Tuple{Int, Int}) where {FS, BS}
    owner = res.owner
    state = writable(label.state, res.lib.copyFwd, res.lib.freeFwd)
    c = GC.@preserve owner ccall(res.lib.extendFwd, Cdouble, (Ptr{Cvoid}, Ptr{Cvoid}, Cint), handleOf(owner), state, arcId(res, a[1], a[2]))
    return (stateOf(state), label.cost + c)
end

"""
Backward extension along `a = (next, cur)`: the interface wants the arc in the direction of travel,
`(cur, next)`.
"""
function extendAlongArc(res::CppResource{FS, BS}, label::BackwardLabel, a::Tuple{Int, Int}) where {FS, BS}
    owner = res.owner
    state = writable(label.state, res.lib.copyBwd, res.lib.freeBwd)
    c = GC.@preserve owner ccall(res.lib.extendBwd, Cdouble, (Ptr{Cvoid}, Ptr{Cvoid}, Cint), handleOf(owner), state, arcId(res, a[2], a[1]))
    return (stateOf(state), label.cost + c)
end

"Vertex step after the forward arc; `v` is the matrix index of the vertex, `v - 1` in the interface."
function extendToVertex(res::CppResource{FS, BS}, label::ForwardLabel, v::Int) where {FS, BS}
    owner = res.owner
    state = writable(label.state, res.lib.copyFwd, res.lib.freeFwd)
    c = GC.@preserve owner ccall(res.lib.vertexFwd, Cdouble, (Ptr{Cvoid}, Ptr{Cvoid}, Cint), handleOf(owner), state, v - 1)
    return (stateOf(state), label.cost + c)
end

"Vertex step after the backward arc `(next, cur)`; `v` is `next`, the vertex that joins the suffix."
function extendToVertex(res::CppResource{FS, BS}, label::BackwardLabel, v::Int) where {FS, BS}
    owner = res.owner
    state = writable(label.state, res.lib.copyBwd, res.lib.freeBwd)
    c = GC.@preserve owner ccall(res.lib.vertexBwd, Cdouble, (Ptr{Cvoid}, Ptr{Cvoid}, Cint), handleOf(owner), state, v - 1)
    return (stateOf(state), label.cost + c)
end

"Cost of the route made by the prefix and the suffix. The state is not used afterwards, so the forward one is kept."
function concatenationCost(res::CppResource{FS, BS}, v::Int, forwardLabel::ForwardLabel, backwardLabel::BackwardLabel) where {FS, BS}
    owner = res.owner
    c = GC.@preserve owner ccall(res.lib.concatFB, Cdouble, (Ptr{Cvoid}, Cint, Ptr{Cvoid}, Ptr{Cvoid}), handleOf(owner), v,
        readable(forwardLabel.state), readable(backwardLabel.state))
    return (forwardLabel.state, forwardLabel.cost + backwardLabel.cost + c)
end

end # module CppResources
