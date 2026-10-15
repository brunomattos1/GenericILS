"""
The three resources of a problem: one user-defined `customResource` and the two built-in
`StandardResource` slots (`stdResource1`, `stdResource2`).
"""
mutable struct Resources{CR <: AbstractResource} <: AbstractResources
    customResource::CR
    stdResource1::StandardResource{1}
    stdResource2::StandardResource{2}
end
