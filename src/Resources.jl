mutable struct Resources{CR <: AbstractResource} <: AbstractResources
    customResource::CR
    stdResource1::StandardResource{1}
    stdResource2::StandardResource{2}
end
