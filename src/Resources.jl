mutable struct Resources{CR <: AbstractResource} <: AbstractResources
    customResource::CR
    stdResource1::StandardResource{1}
    stdResource2::StandardResource{2}
end

Resources(custom::CR, std1::StandardResource{1}, std2::StandardResource{2}) where {CR <: AbstractResource} =
    Resources{CR}(custom, std1, std2)
