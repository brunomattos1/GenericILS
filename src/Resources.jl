mutable struct Resources{CR <: AbstractResource} <: AbstractResources
    customResource::Union{CR, Nothing}
    stdResource1::Union{StandardResource{1}, Nothing}
    stdResource2::Union{StandardResource{2}, Nothing}
end

Resources{CR}() where {CR <: AbstractResource} = Resources{CR}(nothing, nothing, nothing)

function addResource!(res::Resources{CR}, custom::CR) where {CR <: AbstractResource}
    res.customResource = custom
end

function addResource!(res::Resources, standard::StandardResource{1})
    res.stdResource1 = standard
end

function addResource!(res::Resources, standard::StandardResource{2})
    res.stdResource2 = standard
end
