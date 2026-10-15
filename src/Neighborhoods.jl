"""Position of the neighborhood type `T` inside the solver's neighborhood tuple (resolved at compile time)."""
@generated function neighIndex(::Type{Solver{N, R, PM, FL, BL, AL}}, ::Type{T}) where {N, R, PM, FL, BL, AL, T}
    for (i, n) in enumerate(N.parameters)
        n <: T && return :( $i )
    end
    error("Neighborhood $T not registered in solver.neighborhoods")
end
