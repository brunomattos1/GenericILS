@generated function neigh_index(::Type{Solver{N, R, PM, FL, BL, AL}}, ::Type{T}) where {N, R, PM, FL, BL, AL, T}
    for (i, n) in enumerate(N.parameters)
        n <: T && return :( $i )
    end
    error("Neighborhood $T not registered in solver.neighborhoods")
end
