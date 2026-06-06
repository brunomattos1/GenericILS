const NEIGHBORHOODS = (
    TwoOptStar(),
    IntraShift(),
    InterShift{1}(),
    InterShift{2}(),
    InterSwap{1, 1}(),
    InterSwap{2, 1}(),
    InterSwap{2, 2}()
)
const NUM_NEIGHBORHOODS = length(NEIGHBORHOODS)

@generated function neigh_index(::Type{T}) where T
    for (i, n) in enumerate(NEIGHBORHOODS)
        if n isa T
            return :( $i )
        end
    end
    error("Neighborhood not registered")
end