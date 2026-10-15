@testset "Route block moves" begin
    # reference implementations on plain vectors
    function refMoveBlocks(r1, i, k1, r2, j, k2)
        b1 = r1[i:i+k1-1]
        b2 = r2[j:j+k2-1]
        return vcat(r1[1:i-1], b2, r1[i+k1:end]), vcat(r2[1:j-1], b1, r2[j+k2:end])
    end

    @testset "moveBlocks! swaps blocks of any size between two routes" begin
        for k1 in 0:3, k2 in 0:3
            r1 = collect(100:111)
            r2 = collect(200:211)
            i, j = 3, 4
            e1, e2 = refMoveBlocks(r1, i, k1, r2, j, k2)
            buffer = zeros(Int, 3)
            GenericILS.moveBlocks!(r1, i, k1, r2, j, k2, buffer)
            @test r1 == e1
            @test r2 == e2
        end
    end

    @testset "moveBlocksIntra! moves a block forward and backward" begin
        for (i, j) in [(2, 7), (7, 2), (3, 3), (2, 3)], k in 0:2
            i < j && j < i + k && continue          # overlapping blocks are not valid moves
            route = collect(1:10)
            expected = copy(route)
            if i != j && k > 0
                block = route[i:i+k-1]
                rest = vcat(route[1:i-1], route[i+k:end])
                pos = i < j ? j - k : j          # j is a position in the original route
                expected = vcat(rest[1:pos-1], block, rest[pos:end])
            end
            GenericILS.moveBlocksIntra!(route, i, k, j, Int[])
            @test route == expected
        end
    end
end
