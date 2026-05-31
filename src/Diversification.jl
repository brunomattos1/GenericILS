struct Diversification
    outerShift::Int
    outerSwap::Int
    innerShift::Int
    innerSwap::Int
end
Diversification(; outerShift = 2, outerSwap = 0, innerShift = 2, innerSwap = 0) = Diversification(outerShift, outerSwap, innerShift, innerSwap)
