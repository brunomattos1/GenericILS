
# ─────────────────────────────────────────────────────────────
# BestOnly — aceita apenas se a solução corrente for estritamente
#            melhor que a melhor solução conhecida.
# ─────────────────────────────────────────────────────────────
struct BestOnly end

accept!(::BestOnly, sol::Solution, bestSol::Solution, rng) =
    sol.cost < bestSol.cost - 1e-6

# após recusar, reverte sol para bestSol
revert!(::BestOnly, sol::Solution, bestSol::Solution) =
    copy_solution!(sol, bestSol)

update!(::BestOnly)   = nothing
reset!(::BestOnly)    = nothing
tick!(::BestOnly)     = nothing
improved!(::BestOnly) = nothing

# stop é delegado ao critério de parada externo
stop!(::BestOnly) = false


# ─────────────────────────────────────────────────────────────
# SimulatedAnnealing — aceita soluções piores com probabilidade
#                      exp(-Δ/T), onde T resfria a cada iteração.
#
# Parâmetros:
#   T₀    — temperatura inicial
#   T_min — temperatura mínima (condição de parada)
#   alpha — fator de resfriamento (0 < alpha < 1)
# ─────────────────────────────────────────────────────────────
mutable struct SimulatedAnnealing
    T₀::Float64
    T::Float64
    T_min::Float64
    alpha::Float64
end

SimulatedAnnealing(T₀::Float64, T_min::Float64, alpha::Float64) =
    SimulatedAnnealing(T₀, T₀, T_min, alpha)

function accept!(sa::SimulatedAnnealing, sol::Solution, bestSol::Solution, rng)
    Δ = sol.cost - bestSol.cost
    Δ < -1e-6 && return true
    sa.T > sa.T_min || return false
    return rand(rng) < exp(-Δ / sa.T)
end

# SA mantém a solução corrente (não reverte para best)
revert!(::SimulatedAnnealing, sol::Solution, bestSol::Solution) = nothing

update!(sa::SimulatedAnnealing)   = (sa.T = max(sa.T * sa.alpha, sa.T_min); nothing)
reset!(sa::SimulatedAnnealing)    = (sa.T = sa.T₀; nothing)
tick!(::SimulatedAnnealing)       = nothing   # progressão é pelo resfriamento, não por contagem
improved!(::SimulatedAnnealing)   = nothing

# SA para quando a temperatura atinge o mínimo
stop!(sa::SimulatedAnnealing) = sa.T <= sa.T_min
