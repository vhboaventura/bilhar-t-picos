module BilharSim

export Ball, Table,
       TrackerState,
       brute_force_contact, init_tracker, step_tracker!, contact_point

# ---------------------------------------------------------------------------
# Modelo
# ---------------------------------------------------------------------------

"""
    Ball(center, radius; velocity=[0,0])

Bola circular da mesa. `center` é `Cⁱ(t)`, `radius` é `rⁱ`. `velocity` é
`dCⁱ/dt`, usada tanto para animar a bola quanto, exatamente, no termo `C′`
da EDO do algoritmo 2 (não é aproximada por diferença finita).
"""
mutable struct Ball
    center::Vector{Float64}
    radius::Float64
    velocity::Vector{Float64}
end
Ball(center::AbstractVector, radius::Real; velocity::AbstractVector = [0.0, 0.0]) =
    Ball(collect(Float64, center), Float64(radius), collect(Float64, velocity))

"""Mesa retangular `[0, width] x [0, height]`."""
struct Table
    width::Float64
    height::Float64
end

contact_point(P::AbstractVector, d::AbstractVector, s::Real) = P .+ s .* d

# ---------------------------------------------------------------------------
# Algoritmo 1 — força bruta (T1)
#
# Reta do taco: X(s) = P + s d, com d unitário e s >= 0 (à frente do taco).
# Interseção com a bola i: |X(s) - Cⁱ|² = (rⁱ)², uma equação quadrática em s.
# Entre todas as bolas atingidas, o ponto de contato é o de menor s (o
# primeiro que a reta encontra à frente do taco).
# ---------------------------------------------------------------------------

"""
    solve_line_circle(P, d, C, r) -> (s1, s2) ou nothing

Raízes da interseção reta-círculo, em ordem crescente. `nothing` se a reta
não intersecta o círculo (discriminante negativo).
"""
function solve_line_circle(P::AbstractVector, d::AbstractVector, C::AbstractVector, r::Real)
    V = P .- C
    b = 2 * (V[1] * d[1] + V[2] * d[2])
    c = (V[1]^2 + V[2]^2) - r^2
    disc = b^2 - 4c
    disc < 0 && return nothing
    sq = sqrt(disc)
    s1 = (-b - sq) / 2
    s2 = (-b + sq) / 2
    return (s1, s2)
end

"""
    brute_force_contact(balls, P, d; s_min=1e-6) -> (index, s, point) ou nothing

Algoritmo 1 (T1). Testa a reta do taco contra **todas** as bolas e retorna a
que produz o menor `s >= s_min` (bola/ponto de contato mais próximo do
taco). `nothing` se a reta não atinge nenhuma bola à frente do taco.
Custo: `O(n)` interseções reta-círculo por chamada, `n` = número de bolas.
"""
function brute_force_contact(balls::AbstractVector{Ball}, P::AbstractVector, d::AbstractVector;
                              s_min::Real = 1e-6)
    best_s = Inf
    best_i = 0
    for (i, ball) in enumerate(balls)
        roots = solve_line_circle(P, d, ball.center, ball.radius)
        roots === nothing && continue
        s1, s2 = roots
        s = s1 >= s_min ? s1 : (s2 >= s_min ? s2 : Inf)
        if s < best_s
            best_s = s
            best_i = i
        end
    end
    best_i == 0 && return nothing
    return (best_i, best_s, contact_point(P, d, best_s))
end

# ---------------------------------------------------------------------------
# Algoritmo 2 — rastreamento do ponto de contato via EDO (T2)
#
# Em vez de repetir a força bruta em todo frame, mantemos qual bola está
# "travada" (locked) e evoluímos apenas o parâmetro s(t) do contato com essa
# bola, através da EDO obtida por diferenciação implícita de
#   F(s,t) = |P(t) + s d(t) - C(t)|² - r² = 0
# em relação a t:
#   V := P + s d - C
#   2 V·(P′ + s′ d + s d′ - C′) = 0        (r constante)
#   s′ = -V·(P′ + s d′ - C′) / (V·d)
#
# Cada passo é um preditor de Euler nessa EDO seguido de UM passo de Newton
# (corretor) sobre F(s)=0 já no novo frame, o que mantém o erro de deriva
# limitado sem o custo de testar as outras bolas. Custo: O(1) por frame.
#
# "Melhoria 1" do quadro: a cada `recompute_every` frames (ou sempre que o
# corretor falhar / s sair da faixa válida) o rastreador é re-ancorado com
# uma chamada de força bruta — é o único jeito barato de detectar que a
# bola mais próxima do taco mudou (troca de alvo), algo que a EDO local não
# enxerga sozinha.
# ---------------------------------------------------------------------------

"""Estado do rastreador de contato do algoritmo 2 (T2)."""
mutable struct TrackerState
    ball_index::Int          # bola atualmente travada (0 = nenhum contato)
    s::Float64                # parâmetro de contato atual
    P_prev::Vector{Float64}
    d_prev::Vector{Float64}
    frame::Int
    recompute_every::Int      # "Melhoria 1": período de re-ancoragem por força bruta
    last_used_brute_force::Bool
end

"""
    init_tracker(balls, P, d; recompute_every=30) -> TrackerState

Inicializa o rastreador com uma chamada de força bruta (T1).
"""
function init_tracker(balls::AbstractVector{Ball}, P::AbstractVector, d::AbstractVector;
                       recompute_every::Int = 30)
    hit = brute_force_contact(balls, P, d)
    idx, s = hit === nothing ? (0, 0.0) : (hit[1], hit[2])
    return TrackerState(idx, s, collect(Float64, P), collect(Float64, d), 0, recompute_every, true)
end

const S_MAX = 1e6       # limite de sanidade para s (taco "infinito")
const NEWTON_TOL = 1e-9

"""
    step_tracker!(tracker, balls, P, d, dt) -> (index, s, point) ou nothing

Avança o rastreador um frame (algoritmo 2, T2). `P`, `d` são a posição e
direção *atuais* do taco; `dt` é o passo de tempo desde o frame anterior.
Retorna o mesmo formato de [`brute_force_contact`](@ref).
"""
function step_tracker!(tracker::TrackerState, balls::AbstractVector{Ball},
                        P::AbstractVector, d::AbstractVector, dt::Real)
    tracker.frame += 1
    tracker.last_used_brute_force = false

    force_recompute = tracker.ball_index == 0 ||
                       tracker.recompute_every <= 0 ||
                       tracker.frame % tracker.recompute_every == 0

    if !force_recompute
        ball = balls[tracker.ball_index]
        s = _ode_predict_correct(tracker, ball, P, d, dt)
        if s !== nothing && 0 <= s <= S_MAX
            tracker.s = s
            tracker.P_prev = collect(Float64, P)
            tracker.d_prev = collect(Float64, d)
            return (tracker.ball_index, s, contact_point(P, d, s))
        end
        # corretor falhou (fora do alcance/tangência degenerada): re-ancora abaixo
    end

    hit = brute_force_contact(balls, P, d)
    tracker.last_used_brute_force = true
    tracker.P_prev = collect(Float64, P)
    tracker.d_prev = collect(Float64, d)
    if hit === nothing
        tracker.ball_index = 0
        tracker.s = 0.0
        return nothing
    end
    tracker.ball_index, tracker.s, _ = hit
    return hit
end

function _ode_predict_correct(tracker::TrackerState, ball::Ball,
                               P::AbstractVector, d::AbstractVector, dt::Real)
    dt <= 0 && return nothing
    s0 = tracker.s
    Pp, dp = tracker.P_prev, tracker.d_prev

    Pdot = (collect(Float64, P) .- Pp) ./ dt
    ddot = (collect(Float64, d) .- dp) ./ dt
    Cdot = ball.velocity

    V = Pp .+ s0 .* dp .- ball.center           # avaliado no estado anterior
    denom = V[1] * dp[1] + V[2] * dp[2]
    abs(denom) < 1e-8 && return nothing          # reta ~tangente: mal condicionado

    rate = Pdot .+ s0 .* ddot .- Cdot
    num = V[1] * rate[1] + V[2] * rate[2]
    sdot = -num / denom
    s_pred = s0 + sdot * dt

    # corretor de Newton (1 passo) sobre F(s) = |P + s d - C|^2 - r^2 = 0,
    # já com P, d do frame atual, para conter a deriva do integrador de Euler.
    Vn = P .+ s_pred .* d .- ball.center
    F = Vn[1]^2 + Vn[2]^2 - ball.radius^2
    Fp = 2 * (Vn[1] * d[1] + Vn[2] * d[2])
    abs(Fp) < 1e-8 && return s_pred
    s_corr = s_pred - F / Fp

    Vc = P .+ s_corr .* d .- ball.center
    resid = abs(Vc[1]^2 + Vc[2]^2 - ball.radius^2)
    resid > 1e-3 * max(1.0, ball.radius^2) && return nothing  # corretor não convergiu

    return s_corr
end

end # module
