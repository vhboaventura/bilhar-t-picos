module BilharSim

export Ball, Table,
       TrackerState, PREDICTORS,
       brute_force_contact, init_tracker, step_tracker!, contact_point, advance_balls!

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

"""
    advance_balls!(balls, table, dt; collide=false)

Move cada bola por `velocity * dt` e faz ela quicar elasticamente nas bordas
da mesa. Com `collide=true` também há colisão elástica bola-bola (massas
iguais), só para a visualização ficar realista — o foco do trabalho é o ponto
de contato do taco, não a dinâmica do jogo.
"""
function advance_balls!(balls::AbstractVector{Ball}, table::Table, dt::Real; collide::Bool = false)
    collide && _collide_balls!(balls)
    for b in balls
        for k in 1:2
            b.center[k] += b.velocity[k] * dt
            lo = b.radius
            hi = (k == 1 ? table.width : table.height) - b.radius
            if b.center[k] < lo
                b.center[k] = lo
                b.velocity[k] = abs(b.velocity[k])
            elseif b.center[k] > hi
                b.center[k] = hi
                b.velocity[k] = -abs(b.velocity[k])
            end
        end
    end
    return balls
end

function _collide_balls!(balls::AbstractVector{Ball})
    n = length(balls)
    for i in 1:n-1, j in i+1:n
        a, b = balls[i], balls[j]
        dx = b.center[1] - a.center[1]
        dy = b.center[2] - a.center[2]
        rsum = a.radius + b.radius
        dist2 = dx * dx + dy * dy
        (dist2 >= rsum * rsum || dist2 < 1e-12) && continue
        dist = sqrt(dist2)
        nx, ny = dx / dist, dy / dist
        vn = (a.velocity[1] - b.velocity[1]) * nx + (a.velocity[2] - b.velocity[2]) * ny
        if vn > 0                      # só se estiverem se aproximando
            a.velocity[1] -= vn * nx; a.velocity[2] -= vn * ny
            b.velocity[1] += vn * nx; b.velocity[2] += vn * ny
        end
        push = (rsum - dist) / 2       # separa as bolas sobrepostas
        a.center[1] -= push * nx; a.center[2] -= push * ny
        b.center[1] += push * nx; b.center[2] += push * ny
    end
    return balls
end

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
    Vx = P[1] - C[1]
    Vy = P[2] - C[2]
    b = 2 * (Vx * d[1] + Vy * d[2])
    c = Vx * Vx + Vy * Vy - r^2
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
        s1, _ = roots
        s = s1 >= s_min ? s1 : Inf       # só a raiz de entrada; ponta do taco dentro da bola não conta
        if s < best_s
            best_s = s
            best_i = i
        end
    end
    best_i == 0 && return nothing
    return (best_i, best_s, contact_point(P, d, best_s))
end

# ---------------------------------------------------------------------------
# Algoritmo 2 — rastreamento do ponto de contato (T2): preditor + corretor
#
# Em vez de repetir a força bruta em todo frame, mantemos qual bola está
# "travada" (locked) e atualizamos só o parâmetro s(t) do contato com essa
# bola, em duas etapas (esquema preditor–corretor):
#
#   PREDITOR  — chuta um valor inicial s_pred para o frame novo:
#     :order0  preditor de ordem 0:   s_pred = s(t)          (reaproveita o frame anterior)
#     :euler   preditor de 1ª ordem:  s_pred = s(t) + s′ dt  (passo de Euler na EDO abaixo)
#
#   CORRETOR  — método de Newton sobre F(s) = |P + s d − C|² − r² = 0, já com
#     P, d, C do frame novo, iterado até convergir (|Δs| < tol) ou até
#     `newton_iters` iterações. Quanto melhor o preditor, menos iterações.
#
# EDO do preditor de 1ª ordem (diferenciação implícita de F(s,t) = 0 em t):
#   V := P + s d − C
#   2 V·(P′ + s′ d + s d′ − C′) = 0        (r constante)
#   s′ = −V·(P′ + s d′ − C′) / (V·d)
#
# "Melhoria 1" do quadro: a cada `recompute_every` frames (ou sempre que o
# corretor falhar) o rastreador é re-ancorado com uma chamada de força bruta —
# é o único jeito barato de detectar que a bola mais próxima do taco mudou
# (troca de alvo), algo que o rastreamento local não enxerga sozinho.
# `recompute_every = 0` desliga a reancoragem periódica (só reancora se o
# corretor falhar).
# ---------------------------------------------------------------------------

const PREDICTORS = (:order0, :euler)

"""Estado do rastreador de contato do algoritmo 2 (T2)."""
mutable struct TrackerState
    ball_index::Int           # bola atualmente travada (0 = nenhum contato)
    s::Float64                 # parâmetro de contato atual
    P_prev::Vector{Float64}
    d_prev::Vector{Float64}
    frame::Int
    recompute_every::Int       # "Melhoria 1": período de re-ancoragem por força bruta (0 = nunca)
    predictor::Symbol          # :order0 ou :euler
    newton_iters::Int          # máximo de iterações de Newton por frame (0 = só preditor)
    newton_tol::Float64
    last_used_brute_force::Bool
    last_newton_iters::Int     # iterações de Newton gastas no último frame
    brute_force_calls::Int     # total de reancoragens (inclui a inicial)
    newton_iters_total::Int
end

"""
    init_tracker(balls, P, d; recompute_every=30, predictor=:euler,
                 newton_iters=10, newton_tol=1e-8) -> TrackerState

Inicializa o rastreador com uma chamada de força bruta (T1).
"""
function init_tracker(balls::AbstractVector{Ball}, P::AbstractVector, d::AbstractVector;
                       recompute_every::Int = 30, predictor::Symbol = :euler,
                       newton_iters::Int = 10, newton_tol::Float64 = 1e-8)
    predictor in PREDICTORS || throw(ArgumentError("predictor deve ser :order0 ou :euler"))
    hit = brute_force_contact(balls, P, d)
    idx, s = hit === nothing ? (0, 0.0) : (hit[1], hit[2])
    return TrackerState(idx, s, collect(Float64, P), collect(Float64, d), 0, recompute_every,
                        predictor, newton_iters, newton_tol, true, 0, 1, 0)
end

const S_MAX = 1e6       # limite de sanidade para s (taco "infinito")

"""
    step_tracker!(tracker, balls, P, d, dt) -> (index, s, point) ou nothing

Avança o rastreador um frame (algoritmo 2, T2). `P`, `d` são a posição e
direção *atuais* do taco, `balls` já estão na posição *atual* e `dt` é o
passo de tempo desde o frame anterior. Retorna o mesmo formato de
[`brute_force_contact`](@ref).
"""
function step_tracker!(tracker::TrackerState, balls::AbstractVector{Ball},
                        P::AbstractVector, d::AbstractVector, dt::Real)
    tracker.frame += 1
    tracker.last_used_brute_force = false
    tracker.last_newton_iters = 0

    force_recompute = tracker.ball_index == 0 ||
                       (tracker.recompute_every > 0 && tracker.frame % tracker.recompute_every == 0)

    if !force_recompute
        ball = balls[tracker.ball_index]
        res = _predict_correct(tracker, ball, P, d, dt)
        if res !== nothing
            s, iters = res
            if 0 <= s <= S_MAX
                tracker.s = s
                tracker.last_newton_iters = iters
                tracker.newton_iters_total += iters
                copyto!(tracker.P_prev, P)
                copyto!(tracker.d_prev, d)
                return (tracker.ball_index, s, contact_point(P, d, s))
            end
        end
        # corretor falhou (tangência, saiu do alcance, raiz errada): re-ancora abaixo
    end

    hit = brute_force_contact(balls, P, d)
    tracker.last_used_brute_force = true
    tracker.brute_force_calls += 1
    copyto!(tracker.P_prev, P)
    copyto!(tracker.d_prev, d)
    if hit === nothing
        tracker.ball_index = 0
        tracker.s = 0.0
        return nothing
    end
    tracker.ball_index, tracker.s, _ = hit
    return hit
end

# Preditor de 1ª ordem: passo de Euler na EDO de s(t). Avaliado no estado do
# frame ANTERIOR (P_prev, d_prev e o centro anterior da bola, recuperado pela
# velocidade conhecida); `nothing` se a reta estava quase tangente.
function _euler_predict(tracker::TrackerState, ball::Ball, P, d, dt)
    dt <= 0 && return nothing
    s0 = tracker.s
    Pp, dp = tracker.P_prev, tracker.d_prev
    cvx, cvy = ball.velocity[1], ball.velocity[2]

    Vx = Pp[1] + s0 * dp[1] - (ball.center[1] - cvx * dt)
    Vy = Pp[2] + s0 * dp[2] - (ball.center[2] - cvy * dt)
    denom = Vx * dp[1] + Vy * dp[2]
    abs(denom) < 1e-8 && return nothing

    rx = (P[1] - Pp[1]) / dt + s0 * (d[1] - dp[1]) / dt - cvx
    ry = (P[2] - Pp[2]) / dt + s0 * (d[2] - dp[2]) / dt - cvy
    sdot = -(Vx * rx + Vy * ry) / denom
    return s0 + sdot * dt
end

# Preditor + corretor de Newton. Retorna (s, iterações) ou `nothing` se o
# corretor não convergiu / caiu na raiz de saída da bola (a de interesse é a
# de entrada, onde V·d < 0).
function _predict_correct(tracker::TrackerState, ball::Ball, P, d, dt)
    s = tracker.predictor === :order0 ? tracker.s : _euler_predict(tracker, ball, P, d, dt)
    s === nothing && return nothing
    tracker.newton_iters == 0 && return (s, 0)

    cx, cy, r2 = ball.center[1], ball.center[2], ball.radius^2
    iters = 0
    for k in 1:tracker.newton_iters
        Vx = P[1] + s * d[1] - cx
        Vy = P[2] + s * d[2] - cy
        F = Vx * Vx + Vy * Vy - r2
        Fp = 2 * (Vx * d[1] + Vy * d[2])
        abs(Fp) < 1e-9 && return nothing          # reta ~tangente: Newton mal condicionado
        ds = F / Fp
        s -= ds
        iters = k
        abs(ds) < tracker.newton_tol && break
    end

    Vx = P[1] + s * d[1] - cx
    Vy = P[2] + s * d[2] - cy
    abs(Vx * Vx + Vy * Vy - r2) > 1e-8 * r2 && return nothing   # não convergiu
    Vx * d[1] + Vy * d[2] >= 0 && return nothing                 # raiz de saída, não de entrada
    return (s, iters)
end

end # module
