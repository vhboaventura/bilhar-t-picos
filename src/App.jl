module App

using GLMakie
using LinearAlgebra
using Printf
include(joinpath(@__DIR__, "BilharSim.jl"))
using .BilharSim

"""
    run_app(; n_balls=12, width=200.0, height=100.0, recompute_every=30,
              min_radius=5.0, max_radius=8.0)

Abre a visualização interativa da mesa de bilhar 2D. `min_radius` e
`max_radius` controlam o tamanho (raio) das bolas, sorteado nesse
intervalo para cada bola.

- As bolas (verde) se movem e quicam nas bordas da mesa: `Cⁱ(t)` variável,
  exatamente como no modelo do algoritmo 2.
- Arraste o botão esquerdo do mouse para mirar o taco: o ponto onde o
  clique começa é `P`, a posição atual do mouse define a direção `d`.
- **× vermelho** = ponto de contato do algoritmo 1 (força bruta), recalculado
  do zero todo frame.
- **○ azul** = ponto de contato rastreado pelo algoritmo 2 (EDO). Quando os
  dois coincidem, o rastreamento é fiel à força bruta a uma fração do custo.
- Teclas `[` / `]` diminuem/aumentam `recompute_every` (Melhoria 1); a barra
  de status fica amarela no frame em que o rastreador reancora por força
  bruta.
"""
function run_app(; n_balls::Int = 12, width::Float64 = 200.0, height::Float64 = 100.0,
                  recompute_every::Int = 30, min_radius::Float64 = 5.0, max_radius::Float64 = 8.0)
    balls = [Ball([10 + rand() * (width - 20), 10 + rand() * (height - 20)],
                   min_radius + rand() * (max_radius - min_radius);
                   velocity = (rand(2) .- 0.5) .* 25.0)
             for _ in 1:n_balls]

    fig = Figure(size = (1000, 640))
    ax = Axis(fig[1, 1], limits = (0, width, 0, height), aspect = DataAspect(),
              title = "Simulação de pontos de contato — mesa de bilhar 2D")

    lines!(ax, [0, width, width, 0, 0], [0, 0, height, height, 0]; color = :black, linewidth = 2)

    centers_obs = Observable(Point2f[(b.center[1], b.center[2]) for b in balls])
    radii = [b.radius for b in balls]
    scatter!(ax, centers_obs; markersize = radii .* 2, markerspace = :data,
             color = (:seagreen, 0.55), strokecolor = :seagreen, strokewidth = 2)

    cue_P = Observable(Point2f(width * 0.1, height * 0.5))
    cue_d = Observable(Point2f(1.0, 0.0))
    dragging = Observable(false)

    cue_line = @lift(Point2f[$cue_P, $cue_P .+ $cue_d .* max(width, height) * 1.5])
    lines!(ax, cue_line; color = :dodgerblue, linewidth = 1.5, linestyle = :dash)
    scatter!(ax, @lift([$cue_P]); color = :dodgerblue, markersize = 10)

    bf_point = Observable(Point2f(NaN, NaN))
    tr_point = Observable(Point2f(NaN, NaN))
    scatter!(ax, @lift([$bf_point]); marker = :xcross, color = :red, markersize = 18,
             label = "T1 força bruta")
    scatter!(ax, @lift([$tr_point]); marker = :circle, color = (:blue, 0.0), strokecolor = :blue,
             strokewidth = 2, markersize = 22, label = "T2 rastreado (EDO)")

    status = Observable("inicializando…")
    Label(fig[2, 1], status; tellwidth = false, fontsize = 15)
    axislegend(ax; position = :rt)

    recompute_obs = Observable(recompute_every)

    on(events(fig.scene).mousebutton) do event
        if event.button == Mouse.left
            if event.action == Mouse.press
                dragging[] = true
                cue_P[] = Point2f(mouseposition(ax))
            elseif event.action == Mouse.release
                dragging[] = false
            end
        end
        return Consume(false)
    end

    on(events(fig.scene).keyboardbutton) do event
        if event.action == Keyboard.press
            if event.key == Keyboard.rightbracket
                recompute_obs[] = min(recompute_obs[] + 5, 600)
            elseif event.key == Keyboard.leftbracket
                recompute_obs[] = max(recompute_obs[] - 5, 1)
            end
        end
        return Consume(false)
    end

    tracker = init_tracker(balls, [cue_P[][1], cue_P[][2]], [cue_d[][1], cue_d[][2]];
                            recompute_every = recompute_obs[])

    t_bf_ema = Ref(0.0)
    t_tr_ema = Ref(0.0)

    on(events(fig.scene).tick) do tick
        dt = max(tick.delta_time, 1e-4)

        for (i, b) in enumerate(balls)
            b.center .+= b.velocity .* dt
            for k in 1:2
                lo = b.radius
                hi = (k == 1 ? width : height) - b.radius
                if b.center[k] < lo
                    b.center[k] = lo
                    b.velocity[k] = abs(b.velocity[k])
                elseif b.center[k] > hi
                    b.center[k] = hi
                    b.velocity[k] = -abs(b.velocity[k])
                end
            end
        end
        centers_obs[] = Point2f[(b.center[1], b.center[2]) for b in balls]

        if dragging[]
            mp = Point2f(mouseposition(ax))
            v = mp .- cue_P[]
            if norm(v) > 1e-6
                cue_d[] = Point2f(normalize(v))
            end
        end
        P = Float64[cue_P[][1], cue_P[][2]]
        d = Float64[cue_d[][1], cue_d[][2]]

        tracker.recompute_every = recompute_obs[]

        t0 = time_ns()
        bf = brute_force_contact(balls, P, d)
        t_bf = time_ns() - t0

        t0 = time_ns()
        tr = step_tracker!(tracker, balls, P, d, dt)
        t_tr = time_ns() - t0

        t_bf_ema[] = 0.9 * t_bf_ema[] + 0.1 * t_bf
        t_tr_ema[] = 0.9 * t_tr_ema[] + 0.1 * t_tr

        bf_point[] = bf === nothing ? Point2f(NaN, NaN) : Point2f(bf[3][1], bf[3][2])
        tr_point[] = tr === nothing ? Point2f(NaN, NaN) : Point2f(tr[3][1], tr[3][2])

        reanchored = tracker.last_used_brute_force
        speedup = t_bf_ema[] / max(t_tr_ema[], 1.0)
        status[] = @sprintf(
            "bolas=%d  recompute_every=%d ('[' / ']' ajusta)  |  T1: %.2f μs   T2: %.2f μs   (T1/T2 ≈ %.1fx)  %s",
            n_balls, recompute_obs[], t_bf / 1e3, t_tr / 1e3, speedup,
            reanchored ? "— reancorado por força bruta neste frame" : ""
        )
    end

    display(fig)
    return fig
end

end # module
