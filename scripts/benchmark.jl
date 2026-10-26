#!/usr/bin/env julia
# Benchmarks do algoritmo 1 (força bruta, T1) contra o algoritmo 2 (T2:
# preditor de ordem 0 ou de 1ª ordem/Euler + corretor de Newton), com e sem
# reancoragem periódica (Melhoria 1).
#
# Dois experimentos:
#   A) escala: custo por frame, speedup, acurácia e iterações de Newton em
#      função do número de bolas.
#   B) Newton: erro de T2 em função do número máximo de iterações de Newton,
#      para cada preditor (mostra a qualidade do preditor).
#
# Saídas: tabelas no terminal, CSVs, gráficos SVG e um relatório HTML em
# docs/benchmark/.
#
# Uso:  julia --project=. scripts/benchmark.jl

include(joinpath(@__DIR__, "..", "src", "BilharSim.jl"))
using .BilharSim
using LinearAlgebra
using Printf
using Random
using Dates
include(joinpath(@__DIR__, "report.jl"))

const DT = 1 / 60
const FRAMES = 300
const OUT_DIR = joinpath(@__DIR__, "..", "docs", "benchmark")

# Mesa com densidade constante de bolas (≈ uma bola por 20x20), proporção 2:1.
function make_scenario(n_balls::Int; seed = 42)
    rng = MersenneTwister(seed)
    h = sqrt(n_balls * 400 / 2)
    table = Table(2h, h)
    balls = [Ball([3 + rand(rng) * (2h - 6), 3 + rand(rng) * (h - 6)], 3.0;
                   velocity = (rand(rng, 2) .- 0.5) .* 5.0) for _ in 1:n_balls]
    return table, balls
end

# Pré-calcula a trajetória: um snapshot das bolas (e do taco) por frame, para
# que todas as configurações vejam exatamente a mesma sequência e a medição
# de tempo não precise descontar custo de carregar frames.
function record_trajectory(n_balls::Int)
    table, balls = make_scenario(n_balls)
    snaps = Vector{Vector{Ball}}()
    cues = Vector{Tuple{Vector{Float64},Vector{Float64}}}()
    for k in 1:FRAMES
        advance_balls!(balls, table, DT)
        θ = 0.35 * sin(k * DT * 0.5)
        push!(cues, ([4.0, table.height / 2], [cos(θ), sin(θ)]))
        push!(snaps, [Ball(copy(b.center), b.radius; velocity = copy(b.velocity)) for b in balls])
    end
    return snaps, cues
end

# Tempo (ns/frame) de uma passada completa pelos frames; mínimo de várias repetições.
function time_pass(f, snaps, cues; reps = 40)
    best = Inf
    GC.gc()
    GC.enable(false)               # sem pausas de coleta de lixo dentro da medição
    try
        for _ in 1:reps
            t0 = time_ns()
            for k in 1:FRAMES
                f(k, snaps[k], cues[k][1], cues[k][2])
            end
            best = min(best, (time_ns() - t0) / FRAMES)
        end
    finally
        GC.enable(true)
    end
    return best
end

function reference(snaps, cues)
    ref = Vector{Union{Nothing,Tuple{Int,Float64}}}(undef, FRAMES)
    for k in 1:FRAMES
        r = brute_force_contact(snaps[k], cues[k][1], cues[k][2])
        ref[k] = r === nothing ? nothing : (r[1], r[2])
    end
    return ref
end

# Acurácia e contadores de uma configuração de T2 (uma passada comparando com T1).
function accuracy_pass(snaps, cues, ref, predictor, rec, iters)
    tracker = init_tracker(snaps[1], cues[1][1], cues[1][2];
                           recompute_every = rec, predictor, newton_iters = iters)
    max_err = 0.0
    matches = 0
    for k in 2:FRAMES
        r = step_tracker!(tracker, snaps[k], cues[k][1], cues[k][2], DT)
        a = ref[k]
        if (r === nothing && a === nothing) || (r !== nothing && a !== nothing && r[1] == a[1])
            matches += 1
            r !== nothing && (max_err = max(max_err, abs(r[2] - a[2])))
        end
    end
    nframes = max(tracker.frame - (tracker.brute_force_calls - 1), 1)
    return (max_err = max_err, target_ok = 100 * matches / (FRAMES - 1),
            fallbacks = tracker.brute_force_calls - 1,
            newton_pf = tracker.newton_iters_total / nframes)
end

function time_t2(snaps, cues, predictor, rec, iters)
    tr = Ref(init_tracker(snaps[1], cues[1][1], cues[1][2];
                          recompute_every = rec, predictor, newton_iters = iters))
    run = function (k, balls, P, d)
        if k == 1
            tr[] = init_tracker(balls, P, d; recompute_every = rec, predictor, newton_iters = iters)
            return nothing
        end
        return step_tracker!(tr[], balls, P, d, DT)
    end
    time_pass(run, snaps, cues; reps = 2)   # aquecimento
    return time_pass(run, snaps, cues)
end

fmt_ns(t) = t >= 1e6 ? @sprintf("%g ms", t / 1e6) : t >= 1e3 ? @sprintf("%g µs", t / 1e3) : @sprintf("%g ns", t)

function main()
    ns_list = [10, 20, 50, 100, 200, 500, 1000]
    configs = [
        (key = :o0,  name = "T2 ordem 0, sem reancorar", predictor = :order0, rec = 0),
        (key = :eu,  name = "T2 Euler, sem reancorar",   predictor = :euler,  rec = 0),
        (key = :o0r, name = "T2 ordem 0, reancora/30",   predictor = :order0, rec = 30),
        (key = :eur, name = "T2 Euler, reancora/30",     predictor = :euler,  rec = 30),
    ]
    colors = Dict(:t1 => "#e5484d", :o0 => "#f59e0b", :eu => "#0ea5b7", :o0r => "#8b5cf6", :eur => "#10b981")

    println("Benchmarks T1 (força bruta) × T2 (preditor + corretor de Newton)")
    println("$FRAMES frames por cenário, dt = 1/60 s, densidade de bolas constante.\n")

    # ------------------------------------------------------------------ A) escala
    rowsA = NamedTuple[]
    for n in ns_list
        snaps, cues = record_trajectory(n)
        ref = reference(snaps, cues)
        brute = (k, balls, P, d) -> brute_force_contact(balls, P, d)
        time_pass(brute, snaps, cues; reps = 2)
        t1 = time_pass(brute, snaps, cues)
        push!(rowsA, (n = n, key = :t1, name = "T1 força bruta", ns = t1, speedup = 1.0, max_err = 0.0,
                      target_ok = 100.0, fallbacks = FRAMES, newton_pf = 0.0))
        println("== $n bolas ", repeat("=", 70))
        @printf("%-28s %10s %9s %10s %9s %9s %8s\n", "algoritmo", "ns/frame", "speedup", "máx |Δs|",
                "alvo ok", "fallbacks", "Newton/f")
        @printf("%-28s %10.0f %9s %10s %9s %9d %8s\n", "T1 força bruta", t1, "1.0×", "—", "100%", FRAMES, "—")
        for c in configs
            acc = accuracy_pass(snaps, cues, ref, c.predictor, c.rec, 10)
            t2 = time_t2(snaps, cues, c.predictor, c.rec, 10)
            push!(rowsA, (n = n, key = c.key, name = c.name, ns = t2, speedup = t1 / t2, acc...))
            @printf("%-28s %10.0f %8.1f× %10.1e %8.1f%% %9d %8.2f\n", c.name, t2, t1 / t2, acc.max_err,
                    acc.target_ok, acc.fallbacks, acc.newton_pf)
        end
        println()
    end

    # ------------------------------------------------------------------ B) Newton
    iters_list = [0, 1, 2, 3, 5, 10]
    snaps, cues = record_trajectory(100)
    ref = reference(snaps, cues)
    rowsB = NamedTuple[]
    println("== Erro × iterações de Newton (100 bolas, sem reancoragem periódica) ", repeat("=", 20))
    @printf("%-10s %6s %12s %10s %9s\n", "preditor", "iters", "máx |Δs|", "fallbacks", "Newton/f")
    for pred in (:order0, :euler), it in iters_list
        acc = accuracy_pass(snaps, cues, ref, pred, 0, it)
        push!(rowsB, (predictor = pred, iters = it, acc...))
        @printf("%-10s %6d %12.2e %10d %9.2f\n", pred, it, acc.max_err, acc.fallbacks, acc.newton_pf)
    end
    println()

    # ------------------------------------------------------------------ saídas
    mkpath(OUT_DIR)
    open(joinpath(OUT_DIR, "escala.csv"), "w") do io
        println(io, "n_bolas,algoritmo,ns_por_frame,speedup,erro_max_s,alvo_correto_pct,fallbacks_forca_bruta,newton_por_frame")
        for r in rowsA
            println(io, join([r.n, "\"" * r.name * "\"", round(r.ns; digits = 1), round(r.speedup; digits = 2), r.max_err,
                              round(r.target_ok; digits = 1), r.fallbacks, round(r.newton_pf; digits = 3)], ","))
        end
    end
    open(joinpath(OUT_DIR, "newton.csv"), "w") do io
        println(io, "preditor,iteracoes_newton,erro_max_s,alvo_correto_pct,fallbacks_forca_bruta,newton_por_frame")
        for r in rowsB
            println(io, join([r.predictor, r.iters, r.max_err, round(r.target_ok; digits = 1), r.fallbacks,
                              round(r.newton_pf; digits = 3)], ","))
        end
    end

    pick(key) = [r for r in rowsA if r.key == key]
    xs = ns_list
    ns_fmt = t -> t >= 1e3 ? @sprintf("%g µs", t / 1e3) : @sprintf("%g ns", t)
    s1 = line_chart(joinpath(OUT_DIR, "tempo_por_frame.svg");
        title = "Custo por frame × número de bolas",
        subtitle = "T1 cresce linearmente com n; T2 só paga O(n) nas reancoragens. Menor é melhor.",
        xlabel = "número de bolas", ylabel = "tempo por frame (escala log)", xs = xs, yfmt = ns_fmt,
        series = vcat([(name = "T1 força bruta", color = colors[:t1], ys = [r.ns for r in pick(:t1)])],
                      [(name = c.name, color = colors[c.key], ys = [r.ns for r in pick(c.key)]) for c in configs]))
    s2 = bar_chart(joinpath(OUT_DIR, "speedup.svg");
        title = "Speedup de T2 sobre T1", subtitle = "custo de T1 ÷ custo de T2 (escala log). Maior é melhor.",
        xlabel = "número de bolas", ylabel = "speedup (×)", ylog = true,
        groups = ["$n" for n in xs], yfmt = t -> @sprintf("%g×", t), vfmt = v -> @sprintf("%.1f×", v),
        series = [(name = c.name, color = colors[c.key], ys = [r.speedup for r in pick(c.key)]) for c in configs])
    s3 = bar_chart(joinpath(OUT_DIR, "newton_por_frame.svg");
        title = "Iterações de Newton por frame: ordem 0 × Euler",
        subtitle = "Sem reancoragem periódica. O preditor de 1ª ordem chega mais perto da raiz e precisa de menos iterações.",
        xlabel = "número de bolas", ylabel = "iterações de Newton por frame",
        groups = ["$n" for n in xs], vfmt = v -> @sprintf("%.2f", v), yfmt = t -> @sprintf("%g", t),
        series = [(name = "Preditor de ordem 0", color = colors[:o0], ys = [r.newton_pf for r in pick(:o0)]),
                  (name = "Preditor de Euler (1ª ordem)", color = colors[:eu], ys = [r.newton_pf for r in pick(:eu)])])
    floor_err(x) = max(x, 1e-16)
    s4 = bar_chart(joinpath(OUT_DIR, "erro_vs_newton.svg");
        title = "Erro de T2 × iterações de Newton (100 bolas)",
        subtitle = "Erro máximo em s contra T1 nos frames em que T2 usou o corretor (fallback para força bruta é exato). Menor é melhor.",
        xlabel = "máximo de iterações de Newton por frame (0 = só preditor)", ylabel = "erro máximo |Δs| (escala log)",
        ylog = true, groups = ["$i" for i in iters_list],
        yfmt = t -> @sprintf("%.0e", t), vfmt = v -> v <= 1.01e-16 ? "0" : @sprintf("%.0e", v),
        series = [(name = "Preditor de ordem 0", color = colors[:o0],
                   ys = [floor_err(r.max_err) for r in rowsB if r.predictor == :order0]),
                  (name = "Preditor de Euler (1ª ordem)", color = colors[:eu],
                   ys = [floor_err(r.max_err) for r in rowsB if r.predictor == :euler])])
    s5 = bar_chart(joinpath(OUT_DIR, "alvo_correto.svg");
        title = "Frames em que T2 acerta a bola-alvo",
        subtitle = "Sem reancoragem periódica o rastreio local não percebe troca de alvo; reancorar a cada 30 frames corrige. Maior é melhor.",
        xlabel = "número de bolas", ylabel = "frames com alvo correto (%)",
        groups = ["$n" for n in xs], yfmt = t -> @sprintf("%g%%", t), vfmt = v -> @sprintf("%.0f", v),
        series = [(name = c.name, color = colors[c.key], ys = [r.target_ok for r in pick(c.key)]) for c in configs])

    meta = "Julia $(VERSION) · $(Sys.cpu_info()[1].model) · $(Sys.CPU_THREADS) threads · $(Sys.KERNEL) · " *
           "$(Dates.format(now(), "yyyy-mm-dd HH:MM")) · $FRAMES frames por cenário, dt = 1/60 s"
    tableA = html_table(["bolas", "algoritmo", "tempo/frame", "speedup", "máx |Δs|", "alvo correto", "fallbacks", "Newton/frame"],
        [[r.n, r.name, fmt_ns(r.ns), r.key == :t1 ? "1.0×" : @sprintf("%.1f×", r.speedup),
          r.key == :t1 ? "—" : @sprintf("%.1e", r.max_err), @sprintf("%.1f%%", r.target_ok), r.fallbacks,
          r.key == :t1 ? "—" : @sprintf("%.2f", r.newton_pf)] for r in rowsA])
    tableB = html_table(["preditor", "iterações de Newton", "máx |Δs|", "fallbacks (força bruta)", "Newton/frame"],
        [[r.predictor == :order0 ? "ordem 0" : "Euler (1ª ordem)", r.iters, @sprintf("%.2e", r.max_err), r.fallbacks,
          @sprintf("%.2f", r.newton_pf)] for r in rowsB])
    write_html_report(joinpath(OUT_DIR, "relatorio.html"); meta,
        sections = [
            ("1. Custo por frame", "O custo do T1 cresce com o número de bolas (uma interseção reta-círculo por bola, por frame). O T2 trabalha só na bola já travada e só recalcula tudo nas reancoragens.", s1, tableA),
            ("2. Speedup", "Quantas vezes o T2 é mais rápido que o T1. O ganho cresce com o número de bolas.", s2, ""),
            ("3. Qualidade do preditor", "Com o preditor de Euler o chute inicial está mais perto da raiz, então o corretor de Newton converge em menos iterações que com o preditor de ordem 0.", s3, ""),
            ("4. Erro × iterações de Newton", "Só com o preditor (0 iterações) a deriva do ordem 0 é muito maior que a do Euler. Com 1–3 iterações o corretor ainda não passa no teste de resíduo em boa parte dos frames e o T2 cai em força bruta (fallback): isso acontece muito mais com o ordem 0 (ver a tabela). Com 5 iterações ou mais ambos chegam à raiz com erro ~1e-14.", s4, tableB),
            ("5. Acerto do alvo", "O rastreio local só segue a bola travada. Trocas de alvo são detectadas quando o corretor falha ou na reancoragem periódica (Melhoria 1).", s5, ""),
        ])

    println("Arquivos gerados em $(normpath(OUT_DIR)):")
    for f in sort(readdir(OUT_DIR))
        println("  ", f)
    end
end

main()
