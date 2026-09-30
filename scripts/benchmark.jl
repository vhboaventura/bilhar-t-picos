#!/usr/bin/env julia
# Compara o custo por frame do algoritmo 1 (força bruta, T1) contra o
# algoritmo 2 (rastreamento via EDO, T2) em função do número de bolas na
# mesa, para vários valores de `recompute_every` (Melhoria 1).
#
# Uso:
#   julia --project=. scripts/benchmark.jl

include(joinpath(@__DIR__, "..", "src", "BilharSim.jl"))
using .BilharSim
using BenchmarkTools
using LinearAlgebra
using Printf
using Random

Random.seed!(42)

function make_table(n_balls::Int; width = 200.0, height = 100.0)
    balls = Ball[]
    for _ in 1:n_balls
        c = [10 + rand() * (width - 20), 10 + rand() * (height - 20)]
        v = (rand(2) .- 0.5) .* 4.0
        push!(balls, Ball(c, 1.0; velocity = v))
    end
    return balls
end

function bench_brute_force(balls, n_frames)
    P, d = [0.0, height_mid(balls)], normalize([1.0, 0.05])
    dt = 1 / 60
    b = @benchmark begin
        for _ in 1:$n_frames
            brute_force_contact($balls, $P, $d)
        end
    end samples = 20 evals = 1
    return time(minimum(b)) / n_frames  # ns por frame
end

function bench_tracker(balls, n_frames, recompute_every)
    P, d = [0.0, height_mid(balls)], normalize([1.0, 0.05])
    dt = 1 / 60
    b = @benchmark begin
        tracker = init_tracker($balls, $P, $d; recompute_every = $recompute_every)
        p = copy($P)
        for _ in 1:$n_frames
            p = p .+ [0.01, 0.0]
            step_tracker!(tracker, $balls, p, $d, $dt)
        end
    end samples = 20 evals = 1
    return time(minimum(b)) / n_frames  # ns por frame
end

height_mid(balls) = 50.0

function main()
    n_frames = 500
    ball_counts = [5, 20, 50, 100, 300]
    recompute_options = [10, 30, 60]

    println("Comparação de custo médio por frame — algoritmo 1 (força bruta) vs algoritmo 2 (EDO)")
    println(repeat("-", 88))
    @printf("%-10s %-16s", "bolas", "T1 força bruta")
    for k in recompute_options
        @printf(" %-22s", "T2 (recompute=$k)")
    end
    println()

    for n in ball_counts
        balls = make_table(n)
        t1_ns = bench_brute_force(balls, n_frames)
        @printf("%-10d %-16s", n, "$(round(t1_ns, digits=1)) ns")
        for k in recompute_options
            t2_ns = bench_tracker(balls, n_frames, k)
            speedup = t1_ns / t2_ns
            @printf(" %-22s", "$(round(t2_ns, digits=1)) ns (×$(round(speedup, digits=1)))")
        end
        println()
    end

    println()
    println("Observação: o custo de T1 cresce linearmente com o número de bolas (O(n)")
    println("interseções reta-círculo por frame). T2 é O(1) por frame na maioria dos")
    println("frames (apenas 1 predição de Euler + 1 correção de Newton sobre a bola já")
    println("travada) e só paga o custo de T1 nos frames de reancoragem, a cada")
    println("`recompute_every` frames — por isso o ganho relativo cresce com o número")
    println("de bolas e com `recompute_every`.")
end

main()
