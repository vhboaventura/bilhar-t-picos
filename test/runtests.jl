include(joinpath(@__DIR__, "..", "src", "BilharSim.jl"))
using .BilharSim
using Test
using LinearAlgebra

@testset "brute_force_contact — geometria" begin
    balls = [Ball([5.0, 0.0], 1.0)]
    hit = brute_force_contact(balls, [0.0, 0.0], [1.0, 0.0])
    @test hit !== nothing
    idx, s, pt = hit
    @test idx == 1
    @test isapprox(s, 4.0; atol=1e-10)          # bate na borda da bola em x=4
    @test isapprox(pt, [4.0, 0.0]; atol=1e-10)

    # reta que passa ao largo (não intersecta)
    @test brute_force_contact(balls, [0.0, 5.0], [1.0, 0.0]) === nothing

    # bola atrás do taco não conta (s < s_min)
    @test brute_force_contact([Ball([-5.0, 0.0], 1.0)], [0.0, 0.0], [1.0, 0.0]) === nothing
end

@testset "brute_force_contact — escolhe a bola mais próxima" begin
    balls = [Ball([10.0, 0.0], 1.0), Ball([3.0, 0.0], 1.0)]
    idx, s, pt = brute_force_contact(balls, [0.0, 0.0], [1.0, 0.0])
    @test idx == 2                                # bola mais perto do taco
    @test isapprox(s, 2.0; atol=1e-10)
end

@testset "tracker — taco reto move sem trocar de alvo" begin
    balls = [Ball([5.0, 5.0], 1.0), Ball([2.0, 2.0], 0.5)]
    P, d = [0.0, 0.0], normalize([1.0, 1.0])
    dt = 1 / 60

    tracker = init_tracker(balls, P, d; recompute_every=10_000)
    for step in 1:120
        P = P .+ [0.01, 0.0]
        tracked = step_tracker!(tracker, balls, P, d, dt)
        brute = brute_force_contact(balls, P, d)
        @test tracked !== nothing && brute !== nothing
        @test tracked[1] == brute[1]
        @test isapprox(tracked[2], brute[2]; atol=2e-3)
        @test isapprox(tracked[3], brute[3]; atol=2e-3)
    end
end

@testset "tracker — bolas em movimento (Cⁱ(t) não constante)" begin
    balls = [Ball([6.0, 0.0], 1.0; velocity=[-0.5, 0.2]),
             Ball([3.0, 3.0], 0.8; velocity=[0.0, -0.3])]
    P, d = [0.0, 0.0], normalize([1.0, 0.0])
    dt = 1 / 60

    tracker = init_tracker(balls, P, d; recompute_every=10_000)
    for step in 1:200
        balls[1].center .+= balls[1].velocity .* dt
        balls[2].center .+= balls[2].velocity .* dt
        tracked = step_tracker!(tracker, balls, P, d, dt)
        brute = brute_force_contact(balls, P, d)
        tracked === nothing && continue
        @test tracked[1] == brute[1]
        @test isapprox(tracked[3], brute[3]; atol=5e-3)
    end
end

@testset "tracker — Melhoria 1: recompute_every reancora e detecta troca de alvo" begin
    # Bola 1 começa mais perto; bola 2 se move até ficar mais perto do taco,
    # o rastreador local (travado na bola 1) não percebe a troca sozinho —
    # só a reancoragem periódica por força bruta corrige isso.
    balls = [Ball([4.0, 0.0], 1.0), Ball([9.0, 0.0], 1.0; velocity=[-2.0, 0.0])]
    P, d = [0.0, 0.0], [1.0, 0.0]
    dt = 1 / 60

    tracker = init_tracker(balls, P, d; recompute_every=15)
    @test tracker.ball_index == 1

    switched = false
    for step in 1:300
        balls[2].center .+= balls[2].velocity .* dt
        balls[2].center[1] < 1.5 && (balls[2].center[1] = 1.5)  # não atravessa a bola 1
        tracked = step_tracker!(tracker, balls, P, d, dt)
        if tracked !== nothing && tracked[1] == 2
            switched = true
            break
        end
    end
    @test switched   # a reancoragem periódica eventualmente pega o novo alvo mais próximo
end

@testset "tracker — sem contato (reta não atinge nenhuma bola)" begin
    balls = [Ball([5.0, 10.0], 1.0)]
    P, d = [0.0, 0.0], [1.0, 0.0]
    tracker = init_tracker(balls, P, d; recompute_every=5)
    @test tracker.ball_index == 0
    r = step_tracker!(tracker, balls, P, d, 1/60)
    @test r === nothing
end
