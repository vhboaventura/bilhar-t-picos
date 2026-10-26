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

# Cenário comum: taco girando e transladando, bolas em movimento.
function run_scenario(predictor; newton_iters = 10, recompute_every = 0, frames = 300)
    balls = [Ball([30.0, 0.0], 6.0; velocity = [-1.0, 0.3]),
             Ball([60.0, 25.0], 4.0; velocity = [1.0, -1.5]),
             Ball([45.0, -20.0], 3.5; velocity = [0.5, 2.0])]
    dt = 1 / 60
    P = [0.0, 0.0]
    tracker = init_tracker(balls, P, [1.0, 0.0]; recompute_every, predictor, newton_iters)
    max_err = 0.0
    for k in 1:frames
        for b in balls
            b.center .+= b.velocity .* dt
        end
        θ = 0.08 * sin(k * dt * 1.5)
        d = [cos(θ), sin(θ)]
        P = [0.0, 2.0 * sin(k * dt)]
        tracked = step_tracker!(tracker, balls, P, d, dt)
        brute = brute_force_contact(balls, P, d)
        if tracked !== nothing && brute !== nothing && tracked[1] == brute[1]
            max_err = max(max_err, abs(tracked[2] - brute[2]))
        end
    end
    return tracker, max_err
end

@testset "tracker — preditor de ordem 0 + corretor de Newton" begin
    tracker, err = run_scenario(:order0)
    @test err < 1e-8                      # Newton iterado até convergir: erro ~ tolerância
    @test tracker.brute_force_calls == 1  # nunca precisou recalcular tudo (só a inicialização)
end

@testset "tracker — preditor de 1ª ordem (Euler) + corretor de Newton" begin
    tracker, err = run_scenario(:euler)
    @test err < 1e-8
    @test tracker.brute_force_calls == 1
end

@testset "tracker — Euler é melhor preditor: menos iterações de Newton que ordem 0" begin
    t0, _ = run_scenario(:order0)
    t1, _ = run_scenario(:euler)
    @test t1.newton_iters_total < t0.newton_iters_total
end

@testset "tracker — só preditor (0 iterações de Newton): Euler deriva menos que ordem 0" begin
    _, err0 = run_scenario(:order0; newton_iters = 0)
    _, err1 = run_scenario(:euler;  newton_iters = 0)
    @test err1 < err0 / 10                # 1ª ordem deriva muito menos que ordem 0
    @test err1 < 0.1                      # e sozinha ainda acompanha o contato de perto
end

@testset "tracker — com 1 passo de Newton só o preditor de Euler basta" begin
    t_e, err_e = run_scenario(:euler;  newton_iters = 1)
    t_0, _     = run_scenario(:order0; newton_iters = 1)
    @test err_e < 1e-6
    @test t_e.brute_force_calls == 1                       # Euler+1 Newton nunca precisa recalcular tudo
    @test t_0.brute_force_calls > t_e.brute_force_calls    # ordem 0 + 1 Newton não converge e reancora
end

@testset "tracker — Melhoria 1: recompute_every reancora e detecta troca de alvo" begin
    # Bola 1 começa mais perto; bola 2 se move até ficar mais perto do taco.
    # O rastreador local (travado na bola 1) não percebe a troca sozinho —
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
    @test switched
end

@testset "tracker — corretor falha (reta sai da bola) e cai em força bruta" begin
    balls = [Ball([10.0, 0.0], 1.0)]
    tracker = init_tracker(balls, [0.0, 0.0], [1.0, 0.0]; recompute_every=0)
    @test tracker.ball_index == 1
    # taco gira para longe: a reta deixa de intersectar a bola
    r = step_tracker!(tracker, balls, [0.0, 0.0], normalize([1.0, 1.0]), 1/60)
    @test r === nothing
    @test tracker.last_used_brute_force
end

@testset "tracker — sem contato (reta não atinge nenhuma bola)" begin
    balls = [Ball([5.0, 10.0], 1.0)]
    P, d = [0.0, 0.0], [1.0, 0.0]
    tracker = init_tracker(balls, P, d; recompute_every=5)
    @test tracker.ball_index == 0
    r = step_tracker!(tracker, balls, P, d, 1/60)
    @test r === nothing
end

@testset "init_tracker rejeita preditor desconhecido" begin
    @test_throws ArgumentError init_tracker([Ball([5.0, 0.0], 1.0)], [0.0, 0.0], [1.0, 0.0];
                                             predictor = :rk4)
end

# ---------------------------------------------------------------------------
# Servidor web (backend Julia da visualização)
# ---------------------------------------------------------------------------
include(joinpath(@__DIR__, "..", "src", "Server.jl"))
using HTTP, JSON3

@testset "Server — simulação: T2 acompanha T1 e rejeita configuração inválida" begin
    sim = Server.new_sim(n_balls = 15, seed = 3)
    P, d = [-1.5, 50.0], [1.0, 0.0]
    errs = Float64[]
    for k in 1:300
        out = Server.tick!(sim, 1 / 60, P, d, false, :euler, 10, 30)
        @test length(out.balls) == 3 * 15
        out.same && out.err !== nothing && push!(errs, out.err)
    end
    @test !isempty(errs)
    @test maximum(errs) < 1e-5

    Server.configure!(sim; n_balls = 40, radius = 4.0)
    @test length(sim.balls) == 40
    @test all(b -> b.radius == 4.0, sim.balls)
    Server.configure!(sim; n_balls = 5)
    @test length(sim.balls) == 5
end

@testset "Server — API HTTP" begin
    port = 18_765
    server = Server.start(; port = port, open_browser = false, n_balls = 10, seed = 1)
    try
        base = "http://127.0.0.1:$port"
        info = JSON3.read(String(HTTP.get("$base/api/info").body))
        @test info.n_balls == 10 && info.width == 200

        body = JSON3.write((dt = 0.016, P = [-1.5, 50.0], d = [1.0, 0.0], paused = false,
                            predictor = "order0", newton_iters = 10, recompute_every = 30))
        out = JSON3.read(String(HTTP.post("$base/api/tick", ["Content-Type" => "application/json"], body).body))
        @test length(out.balls) == 30
        @test haskey(out, :t1) && haskey(out, :t2) && haskey(out, :same)

        bad = JSON3.write((dt = 0.016, P = [0.0, 0.0], d = [1.0, 0.0], predictor = "rk4"))
        r = HTTP.post("$base/api/tick", ["Content-Type" => "application/json"], bad; status_exception = false)
        @test r.status == 400

        cfg = JSON3.read(String(HTTP.post("$base/api/config", [], JSON3.write((n_balls = 25, radius = 5.0))).body))
        @test cfg.n_balls == 25

        page = HTTP.get("$base/")
        @test page.status == 200 && occursin("Taco", String(page.body))
        @test HTTP.get("$base/..%2F..%2FProject.toml"; status_exception = false).status == 404
        @test HTTP.get("$base/nao-existe.js"; status_exception = false).status == 404
    finally
        close(server)
    end
end
