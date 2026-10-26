module Server

# Backend web: o servidor Julia guarda a simulação (bolas + rastreador T2) e
# roda os algoritmos 1 e 2; a página em web/ (HTML + canvas) só desenha e envia
# a posição do taco e os controles a cada frame.

using HTTP
using JSON3
using LinearAlgebra
using Random
include(joinpath(@__DIR__, "BilharSim.jl"))
using .BilharSim

const WEB_DIR = normpath(joinpath(@__DIR__, "..", "web"))
const MIME = Dict(".html" => "text/html; charset=utf-8", ".js" => "text/javascript; charset=utf-8",
                  ".css" => "text/css; charset=utf-8", ".png" => "image/png",
                  ".svg" => "image/svg+xml", ".ico" => "image/x-icon")

# ---------------------------------------------------------------------------
# Simulação
# ---------------------------------------------------------------------------

mutable struct Sim
    table::Table
    balls::Vector{Ball}
    tracker::TrackerState
    rng::AbstractRNG
    ball_radius::Float64
    P::Vector{Float64}
    d::Vector{Float64}
    frame::Int
    ema_bf::Float64
    ema_tr::Float64
    recent_bf::Vector{Bool}
    lock::ReentrantLock
end

function place!(sim::Sim, b::Ball)
    r = b.radius
    w, h = sim.table.width, sim.table.height
    for _ in 1:80
        c = [r + rand(sim.rng) * (w - 2r), r + rand(sim.rng) * (h - 2r)]
        b.center .= c
        all(o -> o === b || norm(o.center .- c) > o.radius + r + 0.5, sim.balls) && break
    end
    ang = 2π * rand(sim.rng)
    speed = 10 + 18 * rand(sim.rng)
    b.velocity .= speed .* [cos(ang), sin(ang)]
    return b
end

function spawn!(sim::Sim)
    b = Ball([0.0, 0.0], sim.ball_radius)
    push!(sim.balls, b)
    place!(sim, b)
end

function reset_tracker!(sim::Sim)
    old = sim.tracker
    sim.tracker = init_tracker(sim.balls, sim.P, sim.d; recompute_every = old.recompute_every,
                               predictor = old.predictor, newton_iters = old.newton_iters)
    fill!(sim.recent_bf, false)
    return sim
end

function new_sim(; n_balls::Int = 15, width::Float64 = 200.0, height::Float64 = 100.0,
                  ball_radius::Float64 = 6.0, seed::Union{Int,Nothing} = nothing)
    rng = seed === nothing ? Random.default_rng() : MersenneTwister(seed)
    table = Table(width, height)
    P, d = [-1.5, height / 2], [1.0, 0.0]
    sim = Sim(table, Ball[], init_tracker(Ball[], P, d), rng, ball_radius, P, d, 0, 0.0, 0.0,
              fill(false, 120), ReentrantLock())
    for _ in 1:n_balls
        spawn!(sim)
    end
    reset_tracker!(sim)
    return sim
end

function configure!(sim::Sim; n_balls = nothing, radius = nothing, shuffle::Bool = false)
    if radius !== nothing
        sim.ball_radius = Float64(radius)
        foreach(b -> b.radius = sim.ball_radius, sim.balls)
    end
    if n_balls !== nothing
        n = clamp(Int(n_balls), 1, 300)
        while length(sim.balls) < n
            spawn!(sim)
        end
        while length(sim.balls) > n
            pop!(sim.balls)
        end
    end
    shuffle && foreach(b -> place!(sim, b), sim.balls)
    return reset_tracker!(sim)
end

ema(old, x) = old == 0.0 ? x : 0.96 * old + 0.04 * x
r3(x) = round(x; digits = 3)

contact_json(hit) = hit === nothing ? nothing :
    (idx = hit[1] - 1, s = hit[2], x = hit[3][1], y = hit[3][2])

"""
    tick!(sim, dt, P, d, paused, predictor, newton_iters, recompute_every)

Avança a simulação um frame, roda T1 e T2 com a mesma entrada e devolve o
estado para a página desenhar.
"""
function tick!(sim::Sim, dt, P, d, paused::Bool, predictor::Symbol, newton_iters::Int,
               recompute_every::Int)
    dt = clamp(dt, 1e-4, 0.05)
    sim.frame += 1
    paused || advance_balls!(sim.balls, sim.table, dt; collide = true)

    tr = sim.tracker
    tr.predictor = predictor
    tr.newton_iters = clamp(newton_iters, 0, 50)
    tr.recompute_every = max(recompute_every, 0)
    sim.P .= P
    sim.d .= d

    t0 = time_ns()
    bf = brute_force_contact(sim.balls, sim.P, sim.d)
    ns_bf = time_ns() - t0

    saved = nothing
    if paused   # bolas paradas: o preditor não pode usar a velocidade como C′
        saved = [copy(b.velocity) for b in sim.balls]
        foreach(b -> fill!(b.velocity, 0.0), sim.balls)
    end
    t0 = time_ns()
    res = step_tracker!(tr, sim.balls, sim.P, sim.d, dt)
    ns_tr = time_ns() - t0
    if saved !== nothing
        for (b, v) in zip(sim.balls, saved)
            b.velocity .= v
        end
    end

    if sim.frame > 3
        sim.ema_bf = ema(sim.ema_bf, ns_bf)
        sim.ema_tr = ema(sim.ema_tr, ns_tr)
    end
    sim.recent_bf[mod1(sim.frame, length(sim.recent_bf))] = tr.last_used_brute_force
    nrecent = min(sim.frame, length(sim.recent_bf))

    same = (bf === nothing && res === nothing) ||
           (bf !== nothing && res !== nothing && bf[1] == res[1])
    err = (same && bf !== nothing) ? abs(res[2] - bf[2]) : nothing

    flat = Float64[]
    sizehint!(flat, 3 * length(sim.balls))
    for b in sim.balls
        push!(flat, r3(b.center[1]), r3(b.center[2]), b.radius)
    end

    return (balls = flat, t1 = contact_json(bf), t2 = contact_json(res), same = same, err = err,
            ns_bf = ns_bf, ns_tr = ns_tr, ema_bf = sim.ema_bf, ema_tr = sim.ema_tr,
            anchored = tr.last_used_brute_force, newton_last = tr.last_newton_iters,
            newton_total = tr.newton_iters_total, frames = tr.frame,
            bf_calls = tr.brute_force_calls,
            recent_bf_pct = 100 * count(view(sim.recent_bf, 1:nrecent)) / nrecent)
end

# ---------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------

json_response(obj; status = 200) =
    HTTP.Response(status, ["Content-Type" => "application/json"], JSON3.write(obj))

function static_response(path::AbstractString)
    rel = path == "/" ? "index.html" : lstrip(path, '/')
    file = normpath(joinpath(WEB_DIR, rel))
    # só serve arquivos dentro de web/
    (startswith(file, WEB_DIR) && isfile(file)) || return HTTP.Response(404, "não encontrado")
    mime = get(MIME, lowercase(splitext(file)[2]), "application/octet-stream")
    return HTTP.Response(200, ["Content-Type" => mime, "Cache-Control" => "no-store"], read(file))
end

function make_handler(sim::Sim)
    return function (req::HTTP.Request)
        try
            path = HTTP.URI(req.target).path
            if req.method == "POST" && path == "/api/tick"
                q = JSON3.read(String(req.body))
                predictor = Symbol(get(q, :predictor, "euler"))
                predictor in PREDICTORS || return json_response((error = "predictor inválido",); status = 400)
                P = Float64[q.P[1], q.P[2]]
                d = Float64[q.d[1], q.d[2]]
                out = lock(sim.lock) do
                    tick!(sim, Float64(get(q, :dt, 1 / 60)), P, d, Bool(get(q, :paused, false)),
                          predictor, Int(get(q, :newton_iters, 10)), Int(get(q, :recompute_every, 30)))
                end
                return json_response(out)
            elseif req.method == "POST" && path == "/api/config"
                q = JSON3.read(String(req.body))
                lock(sim.lock) do
                    configure!(sim; n_balls = get(q, :n_balls, nothing), radius = get(q, :radius, nothing),
                               shuffle = Bool(get(q, :shuffle, false)))
                end
                return json_response((ok = true, n_balls = length(sim.balls)))
            elseif req.method == "GET" && path == "/api/info"
                return json_response((width = sim.table.width, height = sim.table.height,
                                      n_balls = length(sim.balls), radius = sim.ball_radius))
            elseif req.method == "GET"
                return static_response(path)
            end
            return HTTP.Response(405, "método não permitido")
        catch err
            @error "erro no servidor" exception = (err, catch_backtrace())
            return json_response((error = sprint(showerror, err),); status = 500)
        end
    end
end

"""
    start(; host="127.0.0.1", port=8080, open_browser=true, n_balls=15, seed=nothing)

Sobe o servidor e (opcionalmente) abre o navegador. Devolve o servidor
(`close(server)` para parar).
"""
function start(; host::AbstractString = "127.0.0.1", port::Integer = 8080,
                open_browser::Bool = true, n_balls::Int = 15, seed::Union{Int,Nothing} = nothing)
    sim = new_sim(; n_balls, seed)
    # aquecimento: compila os caminhos quentes antes do primeiro frame real
    tick!(sim, 1 / 60, [-1.5, 50.0], [1.0, 0.0], false, :euler, 10, 30)
    configure!(sim)
    sim.frame = 0
    sim.ema_bf = sim.ema_tr = 0.0
    server = HTTP.serve!(make_handler(sim), host, port)
    url = "http://$(host == "0.0.0.0" ? "localhost" : host):$port"
    @info "Servidor no ar" url
    if open_browser
        try
            if Sys.iswindows()
                run(`cmd /c start "" $url`; wait = false)
            elseif Sys.isapple()
                run(`open $url`; wait = false)
            else
                run(`xdg-open $url`; wait = false)
            end
        catch
            @info "Abra o endereço acima no navegador."
        end
    end
    return server
end

end # module
