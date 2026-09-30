#!/usr/bin/env julia
# Abre a visualização interativa. Uso: julia --project=. scripts/run_app.jl

include(joinpath(@__DIR__, "..", "src", "App.jl"))
using .App
using GLMakie: events

fig = App.run_app(n_balls = 12, recompute_every = 30, min_radius = 5.0, max_radius = 8.0)
while events(fig.scene).window_open[]
    sleep(0.1)
end
