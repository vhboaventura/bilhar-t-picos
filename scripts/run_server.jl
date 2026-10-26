#!/usr/bin/env julia
# Sobe o servidor e abre a visualização no navegador (http://localhost:8080).
# Uso: julia --project=. scripts/run_server.jl [porta]

include(joinpath(@__DIR__, "..", "src", "Server.jl"))
using .Server

port = isempty(ARGS) ? 8080 : parse(Int, ARGS[1])
server = Server.start(; port)
println("Pressione Ctrl+C para encerrar.")
try
    wait(server)
catch e
    e isa InterruptException || rethrow()
finally
    close(server)
end
