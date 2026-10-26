# Gráficos em SVG e relatório HTML do benchmark, sem dependências externas.
# Usado por scripts/benchmark.jl.

const INK = "#1f2937"
const MUTED = "#6b7280"
const GRID = "#e5e7eb"

esc(s) = replace(string(s), "&" => "&amp;", "<" => "&lt;", ">" => "&gt;")
f1(x) = @sprintf("%.1f", x)

function decade_ticks(lo, hi)
    a = floor(Int, log10(lo))
    b = ceil(Int, log10(hi))
    b == a && (b += 1)
    return [10.0^k for k in a:b]
end

function nice_linear_max(v)
    v <= 0 && return 1.0
    p = 10.0^floor(log10(v))
    for m in (1, 2, 2.5, 5, 10)
        m * p >= v && return m * p
    end
    return 10p
end

function legend_layout(items, W; x0 = 24)
    pos = Tuple{Float64,Int}[]
    cx = float(x0)
    row = 0
    for (name, _) in items
        w = 30 + 7.0 * length(name)
        if cx + w > W - 24 && cx > x0
            cx = float(x0)
            row += 1
        end
        push!(pos, (cx, row))
        cx += w + 12
    end
    return pos, row + 1
end

function legend_svg(io, items, pos, y0)
    for ((name, color), (cx, row)) in zip(items, pos)
        y = y0 + 22 * row
        print(io, "<rect x=\"$cx\" y=\"$(y - 9)\" width=\"14\" height=\"14\" rx=\"3\" fill=\"$color\"/>")
        print(io, "<text x=\"$(cx + 20)\" y=\"$(y + 3)\" font-size=\"13\" fill=\"$INK\">$(esc(name))</text>")
    end
end

function head_svg(io, W, H, title, subtitle)
    print(io, "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 $W $H\" width=\"$W\" height=\"$H\" ",
          "font-family=\"Segoe UI, Roboto, Arial, sans-serif\">")
    print(io, "<rect width=\"$W\" height=\"$H\" fill=\"#ffffff\"/>")
    print(io, "<text x=\"24\" y=\"32\" font-size=\"19\" font-weight=\"700\" fill=\"$INK\">$(esc(title))</text>")
    isempty(subtitle) || print(io, "<text x=\"24\" y=\"52\" font-size=\"13\" fill=\"$MUTED\">$(esc(subtitle))</text>")
end

"""
Gráfico de linhas com eixos x e y em escala log.
`series` = vetor de (name, color, ys); `xs` são os valores de x (ticks nos próprios xs).
"""
function line_chart(file; title, subtitle = "", xlabel, ylabel, xs, series, yfmt = x -> string(x))
    W, H = 880, 500
    L, R, B = 86, 28, 66
    items = [(s.name, s.color) for s in series]
    lpos, lrows = legend_layout(items, W)
    T = 84 + 22 * lrows
    ys_all = filter(y -> y > 0, vcat((collect(s.ys) for s in series)...))
    ticks = decade_ticks(minimum(ys_all), maximum(ys_all))
    ylo, yhi = log10(first(ticks)), log10(last(ticks))
    xlo, xhi = log10(first(xs)), log10(last(xs))
    px(x) = L + (log10(x) - xlo) / (xhi - xlo) * (W - L - R)
    py(y) = T + (1 - (log10(y) - ylo) / (yhi - ylo)) * (H - T - B)
    io = IOBuffer()
    head_svg(io, W, H, title, subtitle)
    legend_svg(io, items, lpos, 78)
    for t in ticks
        print(io, "<line x1=\"$L\" x2=\"$(W - R)\" y1=\"$(py(t))\" y2=\"$(py(t))\" stroke=\"$GRID\"/>")
        print(io, "<text x=\"$(L - 8)\" y=\"$(py(t) + 4)\" font-size=\"12\" text-anchor=\"end\" fill=\"$MUTED\">$(esc(yfmt(t)))</text>")
    end
    for x in xs
        print(io, "<line x1=\"$(px(x))\" x2=\"$(px(x))\" y1=\"$T\" y2=\"$(H - B)\" stroke=\"$GRID\"/>")
        print(io, "<text x=\"$(px(x))\" y=\"$(H - B + 20)\" font-size=\"12\" text-anchor=\"middle\" fill=\"$MUTED\">$x</text>")
    end
    print(io, "<line x1=\"$L\" x2=\"$(W - R)\" y1=\"$(H - B)\" y2=\"$(H - B)\" stroke=\"$MUTED\"/>")
    print(io, "<line x1=\"$L\" x2=\"$L\" y1=\"$T\" y2=\"$(H - B)\" stroke=\"$MUTED\"/>")
    print(io, "<text x=\"$((L + W - R) / 2)\" y=\"$(H - 16)\" font-size=\"13\" text-anchor=\"middle\" fill=\"$INK\">$(esc(xlabel))</text>")
    print(io, "<text transform=\"translate(22 $((T + H - B) / 2)) rotate(-90)\" font-size=\"13\" text-anchor=\"middle\" fill=\"$INK\">$(esc(ylabel))</text>")
    for s in series
        pts = join(("$(px(x)),$(py(max(y, 10.0^ylo)))" for (x, y) in zip(xs, s.ys)), " ")
        print(io, "<polyline points=\"$pts\" fill=\"none\" stroke=\"$(s.color)\" stroke-width=\"2.6\" stroke-linejoin=\"round\"/>")
        for (x, y) in zip(xs, s.ys)
            print(io, "<circle cx=\"$(px(x))\" cy=\"$(py(max(y, 10.0^ylo)))\" r=\"4\" fill=\"$(s.color)\" stroke=\"#fff\" stroke-width=\"1.5\"/>")
        end
    end
    print(io, "</svg>")
    svg = String(take!(io))
    write(file, svg)
    return svg
end

"""
Gráfico de barras agrupadas. `groups` = rótulos do eixo x; `series` = (name, color, ys).
`ylog = true` usa escala log (valores ≤ 0 não são desenhados).
"""
function bar_chart(file; title, subtitle = "", xlabel, ylabel, groups, series, ylog = false,
                   yfmt = x -> string(x), vfmt = x -> string(round(x; digits = 1)))
    W, H = 880, 500
    L, R, B = 86, 28, 66
    items = [(s.name, s.color) for s in series]
    lpos, lrows = legend_layout(items, W)
    T = 84 + 22 * lrows
    vals = filter(y -> y > 0, vcat((collect(s.ys) for s in series)...))
    if ylog
        ticks = decade_ticks(minimum(vals), maximum(vals))
    else
        ticks = collect(range(0, nice_linear_max(maximum(vals)); length = 6))
    end
    lo, hi = ylog ? (log10(first(ticks)), log10(last(ticks))) : (0.0, last(ticks))
    py(y) = ylog ? T + (1 - (log10(max(y, 10.0^lo)) - lo) / (hi - lo)) * (H - T - B) :
                   T + (1 - y / hi) * (H - T - B)
    io = IOBuffer()
    head_svg(io, W, H, title, subtitle)
    legend_svg(io, items, lpos, 78)
    for t in ticks
        print(io, "<line x1=\"$L\" x2=\"$(W - R)\" y1=\"$(py(t))\" y2=\"$(py(t))\" stroke=\"$GRID\"/>")
        print(io, "<text x=\"$(L - 8)\" y=\"$(py(t) + 4)\" font-size=\"12\" text-anchor=\"end\" fill=\"$MUTED\">$(esc(yfmt(t)))</text>")
    end
    gw = (W - L - R) / length(groups)
    bw = min(46.0, gw * 0.8 / length(series))
    for (gi, g) in enumerate(groups)
        gx = L + (gi - 1) * gw + gw / 2
        print(io, "<text x=\"$gx\" y=\"$(H - B + 20)\" font-size=\"12\" text-anchor=\"middle\" fill=\"$MUTED\">$(esc(g))</text>")
        for (si, s) in enumerate(series)
            y = s.ys[gi]
            y > 0 || continue
            bx = gx + (si - (length(series) + 1) / 2) * bw - bw / 2
            print(io, "<rect x=\"$bx\" y=\"$(py(y))\" width=\"$(bw - 2)\" height=\"$(H - B - py(y))\" rx=\"3\" fill=\"$(s.color)\"/>")
            print(io, "<text x=\"$(bx + (bw - 2) / 2)\" y=\"$(py(y) - 5)\" font-size=\"10.5\" text-anchor=\"middle\" fill=\"$INK\">$(esc(vfmt(y)))</text>")
        end
    end
    print(io, "<line x1=\"$L\" x2=\"$(W - R)\" y1=\"$(H - B)\" y2=\"$(H - B)\" stroke=\"$MUTED\"/>")
    print(io, "<text x=\"$((L + W - R) / 2)\" y=\"$(H - 16)\" font-size=\"13\" text-anchor=\"middle\" fill=\"$INK\">$(esc(xlabel))</text>")
    print(io, "<text transform=\"translate(22 $((T + H - B) / 2)) rotate(-90)\" font-size=\"13\" text-anchor=\"middle\" fill=\"$INK\">$(esc(ylabel))</text>")
    print(io, "</svg>")
    svg = String(take!(io))
    write(file, svg)
    return svg
end

function html_table(header, rows)
    io = IOBuffer()
    print(io, "<table><thead><tr>", join(("<th>$(esc(h))</th>" for h in header)), "</tr></thead><tbody>")
    for r in rows
        print(io, "<tr>", join(("<td>$(esc(c))</td>" for c in r)), "</tr>")
    end
    print(io, "</tbody></table>")
    return String(take!(io))
end

function write_html_report(file; sections, meta)
    io = IOBuffer()
    print(io, """<!doctype html><html lang="pt-BR"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Benchmarks — T1 × T2</title><style>
body{font:15px/1.5 system-ui,"Segoe UI",Roboto,sans-serif;color:#1f2937;background:#f3f4f6;margin:0}
main{max-width:980px;margin:0 auto;padding:28px 20px 60px}
h1{font-size:28px;margin:0 0 4px} .meta{color:#6b7280;margin:0 0 24px;font-size:13px}
section{background:#fff;border:1px solid #e5e7eb;border-radius:12px;padding:20px 22px;margin:0 0 20px}
h2{font-size:19px;margin:0 0 6px} p{margin:6px 0 12px;color:#374151}
svg{max-width:100%;height:auto;display:block;margin:10px 0}
table{border-collapse:collapse;width:100%;font-size:13px;font-variant-numeric:tabular-nums;margin-top:8px}
th,td{padding:6px 10px;border-bottom:1px solid #e5e7eb;text-align:right} th:first-child,td:first-child{text-align:left}
th{background:#f9fafb;color:#374151}
</style></head><body><main>
<h1>Benchmarks — força bruta (T1) × preditor + corretor (T2)</h1>
<p class="meta">$(esc(meta))</p>""")
    for (title, text, svg, table) in sections
        print(io, "<section><h2>$(esc(title))</h2><p>$text</p>", svg, table, "</section>")
    end
    print(io, "</main></body></html>")
    write(file, String(take!(io)))
end
