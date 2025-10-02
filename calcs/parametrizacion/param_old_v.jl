using GLMakie
GLMakie.activate!()
using LaTeXStrings
using Observables: Observable, lift, on

# ---- Parámetros observables ----
n  = Observable(5)         # hojas
a0 = Observable(0.7)       # estilización forma base
d0 = Observable(0.007)     # estilización trebol
rm = Observable(10.0)      # radio máximo objetivo
s  = Observable(1.0)       # escala global (0.75..1.25)
c  = Observable(0.0)       # rotación
ω  = Observable(0.8)       # velocidad angular cursor (rad/s)

# animación / controls
playing = Observable(true)
looping  = Observable(true)
draw_speed = Observable(0.6)   # velocidad de "revelado" (progreso) en unidades 1/s

# malla angular y progreso del trazo
θg = range(0, 2π; length=2001)
progress = Observable(1.0)     # 0..1 indica cuánto de la curva se revela

# ---- Funciones auxiliares: a(n), d(n) ----
a = lift(a0, n) do a0_, n_; a0_ / n_; end
d = lift(d0, n) do d0_, n_; d0_ / n_; end

# Función base (sin normalizar). Queremos normalizar para garantizar rm como máximo.
fvals = (n_, a_, d_, c_) -> (1 .+ a_ .* cos.(n_ .* (θg .- c_ .- π))) .- d_ .* cos.(2n_ .* (θg .- c_))

# rgrid: valores normalizados para que el máximo sea s*rm
rgrid = lift(n, a, d, rm, s, c) do n_, a_, d_, rm_, s_, c_
    vals = fvals(n_, a_, d_, c_)
    fmax = maximum(vals)
    # protección: si fmax ≈ 0, evitar dividir por cero
    if iszero(fmax) || isnan(fmax)
        fill(s_ * rm_, length(vals))
    else
        s_ * rm_ .* (vals ./ fmax)
    end
end

# Coordenadas cartesianas de la malla (esto nos sirve para calcular límites automáticos)
xygrid = lift(rgrid) do rg
    x = rg .* cos.(θg)
    y = rg .* sin.(θg)
    (x, y)
end

# rcur: radio en el ángulo actual (cursor)
θt = Observable(0.0)
rcur = lift(n, a, d, rm, s, c, θt) do n_, a_, d_, rm_, s_, c_, θ_
    f = (1 + a_ * cos(n_ * (θ_ - c_ - π))) - d_ * cos(2n_ * (θ_ - c_))
    # normalizamos con fmax de la malla
    vals = fvals(n_, a_, d_, c_)
    fmax = maximum(vals)
    if iszero(fmax) || isnan(fmax)
        s_ * rm_
    else
        s_ * rm_ * f / fmax
    end
end

# rtrail (curva parcial hasta progress)
rpartial = lift(rgrid, progress) do rg, p
    m = max(2, Int(round(clamp(p, 0, 1) * length(rg))))
    rg[1:m]
end

θpartial = lift(progress) do p
    m = max(2, Int(round(clamp(p, 0, 1) * length(θg))))
    θg[1:m]
end

# ---- Figura y ejes ----
fig = Figure(resolution=(1200, 850))
ax = PolarAxis(fig[1,1];
    title=L"Trébol estilizado — normalizado a $r_m$ y escala s",
    thetalimits=(0,2π),
    rminorgridvisible=true, thetagridvisible=true)

# Curvas (se crearán con observables)
full_line = lines!(ax, θg, rgrid; linewidth=2, color = (:firebrick, 1.0))[end]
trail_line = lines!(ax, θpartial, rpartial; linewidth=3, color = (:firebrick, 1.0))[end]
cursor_scatter = scatter!(ax, lift(θt) do θ_; [θ_] end,
                              lift(rcur) do r_; [r_] end,
                              markersize=10, color=:black)[end]

# ---- Controles UI ----
g = GridLayout(fig[2,1]; tellwidth=false)

Label(g[1,1], "n (hojas)")
s1 = Slider(g[1,2], range=2:12, startvalue=n[])

Label(g[2,1], "a0 (forma base)")
s2 = Slider(g[2,2], range=0:0.01:2, startvalue=a0[])

Label(g[3,1], "d0 (forma trébol)")
s3 = Slider(g[3,2], range=0:0.000:0.1, startvalue=d0[])

Label(g[4,1], "rm (radio máx)")
s4 = Slider(g[4,2], range=1:0.5:40, startvalue=rm[])

Label(g[5,1], "s (escala global)")
s5 = Slider(g[5,2], range=0.5:0.01:1.5, startvalue=s[])

Label(g[6,1], "c (rotación)")
s6 = Slider(g[6,2], range=-π:0.01:π, startvalue=c[])

Label(g[7,1], "ω (vel. cursor rad/s)")
s7 = Slider(g[7,2], range=0.0:0.05:3.0, startvalue=ω[])

Label(g[8,1], "Velocidad de dibujo")
s8 = Slider(g[8,2], range=0.05:0.05:3.0, startvalue=draw_speed[])

# Play / Loop buttons
using MakieLayout
btn_play = Button(g[9,1], label = "Pause")
btn_loop = Toggle(g[9,2], label = "Loop", active = looping[])

# enlaces sliders -> observables
on(s1.value) do v; n[] = Int(round(v)); end
on(s2.value) do v; a0[] = v; end
on(s3.value) do v; d0[] = v; end
on(s4.value) do v; rm[] = v; end
on(s5.value) do v; s[] = v; end
on(s6.value) do v; c[] = v; end
on(s7.value) do v; ω[] = v; end
on(s8.value) do v; draw_speed[] = v; end
on(btn_loop.toggled) do v; looping[] = v; end
on(btn_play.clicks) do _
    # alternar playing y actualizar etiqueta
    playing[] = !playing[]
    btn_play.label[] = playing[] ? "Pause" : "Play"
end

# ---- Ajuste automático del viewport (margen dinámico) ----
# Calculamos bounds cada vez que cambian rgrid -> actualizamos límites del eje polar
lift(xygrid) do xy
    x, y = xy
    xmin, xmax = minimum(x), maximum(x)
    ymin, ymax = minimum(y), maximum(y)
    # margen (6%)
    mx = max(abs(xmin), abs(xmax))
    my = max(abs(ymin), abs(ymax))
    R = max(mx, my)
    if R <= 0
        R = 1.0
    end
    # Intentamos forzar que el PolarAxis muestre hasta R*1.08
    try
        # many Makie backends accept: limits!(ax, (xmin,xmax),(ymin,ymax))
        limits!(ax, (-R*1.08, R*1.08), (-R*1.08, R*1.08))
    catch e
        # fallback: intenta con rlimits si existe
        try
            ax.rlimits[] = (0, R*1.08)
        catch
            # si no podemos, no hacer nada (sigue con valores por defecto)
        end
    end
end

# ---- Animación principal: cursor + "revelado" del trazo ----
screen = display(fig)
last_t = time()

@async begin
    while isopen(screen)
        sleep(1/120)  # doble FPS para suavizar (120Hz ciclo de actualización)
        now = time()
        dt = now - last_t
        last_t = now

        # actualizar el ángulo del cursor (si está en play)
        if playing[]
            θt[] = (θt[] + ω[] * dt) % (2π)
        end

        # actualización del progreso de dibujo
        if playing[]
            # avanzar progresivamente hasta 1.0
            p = progress[]
            pnew = p + dt * draw_speed[]
            if pnew >= 1.0
                if looping[]
                    pnew = 0.0  # reinicia para hacer loop
                else
                    pnew = 1.0
                end
            end
            progress[] = pnew
        end
    end
end

# Si quieres que empiece siempre desde 0 (revelado), descomenta:
# progress[] = 0.0

# bloqueo sólo si ejecutas como script
if !isinteractive()
    while isopen(screen)
        sleep(0.1)
    end
end
