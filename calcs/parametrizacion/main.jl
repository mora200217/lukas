using GLMakie
GLMakie.activate!()
using LaTeXStrings
using Observables: Observable, lift, on   # si aún no lo tenías

# ---- Parámetros iniciales ----
n  = Observable(3)        # hojas
a  = Observable(0.35)     # amplitud relativa (0 → círculo)
b  = Observable(1.0)      # radio base
k  = Observable(1.0)      # escala global
c  = Observable(0.0)      # desfase (rotación) [rad]
ω  = Observable(0.8)      # velocidad angular del cursor (rad/s)

θt = Observable(0.0)      # ángulo actual (cursor animado)
θg = range(0, 2π; length=2001)  # malla angular

# r(θ) según: r(θ)=k·b·(1 + a·cos(n(θ−c)))
rgrid = lift(n, a, b, k, c) do n_, a_, b_, k_, c_
    k_ .* b_ .* (1 .+ a_ .* cos.(n_ .* (θg .- c_)))
end

# r en el punto actual θt
rcur = lift(n, a, b, k, c, θt) do n_, a_, b_, k_, c_, θ_
    k_ * b_ * (1 + a_ * cos(n_ * (θ_ - c_)))
end

fig = Figure(resolution=(1200, 800))
ax  = PolarAxis(fig[1,1];
    title=L"Trébol: $r(\theta)=k\cdot b\cdot(1+a\cos(n(\theta-c)))$",
    thetalimits=(0, 2π),
    rminorgridvisible=true, thetagridvisible=true)

# curva completa
lines!(ax, θg, rgrid, linewidth=2)

# rastro hasta θt
θtrail = lift(θt) do θ_
    # submalla proporcional al avance, sin superar θ_
    range(0, θ_; length=max(2, Int(clamp(θ_ / (2π) * length(θg), 2, length(θg)))))
end

rtrail = lift(n, a, b, k, c, θtrail) do n_, a_, b_, k_, c_, θv
    k_ .* b_ .* (1 .+ a_ .* cos.(n_ .* (θv .- c_)))
end

lines!(ax, θtrail, rtrail, linewidth=3)
scatter!(ax, lift(θt) do θ_; [θ_] end,
             lift(rcur) do r_; [r_] end,
             markersize=12)

# --- Controles (sliders) ---
g = GridLayout(fig[2,1])  # si quieres: GridLayout(fig[2,1], tellwidth=false)

Label(g[1,1], "n (hojas)")
s1 = Slider(g[1,2], range=1:10, startvalue=n[])

Label(g[2,1], "a (A. Relativa: Ondulación)")
s2 = Slider(g[2,2], range=0:0.01:1, startvalue=a[])

Label(g[3,1], "b (radio base)")
s3 = Slider(g[3,2], range=0.2:0.01:3, startvalue=b[])

Label(g[4,1], "k (escala)")
s4 = Slider(g[4,2], range=0.5:0.01:1.2, startvalue=k[])

Label(g[5,1], "c (rotación)")
s5 = Slider(g[5,2], range=-π/4:0.001:π/4, startvalue=c[])

Label(g[6,1], "ω (rad/s)")
s6 = Slider(g[6,2], range=0.0:0.05:3.0, startvalue=ω[])

# Enlaces slider -> observables
on(s1.value) do v; n[] = Int(v); end
on(s2.value) do v; a[] = v; end
on(s3.value) do v; b[] = v; end
on(s4.value) do v; k[] = v; end
on(s5.value) do v; c[] = v; end
on(s6.value) do v; ω[] = v; end


# --- Mostrar y Animación tiempo real ---

# 1) Mostrar la figura y obtener la pantalla
screen = display(fig)

# 2) Bucle de animación (corre mientras la ventana esté abierta)
last_t = time()
@async while isopen(screen)
    sleep(1/60) # ~60 FPS
    now = time()
    dt  = now - last_t
    last_t = now
    θt[] = (θt[] + ω[] * dt) % (2π)
end

# 3) Si corres como script (no REPL), bloquea hasta que cierren la ventana
if !isinteractive()
    while isopen(screen)
        sleep(0.1)
    end
end

close(screen)
