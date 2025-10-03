using Plots, LinearAlgebra, Measures
# using Pkg; Pkg.add(["Plots","FFMPEG"]); using FFMPEG  # si hace falta

# --- Parámetros del robot y escena  ---
L1, L2 = 16.0, 12.0         # longitudes de los eslabones
xc, yc = -5.0, 10.0         # base del robot
R  = L1 + L2                # alcance máximo
r  = abs(L1 - L2)           # alcance mínimo
xtc, ytc = 10.0, 10.0       # centro del trébol
Rtrebol = 10.0              # radio del círculo del trébol

B = [xc, yc]                # vector base del robot

# Utilidades IK
function ik2r_both(B, P, L1, L2)
    # B: Coordenadas base robot
    # P: Coordenadas objetivo

    d = P .- B      # vector base → objetivo
    D = norm(d)     # distancia base → objetivo

    # Comprobar si es alcanzable
    if D < abs(L1 - L2) - 1e-9 || D > (L1 + L2) + 1e-9
        return nothing, nothing
    end

    
    a  = atan(d[2], d[1]) 
    c2 = clamp((D^2 - L1^2 - L2^2) / (2L1*L2), -1.0, 1.0)
    s2 = sqrt(1 - c2^2)

    θ2_up   = acos(c2)
    θ1_up   = a - atan(L2*s2,   L1 + L2*c2)
    θ2_down = θ2_up
    θ1_down = a - atan(-L2*s2,  L1 + L2*c2)

    E_up    = B .+ L1 .* [cos(θ1_up),   sin(θ1_up)]
    E_down  = B .+ L1 .* [cos(θ1_down), sin(θ1_down)]
    return (θ1_up, θ2_up, E_up), (θ1_down, θ2_down, E_down)
end

# Elegir solución continua (la más cercana en ángulos a la previa)
function pick_continuous(prev::Union{Nothing,Tuple{Float64,Float64}}, sol_up, sol_down)
    if prev === nothing
        return sol_up
    end
    (θ1p, θ2p) = prev
    (θ1u, θ2u, Eu) = sol_up
    (θ1d, θ2d, Ed) = sol_down
    du = hypot(θ1u-θ1p, θ2u-θ2p)
    dd = hypot(θ1d-θ1p, θ2d-θ2p)
    return du <= dd ? sol_up : sol_down
end

# Trayectorias
function line_path(A, B; steps=60)
    [(A .* (1-α) .+ B .* α) for α in range(0, 1, length=steps)]
end

# Waypoints pedidos
P_down  = [xc, yc - (L1+L2)]         # (-5, -18) estirado hacia abajo
P_first = [0.0, 10.0]                # primer punto (en el círculo del trébol)

# Círculo del trébol CCW empezando en (0,10): ángulo π → π+2π
Ncircle = 240
φs = range(π, π + 2π, length=Ncircle+1)[1:end-1]
circle_path = [[xtc + Rtrebol*cos(φ), ytc + Rtrebol*sin(φ)] for φ in φs]

# Trayectoria compuesta: abajo → (0,10) → círculo → (0,10) → abajo
traj = vcat(
    line_path(P_down,  P_first; steps=120),
    line_path(P_first, P_first; steps=20),   # pequeña pausa
    circle_path,
    line_path(P_first, P_first; steps=20),   # pequeña pausa
    line_path(P_first, P_down;  steps=120),
    line_path(P_down,  P_down;  steps=20)    # pequeña pausa
)

# Precalcula contornos (para redibujar en cada frame)
t = range(0, 2π, length=600)
x  = xc .+ R .* cos.(t);  y  = yc .+ R .* sin.(t)
x2 = xc .+ r .* cos.(t);  y2 = yc .+ r .* sin.(t)
xtrebol = xtc .+ Rtrebol .* cos.(t); ytrebol = ytc .+ Rtrebol .* sin.(t)

# Cuadrado opcional
qx, qy = 0.0, 0.0; s = 20.0
xq = [qx, qx+s, qx+s, qx,   qx];  yq = [qy, qy, qy+s, qy+s, qy]

# Animación
fps = 30
let last_angles = nothing  # para continuidad
    anim = @animate for k in 1:length(traj)
        
        P = traj[k]

        sol_up, sol_down = ik2r_both(B, P, L1, L2)
        if sol_up === nothing
            continue
        end

        chosen = pick_continuous(last_angles, sol_up, sol_down)
        θ1, θ2, E = chosen
        last_angles = (θ1, θ2)

        p = plot(x, y;
            label="Alcance máximo", color=:steelblue, linestyle=:solid, linewidth=2,
            xlabel="X", ylabel="Y", title="Espacio de trabajo LUKAS — animación 2R",
            aspect_ratio=1, grid=:on, legend=:outerright, framestyle=:box,
            size=(1200, 900)
        )
        hline!([0], label="", color=:gray40, linestyle=:dashdot, linewidth=1)
        vline!([0], label="", color=:gray40, linestyle=:dashdot, linewidth=1)

        # Sombreado del anillo
        xfill = vcat(x, reverse(x2)); yfill = vcat(y, reverse(y2))
        plot!(xfill, yfill; seriestype=:shape, label="", linecolor=:transparent,
            fillcolor=:lightskyblue, fillalpha=0.25)
        plot!(x2, y2; label="Alcance mínimo", color=:crimson, linestyle=:dash, linewidth=2)

        # Trébol y cuadrado
        plot!(xtrebol, ytrebol; label="Círculo trébol", color=:forestgreen, linestyle=:dot, linewidth=2)
        plot!(xq, yq; seriestype=:shape, fillalpha=0.20, fillcolor=:gold, linecolor=:gold, label="Cuadrado")
        plot!(xq, yq; linewidth=2, color=:black, linestyle=:solid, label="")

        # Base, brazos, efector
        scatter!([B[1]], [B[2]]; label="Base del robot", markersize=8, marker=:xcross, markercolor=:black)
        plot!([B[1], E[1]], [B[2], E[2]]; label="Eslabón 1 (16)", color=:dodgerblue, linewidth=4)
        plot!([E[1], P[1]], [E[2], P[2]]; label="Eslabón 2 (12)", color=:orange, linewidth=4)
        scatter!([P[1]], [P[2]]; label="Objetivo", marker=:diamond, markersize=8, markercolor=:red)

        xlims!(xc - R - 1, xc + R + 1)
        ylims!(yc - R - 1, yc + R + 1)
        plot!(legendfont=font(12), right_margin=24mm)
    end

    gif(anim, "2R_trayectoria_compuesta.gif", fps=fps)
end