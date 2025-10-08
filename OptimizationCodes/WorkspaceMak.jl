using GLMakie, LinearAlgebra

# --- Parámetros del robot y escena  ---
L1_init, L2_init = 20.0, 15.0   # longitudes iniciales de los eslabones
xc_init, yc_init = -8.81, 3.22   # posición inicial de la base del robot
xtc, ytc = 10.0, 10.0            # centro del trébol
Rtrebol = 10                     # radio del círculo del trébol
RtrebolScaled = Rtrebol * 1.3    # círculo del trébol escalado (para visualización)
Rmax = 15.0                       # radio máximo seguridad

# Observable para la posición de la base
base_pos = Observable([xc_init, yc_init])

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

# Función para generar la trayectoria basada en L1, L2 y posición de la base
function generate_trajectory(L1, L2, xc, yc)
    P_down  = [xc, yc - (L1 + L2)]    # punto más bajo: brazos completamente extendidos hacia abajo
    P_first = [0.0, 10.0]              # primer punto (en el círculo del trébol)
    
    # Círculo del trébol CCW empezando en (0,10): ángulo π → π+2π
    Ncircle = 240
    φs = range(π, π + 2π, length=Ncircle+1)[1:end-1]
    circle_path = [[xtc + Rtrebol*cos(φ), ytc + Rtrebol*sin(φ)] for φ in φs]
    
    # Trayectoria compuesta: abajo → (0,10) → círculo → (0,10) → abajo
    return vcat(
        line_path(P_down,  P_first; steps=120),
        line_path(P_first, P_first; steps=20),   # pequeña pausa
        circle_path,
        line_path(P_first, P_first; steps=20),   # pequeña pausa
        line_path(P_first, P_down;  steps=120),
        line_path(P_down,  P_down;  steps=20)    # pequeña pausa final en posición extendida
    )
end

# Trayectoria inicial
traj = Observable(generate_trajectory(L1_init, L2_init, xc_init, yc_init))

# Precalcula contornos
t = range(0, 2π, length=600)

# Otros círculos (no dependen de L1, L2)
xtrebol = xtc .+ Rtrebol .* cos.(t); ytrebol = ytc .+ Rtrebol .* sin.(t)
xtrebolscaled = xtc .+ RtrebolScaled .* cos.(t); ytrebolscaled = ytc .+ RtrebolScaled .* sin.(t)
xmax = xtc .+ Rmax .* cos.(t); ymax = ytc .+ Rmax .* sin.(t)

# Cuadrado opcional
qx, qy = 0.0, 0.0; s = 20.0
xq = [qx, qx+s, qx+s, qx, qx];  yq = [qy, qy, qy+s, qy+s, qy]

# Observables para círculos de alcance
circle_max_pts = Observable(Point2f[])
circle_min_pts = Observable(Point2f[])
ring_poly = Observable(Point2f[])

# Etiquetas dinámicas para eslabones
link1_label = Observable("Eslabón 1 ($(L1_init))")
link2_label = Observable("Eslabón 2 ($(L2_init))")

# Función para actualizar círculos cuando cambian L1, L2 o la posición de la base
function update_circles!(L1, L2, xc, yc)
    R = L1 + L2
    r = abs(L1 - L2)
    x_max = xc .+ R .* cos.(t)
    y_max = yc .+ R .* sin.(t)
    x_min = xc .+ r .* cos.(t)
    y_min = yc .+ r .* sin.(t)
    
    circle_max_pts[] = Point2f.(zip(x_max, y_max))
    circle_min_pts[] = Point2f.(zip(x_min, y_min))
    
    # Actualizar polígono de sombreado
    xfill = vcat(x_max, reverse(x_min))
    yfill = vcat(y_max, reverse(y_min))
    ring_poly[] = Point2f.(zip(xfill, yfill))
    
    # Actualizar etiquetas de eslabones
    link1_label[] = "Eslabón 1 ($(round(L1, digits=1)))"
    link2_label[] = "Eslabón 2 ($(round(L2, digits=1)))"
end

# Inicializar círculos
update_circles!(L1_init, L2_init, xc_init, yc_init)

# Crear figura y axis
fig = Figure(size=(1400, 900))
ax = Axis(fig[1, 1], 
    xlabel="X", 
    ylabel="Y", 
    title="Espacio de trabajo LUKAS — animación 2R",
    aspect=DataAspect()
)

# Elementos estáticos (círculos dinámicos basados en L1, L2)
lines!(ax, circle_max_pts, color=:steelblue, linewidth=2, label="Alcance máximo")
lines!(ax, circle_min_pts, color=:crimson, linewidth=2, linestyle=:dash, label="Alcance mínimo")
hlines!(ax, [0], color=:gray40, linewidth=1, linestyle=:dashdot)
vlines!(ax, [0], color=:gray40, linewidth=1, linestyle=:dashdot)

# Sombreado del anillo (polígono correcto)
poly!(ax, ring_poly, color=(:lightskyblue, 0.25), strokewidth=0)

# Trébol y otros círculos
lines!(ax, xtrebol, ytrebol, color=:forestgreen, linewidth=2, linestyle=:dot, label="Círculo trébol")
lines!(ax, xtrebolscaled, ytrebolscaled, color=:green, linewidth=1, linestyle=:dashdot, label="Círculo trébol (escalado)")
lines!(ax, xmax, ymax, color=:gray70, linewidth=1, linestyle=:dashdot, label="Límite máximo")

# Cuadrado
poly!(ax, Point2f.(zip(xq[1:end-1], yq[1:end-1])), color=(:gold, 0.2), strokecolor=:gold, strokewidth=2, label="Cuadrado")

# Base del robot (ahora con observable)
base_scatter = scatter!(ax, @lift([$(base_pos)[1]]), @lift([$(base_pos)[2]]), 
    marker=:xcross, markersize=20, color=:black, label="Base del robot")

# Elementos dinámicos (Observables)
link1_pts = Observable([Point2f(base_pos[][1], base_pos[][2]), Point2f(base_pos[][1], base_pos[][2])])
link2_pts = Observable([Point2f(base_pos[][1], base_pos[][2]), Point2f(base_pos[][1], base_pos[][2])])
target_pt = Observable(Point2f(traj[][1]...))

# Dibujar los brazos dinámicos
lines!(ax, link1_pts, color=:dodgerblue, linewidth=4, label=link1_label)
lines!(ax, link2_pts, color=:orange, linewidth=4, label=link2_label)
scatter!(ax, target_pt, marker=:diamond, markersize=15, color=:red, label="Objetivo")

# Leyenda fuera del axis (a la derecha) - usando axislegend para actualización automática
Legend(fig[1, 2], ax, framevisible=true)

# Control interactivo con Sliders
sg = SliderGrid(
    fig[2, 1:2],
    (label = "Punto de trayectoria", range = 1:length(traj[]), startvalue = 1),
    (label = "Longitud L1", range = 5.0:0.5:40.0, startvalue = L1_init),
    (label = "Longitud L2", range = 5.0:0.5:40.0, startvalue = L2_init),
    (label = "Posición Base X", range = -30.0:0.5:30.0, startvalue = xc_init),
    (label = "Posición Base Y", range = -30.0:0.5:30.0, startvalue = yc_init)
)

frame_idx = sg.sliders[1].value
L1_slider = sg.sliders[2].value
L2_slider = sg.sliders[3].value
xc_slider = sg.sliders[4].value
yc_slider = sg.sliders[5].value

# Variable para mantener continuidad de ángulos
last_angles = Ref{Union{Nothing,Tuple{Float64,Float64}}}(nothing)

# Función para actualizar la visualización
function update_robot!(k, L1, L2, B)
    current_traj = traj[]
    if k > length(current_traj)
        k = length(current_traj)
    end
    
    P = current_traj[k]
    
    sol_up, sol_down = ik2r_both(B, P, L1, L2)
    if sol_up === nothing
        # Si no es alcanzable, no actualizar
        return
    end
    
    chosen = pick_continuous(last_angles[], sol_up, sol_down)
    θ1, θ2, E = chosen
    last_angles[] = (θ1, θ2)
    
    # Actualizar observables
    link1_pts[] = [Point2f(B[1], B[2]), Point2f(E[1], E[2])]
    link2_pts[] = [Point2f(E[1], E[2]), Point2f(P[1], P[2])]
    target_pt[] = Point2f(P[1], P[2])
end

# Conectar slider de frame con la actualización
on(frame_idx) do k
    update_robot!(k, L1_slider[], L2_slider[], base_pos[])
end

# Conectar sliders de longitudes
on(L1_slider) do L1
    L2 = L2_slider[]
    xc = xc_slider[]
    yc = yc_slider[]
    
    update_circles!(L1, L2, xc, yc)
    
    # Regenerar trayectoria con nuevas longitudes
    traj[] = generate_trajectory(L1, L2, xc, yc)
    
    # Actualizar rango del slider de trayectoria
    set_close_to!(sg.sliders[1], 1)  # Resetear a posición inicial
    sg.sliders[1].range[] = 1:length(traj[])
    
    # Resetear ángulos para evitar saltos
    last_angles[] = nothing
    
    update_robot!(1, L1, L2, base_pos[])
    autolimits!(ax)
end

on(L2_slider) do L2
    L1 = L1_slider[]
    xc = xc_slider[]
    yc = yc_slider[]
    
    update_circles!(L1, L2, xc, yc)
    
    # Regenerar trayectoria con nuevas longitudes
    traj[] = generate_trajectory(L1, L2, xc, yc)
    
    # Actualizar rango del slider de trayectoria
    set_close_to!(sg.sliders[1], 1)  # Resetear a posición inicial
    sg.sliders[1].range[] = 1:length(traj[])
    
    # Resetear ángulos para evitar saltos
    last_angles[] = nothing
    
    update_robot!(1, L1, L2, base_pos[])
    autolimits!(ax)
end

# Conectar sliders de posición de la base
on(xc_slider) do xc
    yc = yc_slider[]
    base_pos[] = [xc, yc]
    
    L1 = L1_slider[]
    L2 = L2_slider[]
    
    update_circles!(L1, L2, xc, yc)
    
    # Regenerar trayectoria con nueva posición de base
    traj[] = generate_trajectory(L1, L2, xc, yc)
    
    # Actualizar rango del slider de trayectoria
    set_close_to!(sg.sliders[1], 1)  # Resetear a posición inicial
    sg.sliders[1].range[] = 1:length(traj[])
    
    # Resetear ángulos para evitar saltos
    last_angles[] = nothing
    
    update_robot!(1, L1, L2, base_pos[])
    autolimits!(ax)
end

on(yc_slider) do yc
    xc = xc_slider[]
    base_pos[] = [xc, yc]
    
    L1 = L1_slider[]
    L2 = L2_slider[]
    
    update_circles!(L1, L2, xc, yc)
    
    # Regenerar trayectoria con nueva posición de base
    traj[] = generate_trajectory(L1, L2, xc, yc)
    
    # Actualizar rango del slider de trayectoria
    set_close_to!(sg.sliders[1], 1)  # Resetear a posición inicial
    sg.sliders[1].range[] = 1:length(traj[])
    
    # Resetear ángulos para evitar saltos
    last_angles[] = nothing
    
    update_robot!(1, L1, L2, base_pos[])
    autolimits!(ax)
end

# Inicializar con el primer frame
update_robot!(1, L1_init, L2_init, base_pos[])

# Mostrar figura interactiva
display(fig)