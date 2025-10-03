using GLMakie
GLMakie.activate!()
using Observables
using Printf
using CSV
using DataFrames
using Dates

# Crear carpeta Data si no existe
if !isdir("Data")
    mkdir("Data")
end

# ---------- Configuración de tema visual ----------
set_theme!(theme_light())
update_theme!(
    fontsize = 14,
    Label = (textcolor = :black,),
    Slider = (color_active = RGBf(0.2, 0.6, 0.8), color_inactive = RGBf(0.7, 0.7, 0.7)),
    Button = (buttoncolor = RGBf(0.2, 0.6, 0.8), labelcolor = :white),
    Toggle = (buttoncolor = RGBf(0.2, 0.6, 0.8), labelcolor = :white)
)

# ---------- Parámetros / Observables ----------
n  = Observable(5)         # número de hojas
a0 = Observable(0.7)       # estilización forma base
d0 = Observable(0.007)     # estilización trébol
rm = Observable(10.0)      # radio máximo objetivo
s  = Observable(1.0)       # escala global (0.75..1.25)
c  = Observable(0.0)       # rotación (rad)

# animación
progress   = Observable(0.0)   # 0..1 progreso del trazo
θt         = Observable(0.0)   # ángulo del cursor - INICIALIZADO EN 0

# Puntos para trayectoria exportada
trajectory_points = Observable(2001)  # número de puntos para exportación

# malla angular para visualización
θg = range(0, 2π; length=2001)

# ---------- Funciones auxiliares: a(n), d(n) ----------
a = lift(a0, n) do a0_val, n_val
    a0_val / n_val
end

d = lift(d0, n) do d0_val, n_val
    d0_val / n_val
end

# Función base del trébol
function trebol_function(θ, n_val, a_val, d_val, c_val)
    return 1 .+ a_val .* cos.(n_val .* (θ .- c_val .- pi)) .- d_val .* cos.(2n_val .* (θ .- c_val .- pi))
end 

# ---------- r(θ) normalizado para que max = s*rm ----------
rgrid = lift(n, a, d, rm, s, c) do n_val, a_val, d_val, rm_val, s_val, c_val
    f_vals = trebol_function(θg, n_val, a_val, d_val, c_val)
    f_max = maximum(f_vals)
    
    if f_max ≈ 0.0
        fill(s_val * rm_val, length(θg))
    else
        s_val * rm_val .* (f_vals ./ f_max)
    end
end

# Radio en el ángulo actual (cursor)
rcur = lift(n, a, d, rm, s, c, θt) do n_val, a_val, d_val, rm_val, s_val, c_val, θt_val
    f_val = trebol_function([θt_val], n_val, a_val, d_val, c_val)[1]
    f_vals = trebol_function(θg, n_val, a_val, d_val, c_val)
    f_max = maximum(f_vals)
    
    if f_max ≈ 0.0
        s_val * rm_val
    else
        s_val * rm_val * f_val / f_max
    end
end

# Convertir a coordenadas cartesianas como vectores de Point2
xygrid = lift(rgrid) do rg
    points = [Point2f(r * cos(θ), r * sin(θ)) for (r, θ) in zip(rg, θg)]
    points
end

# Trazo parcial (0..progress)
θpartial = lift(progress) do p_val
    m = max(2, Int(round(clamp(p_val, 0, 1) * length(θg))))
    θg[1:m]
end

rpartial = lift(n, a, d, rm, s, c, θpartial) do n_val, a_val, d_val, rm_val, s_val, c_val, θv
    f_vals = trebol_function(θv, n_val, a_val, d_val, c_val)
    f_max = maximum(trebol_function(θg, n_val, a_val, d_val, c_val))
    
    if f_max ≈ 0.0
        fill(s_val * rm_val, length(θv))
    else
        s_val * rm_val .* (f_vals ./ f_max)
    end
end

xypartial = lift(rpartial, θpartial) do rg, θv
    points = [Point2f(r * cos(θ), r * sin(θ)) for (r, θ) in zip(rg, θv)]
    points
end

# Punto actual como vector de Point2 - CORREGIDO: usa θt que inicia en 0
xycurrent = lift(θt, rcur) do θt_val, rcur_val
    [Point2f(rcur_val * cos(θt_val), rcur_val * sin(θt_val))]
end

# ---------- Expresión matemática de r(θ) ----------
# Observable para la expresión matemática formateada
math_expression = lift(n, a, d, rm, s, c) do n_val, a_val, d_val, rm_val, s_val, c_val
    # Calcular el valor máximo para normalización
    f_vals = trebol_function(θg, n_val, a_val, d_val, c_val)
    f_max = maximum(f_vals)
    
    if f_max ≈ 0.0
        return "r(θ) = $(round(s_val * rm_val; digits=3))"
    else
        return @sprintf "r(θ) = %.2f × [1 + %.3f × cos(%d(θ - %.3f - π)) - %.4f × cos(%d(θ - %.3f - π))] / %.3f" s_val*rm_val a_val n_val c_val d_val 2*n_val c_val f_max
    end
end

# ---------- Funciones para exportar ----------
function export_parameters()
    timestamp = Dates.format(now(), "yyyy-mm-dd_HH-MM-SS")
    filename = "Data/param_$(timestamp).csv"
    
    # Calcular f_max para exportación
    f_vals = trebol_function(θg, n[], a[], d[], c[])
    f_max = maximum(f_vals)
    
    params_dict = Dict(
        "n" => n[],
        "a0" => a0[],
        "d0" => d0[],
        "rm" => rm[],
        "s" => s[],
        "c" => c[],
        "f_max" => f_max
    )
    
    df = DataFrame(params_dict)
    CSV.write(filename, df)
    
    println("Parámetros exportados a: $filename")
    println("f_max exportado: $f_max")
    return filename
end

function export_trajectory()
    timestamp = Dates.format(now(), "yyyy-mm-dd_HH-MM-SS")
    filename = "Data/trajectory_$(timestamp).csv"
    
    # Crear malla angular con el número de puntos especificado
    num_points = trajectory_points[]
    θ_export = range(0, 2π; length=num_points)
    
    # Calcular la trayectoria completa
    f_vals = trebol_function(θ_export, n[], a[], d[], c[])
    f_max = maximum(f_vals)
    
    if f_max ≈ 0.0
        r_export = fill(s[] * rm[], length(θ_export))
    else
        r_export = s[] * rm[] .* (f_vals ./ f_max)
    end
    
    # Convertir a coordenadas cartesianas
    x_export = r_export .* cos.(θ_export.-pi)
    y_export = r_export .* sin.(θ_export.-pi)
    
    # Crear DataFrame con la trayectoria
    trajectory_data = DataFrame(
        theta = θ_export,
        x = x_export,
        y = y_export,
        r = r_export
    )
    
    CSV.write(filename, trajectory_data)
    
    println("Trayectoria exportada a: $filename")
    println("Número de puntos: $num_points")
    return filename
end

function export_all()
    param_file = export_parameters()
    traj_file = export_trajectory()
    println("Exportación completa:")
    println("  - Parámetros: $param_file")
    println("  - Trayectoria: $traj_file")
    return (param_file, traj_file)
end

# ---------- Crear Figura con Layout Mejorado ----------
fig = Figure(
    size = (1600, 1000),
    backgroundcolor = RGBf(0.98, 0.98, 0.98)
)

# Crear un GridLayout principal
main_layout = GridLayout(fig[1, 1])

# Panel izquierdo para la gráfica (70% del ancho)
graph_panel = main_layout[1, 1] = GridLayout()
# Panel derecho para controles (30% del ancho)
control_panel = main_layout[1, 2] = GridLayout()

# ---------- Gráfica Principal ----------
ax = Axis(
    graph_panel[1, 1], 
    xlabel = "Coordenada X",
    ylabel = "Coordenada Y", 
    title = "TRÉBOL ESTILIZADO",
    titlesize = 20,
    xlabelsize = 16,
    ylabelsize = 16,
    aspect = DataAspect(),
    backgroundcolor = :white
)

# Mejorar la estética de la gráfica
lines!(ax, xygrid; 
       linewidth = 3, 
       color = RGBf(0.1, 0.3, 0.7))

trail_line = lines!(ax, xypartial; 
                    linewidth = 4, 
                    color = RGBf(0.8, 0.2, 0.2))

current_point = scatter!(ax, xycurrent; 
                         markersize = 25, 
                         color = RGBf(0.2, 0.7, 0.2),
                         strokecolor = :black,
                         strokewidth = 2)

# Ajustar límites automáticamente
on(xygrid) do points
    xs = [p[1] for p in points]
    ys = [p[2] for p in points]
    padding = 0.15 * maximum(abs.(vcat(xs, ys)))
    xlims!(ax, minimum(xs) - padding, maximum(xs) + padding)
    ylims!(ax, minimum(ys) - padding, maximum(ys) + padding)
end

# ---------- Ecuación del Trébol ----------
# Título de la ecuación
equation_title = Label(graph_panel[2, 1], "ECUACIÓN MATEMÁTICA ACTUAL:",
    fontsize = 16,
    color = :black,
    font = :bold,
    tellwidth = false)

# Expresión matemática actualizada
equation_label = Label(graph_panel[3, 1], math_expression,
    fontsize = 14,
    color = RGBf(0.1, 0.1, 0.5),
    font = :italic,
    tellwidth = false)

# ---------- Panel de Control Mejorado ----------

# Título del panel de control
Label(control_panel[1, 1:2], "PANEL DE CONTROL", 
      fontsize = 18, 
      font = :bold,
      color = RGBf(0.1, 0.1, 0.4))

# Grupo 1: Parámetros de Forma
Label(control_panel[2, 1:2], "PARÁMETROS DE FORMA", 
      fontsize = 16, 
      color = RGBf(0.2, 0.4, 0.6))

s_n    = Slider(control_panel[3, 2], range = 2:1:12, startvalue = n[])
n_label = Label(control_panel[3, 1], "Número de hojas (n): $(Int(n[]))", 
      tellwidth = false)

s_a0   = Slider(control_panel[4, 2], range = 0:0.01:2, startvalue = a0[])
a0_label = Label(control_panel[4, 1], "Forma base (a₀): $(round(a0[]; digits=3))", 
      tellwidth = false)

s_d0   = Slider(control_panel[5, 2], range = 0.0:0.001:0.5, startvalue = d0[])
d0_label = Label(control_panel[5, 1], "Profundidad (d₀): $(round(d0[]; digits=4))", 
      tellwidth = false)

# Grupo 2: Escala y Tamaño
Label(control_panel[6, 1:2], "ESCALA Y TAMAÑO", 
      fontsize = 16, 
      color = RGBf(0.2, 0.4, 0.6))

s_rm   = Slider(control_panel[7, 2], range = 1:0.5:80, startvalue = rm[])
rm_label = Label(control_panel[7, 1], "Radio máximo (rm): $(round(rm[]; digits=1))", 
      tellwidth = false)

s_s    = Slider(control_panel[8, 2], range = 0.5:0.01:1.5, startvalue = s[])
s_label = Label(control_panel[8, 1], "Factor escala (s): $(round(s[]; digits=2))", 
      tellwidth = false)

# Grupo 3: Rotación
Label(control_panel[9, 1:2], "ROTACIÓN", 
      fontsize = 16, 
      color = RGBf(0.2, 0.4, 0.6))

s_c    = Slider(control_panel[10, 2], range = -π:0.01:π, startvalue = c[])
c_label = Label(control_panel[10, 1], "Rotación (c): $(round(c[]; digits=3)) rad", 
      tellwidth = false)

# Grupo 4: Exportación
Label(control_panel[11, 1:2], "EXPORTACIÓN", 
      fontsize = 16, 
      color = RGBf(0.2, 0.4, 0.6))

# Slider para número de puntos de trayectoria
points_label = Label(control_panel[12, 1], "Puntos trayectoria:", tellwidth = false)
s_points = Slider(control_panel[12, 2], range = 100:100:10000, startvalue = trajectory_points[])
points_value_label = Label(control_panel[12, 3], "$(trajectory_points[])", tellwidth = false)

# Botones de exportación
export_buttons_layout = GridLayout(control_panel[13, 1:2])
btn_export_params = Button(export_buttons_layout[1, 1], label = "Exportar Parámetros")
btn_export_traj = Button(export_buttons_layout[1, 2], label = "Exportar Trayectoria")
btn_export_all = Button(export_buttons_layout[2, 1:2], label = "Exportar Todo")

# ---------- Actualizaciones en Tiempo Real ----------
function update_labels()
    n_label.text[] = "Número de hojas (n): $(Int(n[]))"
    a0_label.text[] = "Forma base (a₀): $(round(a0[]; digits=3))"
    d0_label.text[] = "Profundidad (d₀): $(round(d0[]; digits=4))"
    rm_label.text[] = "Radio máximo (rm): $(round(rm[]; digits=1))"
    s_label.text[] = "Factor escala (s): $(round(s[]; digits=2))"
    c_label.text[] = "Rotación (c): $(round(c[]; digits=3)) rad"
    points_value_label.text[] = "$(trajectory_points[])"
end

# Conectar actualizaciones
on(n) do _; update_labels() end
on(a0) do _; update_labels() end
on(d0) do _; update_labels() end
on(rm) do _; update_labels() end
on(s) do _; update_labels() end
on(c) do _; update_labels() end
on(trajectory_points) do _; update_labels() end

# ---------- Enlaces widgets -> observables ----------
on(s_n.value) do v
    n[] = Int(round(v))
end
on(s_a0.value) do v
    a0[] = v
end
on(s_d0.value) do v
    d0[] = v
end
on(s_rm.value) do v
    rm[] = v
end
on(s_s.value) do v
    s[] = v
end
on(s_c.value) do v
    c[] = v
end
on(s_points.value) do v
    trajectory_points[] = Int(round(v))
end

# Botones de exportación
on(btn_export_params.clicks) do _
    export_parameters()
end

on(btn_export_traj.clicks) do _
    export_trajectory()
end

on(btn_export_all.clicks) do _
    export_all()
end

# Ajustar espaciado
colgap!(main_layout, 30)
rowgap!(control_panel, 12)
colgap!(control_panel, 10)
rowgap!(export_buttons_layout, 5)

# ---------- Animación ----------
screen = display(fig)
last_t = time()

# Velocidades fijas (sin controles)
angular_speed = 1.0  # rad/s
draw_speed = 0.8     # unidades/s

@async while isopen(screen)
    sleep(1/120)
    now = time()
    dt = now - last_t
    last_t = now

    # Actualizar ángulo del cursor
    θt[] = (θt[] + angular_speed * dt) % (2π)
    
    # Avanzar progreso del trazo
    p = progress[]
    pnew = p + dt * draw_speed
    if pnew >= 1.0
        pnew = 0.0  # Reiniciar automáticamente
    end
    progress[] = pnew
end

# Bloquear si no es interactivo
if !isinteractive()
    while isopen(screen)
        sleep(0.1)
    end
end