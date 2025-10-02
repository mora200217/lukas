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
# Aplicar tema light primero y luego personalizaciones específicas
set_theme!(theme_light())

# Personalizaciones individuales del tema
update_theme!(
    fontsize = 14,
    Label = (textcolor = :black,),
    Slider = (color_active = RGBf(0.2, 0.6, 0.8), color_inactive = RGBf(0.7, 0.7, 0.7)),
    Button = (buttoncolor = RGBf(0.2, 0.6, 0.8), textcolor = :white),
    Toggle = (buttoncolor = RGBf(0.2, 0.6, 0.8),)
)

# ---------- Parámetros / Observables ----------
n  = Observable(5)         # número de hojas
a0 = Observable(0.7)       # estilización forma base
d0 = Observable(0.007)     # estilización trébol
rm = Observable(10.0)      # radio máximo objetivo
s  = Observable(1.0)       # escala global (0.75..1.25)
c  = Observable(0.0)       # rotación (rad)
ω  = Observable(0.8)       # velocidad angular del cursor (rad/s)

# animación / controles
playing    = Observable(true)
looping    = Observable(true)
draw_speed = Observable(0.6)   # velocidad de "revelado" (unidades 1/s)
progress   = Observable(0.0)   # 0..1 progreso del trazo
θt         = Observable(0.0)   # ángulo del cursor

# malla angular
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
    return 1 .+ a_val .* cos.(n_val .* (θ .- c_val .- π)) .- d_val .* cos.(2n_val .* (θ .- c_val))
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

# Punto actual como vector de Point2
xycurrent = lift(θt, rcur) do θt_val, rcur_val
    [Point2f(rcur_val * cos(θt_val), rcur_val * sin(θt_val))]
end

# ---------- Función para exportar parámetros ----------
function export_parameters()
    timestamp = Dates.format(now(), "yyyy-mm-dd_HH-MM-SS")
    filename = "Data/param_$(timestamp).csv"
    
    # Crear datos para exportar
    params_dict = Dict(
        "n" => n[],
        "a0" => a0[],
        "d0" => d0[],
        "rm" => rm[],
        "s" => s[],
        "c" => c[],
        "omega" => ω[],
        "timestamp" => timestamp
    )
    
    # Crear DataFrame y exportar
    df = DataFrame(params_dict)
    CSV.write(filename, df)
    
    println("Parámetros exportados a: $filename")
    return filename
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
    title = "TRÉBOL ESTILIZADO - RADIO MÁXIMO FIJO = rm × s",
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

# Grupo 3: Rotación y Movimiento
Label(control_panel[9, 1:2], "ROTACIÓN Y MOVIMIENTO", 
      fontsize = 16, 
      color = RGBf(0.2, 0.4, 0.6))

s_c    = Slider(control_panel[10, 2], range = -π:0.01:π, startvalue = c[])
c_label = Label(control_panel[10, 1], "Rotación (c): $(round(c[]; digits=3)) rad", 
      tellwidth = false)

s_ω    = Slider(control_panel[11, 2], range = 0.0:0.05:3.0, startvalue = ω[])
ω_label = Label(control_panel[11, 1], "Velocidad (ω): $(round(ω[]; digits=2)) rad/s", 
      tellwidth = false)

# Grupo 4: Animación
Label(control_panel[12, 1:2], "CONTROL DE ANIMACIÓN", 
      fontsize = 16, 
      color = RGBf(0.2, 0.4, 0.6))

s_draw = Slider(control_panel[13, 2], range = 0.01:0.01:3.0, startvalue = draw_speed[])
draw_label = Label(control_panel[13, 1], "Velocidad dibujo: $(round(draw_speed[]; digits=2))", 
      tellwidth = false)

# Botones en una disposición horizontal
button_layout = GridLayout(control_panel[14, 1:2])
btn_play = Button(button_layout[1, 1], label = playing[] ? "Pausar" : "Reproducir")
btn_reset = Button(button_layout[1, 2], label = "Reiniciar")
t_loop   = Toggle(button_layout[1, 3], active = looping[])
loop_label = Label(button_layout[1, 4], looping[] ? "Bucle: ON" : "Bucle: OFF")

# Botón de exportación
export_layout = GridLayout(control_panel[15, 1:2])
btn_export = Button(export_layout[1, 1:2], label = "Exportar Parámetros")

# ---------- Actualizaciones en Tiempo Real de las Etiquetas ----------

# Función para actualizar etiquetas
function update_labels()
    n_label.text[] = "Número de hojas (n): $(Int(n[]))"
    a0_label.text[] = "Forma base (a₀): $(round(a0[]; digits=3))"
    d0_label.text[] = "Profundidad (d₀): $(round(d0[]; digits=4))"
    rm_label.text[] = "Radio máximo (rm): $(round(rm[]; digits=1))"
    s_label.text[] = "Factor escala (s): $(round(s[]; digits=2))"
    c_label.text[] = "Rotación (c): $(round(c[]; digits=3)) rad"
    ω_label.text[] = "Velocidad (ω): $(round(ω[]; digits=2)) rad/s"
    draw_label.text[] = "Velocidad dibujo: $(round(draw_speed[]; digits=2))"
    btn_play.label[] = playing[] ? "Pausar" : "Reproducir"
    loop_label.text[] = looping[] ? "Bucle: ON" : "Bucle: OFF"
end

# Conectar actualizaciones
on(n) do _; update_labels() end
on(a0) do _; update_labels() end
on(d0) do _; update_labels() end
on(rm) do _; update_labels() end
on(s) do _; update_labels() end
on(c) do _; update_labels() end
on(ω) do _; update_labels() end
on(draw_speed) do _; update_labels() end
on(playing) do _; update_labels() end
on(looping) do _; update_labels() end

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
on(s_ω.value) do v
    ω[] = v
end
on(s_draw.value) do v
    draw_speed[] = v
end

on(btn_play.clicks) do _
    playing[] = !playing[]
end

on(btn_reset.clicks) do _
    progress[] = 0.0
end

on(t_loop.active) do val
    looping[] = val
end

# Exportar parámetros cuando se presiona el botón
on(btn_export.clicks) do _
    export_parameters()
end

# Ajustar espaciado y márgenes
colgap!(main_layout, 30)
rowgap!(control_panel, 15)
colgap!(control_panel, 10)
rowgap!(button_layout, 5)
rowgap!(export_layout, 10)

# ---------- Animación ----------
screen = display(fig)
last_t = time()

@async while isopen(screen)
    sleep(1/120)
    now = time()
    dt = now - last_t
    last_t = now

    if playing[]
        # Actualizar ángulo del cursor
        θt[] = (θt[] + ω[] * dt) % (2π)
        
        # Avanzar progreso del trazo
        p = progress[]
        pnew = p + dt * draw_speed[]
        if pnew >= 1.0
            if looping[]
                pnew = 0.0
            else
                pnew = 1.0
            end
        end
        progress[] = pnew
    end
end

# Bloquear si no es interactivo
if !isinteractive()
    while isopen(screen)
        sleep(0.1)
    end
end