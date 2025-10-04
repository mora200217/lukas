using GLMakie
GLMakie.activate!()
using Observables
using Printf
using LinearAlgebra
using DataFrames
using Dates

# ---------- Configuración de tema visual ----------
set_theme!(theme_light())
update_theme!(
    fontsize = 14,
    Label = (textcolor = :black,),
    Slider = (color_active = RGBf(0.2, 0.6, 0.8), color_inactive = RGBf(0.7, 0.7, 0.7)),
    Button = (buttoncolor = RGBf(0.2, 0.6, 0.8), labelcolor = :white),
    Toggle = (buttoncolor = RGBf(0.2, 0.6, 0.8), labelcolor = :white)
)

# ---------- Estructura del Robot ----------
struct Robot2R
    L1::Float64      # Longitud eslabón 1
    L2::Float64      # Longitud eslabón 2
    base_offset_x::Float64  # Desplazamiento de la base en X
end

# ---------- Funciones de Cinemática Inversa ----------
function aplicar_offset_base(x, y, offset)
    return x .+ offset, y
end

function cinematica_inversa_punto(robot::Robot2R, x, y)
    d = sqrt(x^2 + y^2)
    d_max = robot.L1 + robot.L2
    d_min = abs(robot.L1 - robot.L2)
    
    if d > d_max || d < d_min
        return nothing, nothing, false
    end
    
    c2 = (x^2 + y^2 - robot.L1^2 - robot.L2^2) / (2 * robot.L1 * robot.L2)
    c2 = clamp(c2, -1.0, 1.0)
    
    s2_pos = sqrt(1 - c2^2)
    s2_neg = -sqrt(1 - c2^2)
    
    theta2_pos = atan(s2_pos, c2)
    theta2_neg = atan(s2_neg, c2)
    
    k1 = robot.L1 + robot.L2 * c2
    k2_pos = robot.L2 * s2_pos
    k2_neg = robot.L2 * s2_neg
    
    theta1_pos = atan(y, x) - atan(k2_pos, k1)
    theta1_neg = atan(y, x) - atan(k2_neg, k1)
    
    return [theta1_pos, theta2_pos], [theta1_neg, theta2_neg], true
end

function calcular_cinematica_completa(robot::Robot2R, puntos_x, puntos_y; configuracion="arriba")
    n_puntos = length(puntos_x)
    x_robot, y_robot = aplicar_offset_base(puntos_x, puntos_y, robot.base_offset_x)
    
    theta1_tray = zeros(n_puntos)
    theta2_tray = zeros(n_puntos)
    alcanzable = fill(false, n_puntos)
    
    for i in 1:n_puntos
        x, y = x_robot[i], y_robot[i]
        sol_arriba, sol_abajo, es_alcanzable = cinematica_inversa_punto(robot, x, y)
        
        alcanzable[i] = es_alcanzable
        
        if es_alcanzable
            if configuracion == "arriba"
                theta1_tray[i], theta2_tray[i] = sol_arriba
            else
                theta1_tray[i], theta2_tray[i] = sol_abajo
            end
        else
            theta1_tray[i], theta2_tray[i] = NaN, NaN
        end
    end
    
    return theta1_tray, theta2_tray, alcanzable
end

# ---------- Parámetros del Trébol (Observables) ----------
n  = Observable(5)         # número de hojas
a0 = Observable(0.7)       # estilización forma base
d0 = Observable(0.007)     # estilización trébol
rm = Observable(10.0)      # radio máximo objetivo
s  = Observable(1.0)       # escala global (0.75..1.25)
c  = Observable(0.0)       # rotación (rad)

# animación
progress   = Observable(0.0)   # 0..1 progreso del trazo
θt         = Observable(0.0)   # ángulo del cursor

# Parámetros del robot
robot = Robot2R(16.0, 12.0, 15.0)  # L1=16cm, L2=12cm, offset=15cm

# ---------- Funciones del Trébol ----------
θg = range(0, 2π; length=2001)

a = lift(a0, n) do a0_val, n_val
    a0_val / n_val
end

d = lift(d0, n) do d0_val, n_val
    d0_val / n_val
end

function trebol_function(θ, n_val, a_val, d_val, c_val)
    return 1 .+ a_val .* cos.(n_val .* (θ .- c_val .- pi)) .- d_val .* cos.(2n_val .* (θ .- c_val .- pi))
end 

rgrid = lift(n, a, d, rm, s, c) do n_val, a_val, d_val, rm_val, s_val, c_val
    f_vals = trebol_function(θg, n_val, a_val, d_val, c_val)
    f_max = maximum(f_vals)
    
    if f_max ≈ 0.0
        fill(s_val * rm_val, length(θg))
    else
        s_val * rm_val .* (f_vals ./ f_max)
    end
end

xygrid = lift(rgrid) do rg
    points = [Point2f(r * cos(θ), r * sin(θ)) for (r, θ) in zip(rg, θg)]
    points
end

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

xycurrent = lift(θt, rcur) do θt_val, rcur_val
    [Point2f(rcur_val * cos(θt_val), rcur_val * sin(θt_val))]
end

# ---------- Cálculo en vivo de Cinemática Inversa ----------
# Observables para datos de cinemática
n_points = length(θg)
theta1_data = Observable(zeros(n_points))
theta2_data = Observable(zeros(n_points))
alcanzable_data = Observable(fill(false, n_points))
indices = Observable(collect(1:n_points))

configuracion = Observable("arriba")
usar_grados = Observable(false)

# Estadísticas en vivo
n_total = Observable(0)
n_alcanzables = Observable(0)
porcentaje_alcanzable = Observable(0.0)
theta1_min = Observable(0.0)
theta1_max = Observable(0.0)
theta2_min = Observable(0.0)
theta2_max = Observable(0.0)

function actualizar_cinematica_en_vivo()
    # Obtener puntos actuales del trébol
    puntos_actuales = xygrid[]
    puntos_x = [p[1] for p in puntos_actuales]
    puntos_y = [p[2] for p in puntos_actuales]
    
    # Calcular cinemática inversa
    theta1, theta2, alcanzable = calcular_cinematica_completa(
        robot, 
        puntos_x, 
        puntos_y;
        configuracion = configuracion[]
    )
    
    # Aplicar conversión a grados si es necesario
    if usar_grados[]
        theta1 = rad2deg.(theta1)
        theta2 = rad2deg.(theta2)
    end
    
    # Actualizar observables
    theta1_data[] = theta1
    theta2_data[] = theta2
    alcanzable_data[] = alcanzable

    # Calcular estadísticas (siempre en radianes para consistencia)
    theta1_rad, theta2_rad, _ = calcular_cinematica_completa(
        robot, 
        puntos_x, 
        puntos_y;
        configuracion = configuracion[]
    )
    
    valid_theta1 = filter(!isnan, theta1_rad)
    valid_theta2 = filter(!isnan, theta2_rad)
    
    n_total[] = length(alcanzable)
    n_alcanzables[] = sum(alcanzable)
    porcentaje_alcanzable[] = (n_alcanzables[] / n_total[]) * 100
    
    theta1_min[] = isempty(valid_theta1) ? 0.0 : minimum(valid_theta1)
    theta1_max[] = isempty(valid_theta1) ? 0.0 : maximum(valid_theta1)
    theta2_min[] = isempty(valid_theta2) ? 0.0 : minimum(valid_theta2)
    theta2_max[] = isempty(valid_theta2) ? 0.0 : maximum(valid_theta2)
    
    # Actualizar labels
    actualizar_stats_labels()
    
    # Ajustar límites de las gráficas
    autolimits!(ax_theta1)
    autolimits!(ax_theta2)
end

# ---------- Expresión matemática ----------
math_expression = lift(n, a, d, rm, s, c) do n_val, a_val, d_val, rm_val, s_val, c_val
    f_vals = trebol_function(θg, n_val, a_val, d_val, c_val)
    f_max = maximum(f_vals)
    
    if f_max ≈ 0.0
        return "r(θ) = $(round(s_val * rm_val; digits=3))"
    else
        return @sprintf "r(θ) = %.2f × [1 + %.3f × cos(%d(θ - %.3f - π)) - %.4f × cos(%d(θ - %.3f - π))] / %.3f" s_val*rm_val a_val n_val c_val d_val 2*n_val c_val f_max
    end
end

# ---------- Crear Figura Combinada ----------
fig = Figure(
    size = (1800, 1200),
    backgroundcolor = RGBf(0.98, 0.98, 0.98)
)

# Crear GridLayout principal
main_layout = GridLayout(fig[1, 1])

# Panel superior: Trébol (50% alto)
trebol_panel = main_layout[1, 1] = GridLayout()

# Panel inferior izquierdo: Ángulos theta1 y theta2 (25% alto)
angulos_panel = main_layout[2, 1] = GridLayout()

# Panel derecho: Controles (50% ancho)
control_panel = main_layout[1:2, 2] = GridLayout()

# ---------- Gráfica del Trébol ----------
ax_trebol = Axis(
    trebol_panel[1, 1], 
    xlabel = "Coordenada X (cm)",
    ylabel = "Coordenada Y (cm)", 
    title = "TRÉBOL ESTILIZADO - FORMA ACTUAL",
    titlesize = 18,
    xlabelsize = 14,
    ylabelsize = 14,
    aspect = DataAspect(),
    backgroundcolor = :white
)

# Trayectoria completa del trébol
lines!(ax_trebol, xygrid; 
       linewidth = 3, 
       color = RGBf(0.1, 0.3, 0.7))

# Trazo animado
trail_line = lines!(ax_trebol, xypartial; 
                    linewidth = 4, 
                    color = RGBf(0.8, 0.2, 0.2))

# Punto actual
current_point = scatter!(ax_trebol, xycurrent; 
                         markersize = 25, 
                         color = RGBf(0.2, 0.7, 0.2),
                         strokecolor = :black,
                         strokewidth = 2)

# Ajustar límites automáticamente
on(xygrid) do points
    xs = [p[1] for p in points]
    ys = [p[2] for p in points]
    padding = 0.15 * maximum(abs.(vcat(xs, ys)))
    xlims!(ax_trebol, minimum(xs) - padding, maximum(xs) + padding)
    ylims!(ax_trebol, minimum(ys) - padding, maximum(ys) + padding)
end

# Ecuación del trébol
equation_title = Label(trebol_panel[2, 1], "ECUACIÓN MATEMÁTICA ACTUAL:",
    fontsize = 16,
    color = :black,
    font = :bold,
    tellwidth = false)

equation_label = Label(trebol_panel[3, 1], math_expression,
    fontsize = 14,
    color = RGBf(0.1, 0.1, 0.5),
    font = :italic,
    tellwidth = false)

# ---------- Gráficas de Ángulos ----------
# Gráfica theta1
ax_theta1 = Axis(
    angulos_panel[1, 1],
    xlabel = "Índice de punto",
    ylabel = "θ₁ [rad]",
    title = "ÁNGULO θ₁ - CINEMÁTICA INVERSA EN VIVO",
    titlesize = 16,
    xlabelsize = 12,
    ylabelsize = 12,
    backgroundcolor = :white
)

theta1_line = lines!(ax_theta1, indices, theta1_data,
    color = RGBf(0.2, 0.4, 0.8),
    linewidth = 2,
    label = "θ₁"
)

# Gráfica theta2
ax_theta2 = Axis(
    angulos_panel[2, 1],
    xlabel = "Índice de punto",
    ylabel = "θ₂ [rad]",
    title = "ÁNGULO θ₂ - CINEMÁTICA INVERSA EN VIVO",
    titlesize = 16,
    xlabelsize = 12,
    ylabelsize = 12,
    backgroundcolor = :white
)

theta2_line = lines!(ax_theta2, indices, theta2_data,
    color = RGBf(0.8, 0.2, 0.2),
    linewidth = 2,
    label = "θ₂"
)

# ---------- Panel de Control Completo ----------

# Título principal
Label(control_panel[1, 1:2], "CONTROL INTEGRADO", 
      fontsize = 20, 
      font = :bold,
      color = RGBf(0.1, 0.1, 0.4))

# Grupo 1: Parámetros del Trébol
Label(control_panel[2, 1:2], "PARÁMETROS DEL TRÉBOL", 
      fontsize = 16, 
      color = RGBf(0.2, 0.4, 0.6))

# Sliders del trébol
s_n    = Slider(control_panel[3, 2], range = 2:1:12, startvalue = n[])
n_label = Label(control_panel[3, 1], "Número de hojas (n): $(Int(n[]))", tellwidth = false)

s_a0   = Slider(control_panel[4, 2], range = 0:0.01:2, startvalue = a0[])
a0_label = Label(control_panel[4, 1], "Forma base (a₀): $(round(a0[]; digits=3))", tellwidth = false)

s_d0   = Slider(control_panel[5, 2], range = 0.0:0.001:0.5, startvalue = d0[])
d0_label = Label(control_panel[5, 1], "Profundidad (d₀): $(round(d0[]; digits=4))", tellwidth = false)

s_rm   = Slider(control_panel[6, 2], range = 1:0.5:80, startvalue = rm[])
rm_label = Label(control_panel[6, 1], "Radio máximo (rm): $(round(rm[]; digits=1))", tellwidth = false)

s_s    = Slider(control_panel[7, 2], range = 0.5:0.01:1.5, startvalue = s[])
s_label = Label(control_panel[7, 1], "Factor escala (s): $(round(s[]; digits=2))", tellwidth = false)

s_c    = Slider(control_panel[8, 2], range = -π:0.01:π, startvalue = c[])
c_label = Label(control_panel[8, 1], "Rotación (c): $(round(c[]; digits=3)) rad", tellwidth = false)

# Grupo 2: Parámetros del Robot
Label(control_panel[9, 1:2], "PARÁMETROS DEL ROBOT", 
      fontsize = 16, 
      color = RGBf(0.2, 0.4, 0.6))

Label(control_panel[10, 1], "L1:", tellwidth = false)
Label(control_panel[10, 2], "16.0 cm", tellwidth = false, font = :bold)

Label(control_panel[11, 1], "L2:", tellwidth = false)
Label(control_panel[11, 2], "12.0 cm", tellwidth = false, font = :bold)

Label(control_panel[12, 1], "Offset Base X:", tellwidth = false)
Label(control_panel[12, 2], "+15.0 cm", tellwidth = false, font = :bold)

# Grupo 3: Configuración de Cinemática
Label(control_panel[13, 1:2], "CONFIGURACIÓN CINEMÁTICA", 
      fontsize = 16, 
      color = RGBf(0.2, 0.4, 0.6))

# CORREGIDO: Reemplazar Menu con ToggleButtons
config_layout = GridLayout(control_panel[14, 1:2])
Label(config_layout[1, 1], "Configuración:", tellwidth = false)
toggle_config = Toggle(config_layout[1, 2])
Label(config_layout[1, 3], "arriba", tellwidth = false)
Label(config_layout[1, 4], "abajo", tellwidth = false)

toggle_grados = Toggle(control_panel[15, 1], active = usar_grados[])
Label(control_panel[15, 2], "Mostrar en grados", tellwidth = false)

# Grupo 4: Estadísticas en Vivo
Label(control_panel[16, 1:2], "ESTADÍSTICAS EN VIVO", 
      fontsize = 16, 
      color = RGBf(0.2, 0.4, 0.6))

stats_label = Label(control_panel[17, 1:2], "Calculando...",
      fontsize = 13,
      tellwidth = false,
      justification = :left)

rango_label = Label(control_panel[18, 1:2], "---",
      fontsize = 13,
      tellwidth = false,
      justification = :left)

# Botón para actualizar cinemática
btn_actualizar = Button(control_panel[19, 1:2], label = "Actualizar Cinemática en Vivo")

# ---------- Funciones de Actualización ----------
function update_trebol_labels()
    n_label.text[] = "Número de hojas (n): $(Int(n[]))"
    a0_label.text[] = "Forma base (a₀): $(round(a0[]; digits=3))"
    d0_label.text[] = "Profundidad (d₀): $(round(d0[]; digits=4))"
    rm_label.text[] = "Radio máximo (rm): $(round(rm[]; digits=1))"
    s_label.text[] = "Factor escala (s): $(round(s[]; digits=2))"
    c_label.text[] = "Rotación (c): $(round(c[]; digits=3)) rad"
end

function actualizar_stats_labels()
    stats_text = """
    Puntos totales: $(n_total[])
    Alcanzables: $(n_alcanzables[]) $(@sprintf("%.2f", porcentaje_alcanzable[]))%
    No alcanzables: $(n_total[] - n_alcanzables[])
    """
    stats_label.text[] = stats_text
    
    if usar_grados[]
        θ1_min_val = rad2deg(theta1_min[])
        θ1_max_val = rad2deg(theta1_max[])
        θ2_min_val = rad2deg(theta2_min[])
        θ2_max_val = rad2deg(theta2_max[])
        unit = "°"
    else
        θ1_min_val = theta1_min[]
        θ1_max_val = theta1_max[]
        θ2_min_val = theta2_min[]
        θ2_max_val = theta2_max[]
        unit = "rad"
    end
    
    rango_text = """
    θ₁: [$(@sprintf("%.3f", θ1_min_val)), $(@sprintf("%.3f", θ1_max_val))] $unit
    θ₂: [$(@sprintf("%.3f", θ2_min_val)), $(@sprintf("%.3f", θ2_max_val))] $unit
    """
    rango_label.text[] = rango_text
end

function actualizar_unidades_graficas()
    if usar_grados[]
        ax_theta1.ylabel = "θ₁ [°]"
        ax_theta2.ylabel = "θ₂ [°]"
    else
        ax_theta1.ylabel = "θ₁ [rad]"
        ax_theta2.ylabel = "θ₂ [rad]"
    end
end

# ---------- Conectar Widgets ----------
# Conectar sliders del trébol
on(s_n.value) do v
    n[] = Int(round(v))
    update_trebol_labels()
    actualizar_cinematica_en_vivo()
end

on(s_a0.value) do v
    a0[] = v
    update_trebol_labels()
    actualizar_cinematica_en_vivo()
end

on(s_d0.value) do v
    d0[] = v
    update_trebol_labels()
    actualizar_cinematica_en_vivo()
end

on(s_rm.value) do v
    rm[] = v
    update_trebol_labels()
    actualizar_cinematica_en_vivo()
end

on(s_s.value) do v
    s[] = v
    update_trebol_labels()
    actualizar_cinematica_en_vivo()
end

on(s_c.value) do v
    c[] = v
    update_trebol_labels()
    actualizar_cinematica_en_vivo()
end

# Conectar configuración de cinemática - CORREGIDO
on(toggle_config.active) do active
    configuracion[] = active ? "arriba" : "abajo"
    actualizar_cinematica_en_vivo()
end

on(toggle_grados.active) do val
    usar_grados[] = val
    actualizar_unidades_graficas()
    actualizar_cinematica_en_vivo()
end

on(btn_actualizar.clicks) do _
    actualizar_cinematica_en_vivo()
end

# ---------- Animación del Trébol ----------
screen = display(fig)
last_t = time()

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
        pnew = 0.0
    end
    progress[] = pnew
end

# Ejecutar primera actualización
actualizar_cinematica_en_vivo()

# Ajustar espaciado
colgap!(main_layout, 20)
rowgap!(main_layout, 20)
colgap!(control_panel, 10)
rowgap!(control_panel, 8)
rowgap!(trebol_panel, 10)
rowgap!(angulos_panel, 15)

# Bloquear si no es interactivo
if !isinteractive()
    while isopen(screen)
        sleep(0.1)
    end
end