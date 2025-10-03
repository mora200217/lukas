using GLMakie
GLMakie.activate!()
using Observables
using Printf
using CSV
using DataFrames
using Dates
using LinearAlgebra

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

# ========================================
# SECCIÓN 1: ROBOT Y CINEMÁTICA
# ========================================

struct Robot2R
    L1::Float64
    L2::Float64
    base_offset_x::Float64
end

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

function exportar_angulos(puntos_x, puntos_y, theta1, theta2, alcanzable, robot)
    x_robot, y_robot = aplicar_offset_base(puntos_x, puntos_y, robot.base_offset_x)
    
    df = DataFrame(
        punto_index = 1:length(puntos_x),
        x_original = round.(puntos_x, digits=2),
        y_original = round.(puntos_y, digits=2),
        x_robot = round.(x_robot, digits=2),
        y_robot = round.(y_robot, digits=2),
        theta1_rad = round.(theta1, digits=2),
        theta2_rad = round.(theta2, digits=2),
        theta1_deg = round.(rad2deg.(theta1), digits=2),
        theta2_deg = round.(rad2deg.(theta2), digits=2),
        alcanzable = alcanzable
    )
    
    timestamp = Dates.format(Dates.now(), "yyyy-mm-dd_HH-MM-SS")
    output_filename = "Data/angulos_cinematica_$(timestamp).csv"
    CSV.write(output_filename, df)
    
    println("✅ Ángulos exportados a: $output_filename")
    return output_filename
end

# ========================================
# SECCIÓN 2: TRAYECTORIA TRÉBOL
# ========================================

function trebol_function(θ, n_val, a_val, d_val, c_val)
    return 1 .+ a_val .* cos.(n_val .* (θ .- c_val .- pi)) .- d_val .* cos.(2n_val .* (θ .- c_val .- pi))
end

function export_trajectory(n_val, a_val, d_val, rm_val, s_val, c_val, num_points)
    timestamp = Dates.format(now(), "yyyy-mm-dd_HH-MM-SS")
    filename = "Data/trajectory_$(timestamp).csv"
    
    θ_export = range(0, 2π; length=num_points)
    
    f_vals = trebol_function(θ_export, n_val, a_val, d_val, c_val)
    f_max = maximum(f_vals)
    
    if f_max ≈ 0.0
        r_export = fill(s_val * rm_val, length(θ_export))
    else
        r_export = s_val * rm_val .* (f_vals ./ f_max)
    end
    
    x_export = r_export .* cos.(θ_export .- pi)
    y_export = r_export .* sin.(θ_export .- pi)
    
    trajectory_data = DataFrame(
        theta = θ_export,
        x = x_export,
        y = y_export,
        r = r_export
    )
    
    CSV.write(filename, trajectory_data)
    
    println("Trayectoria exportada a: $filename")
    println("Número de puntos: $num_points")
    return filename, x_export, y_export
end

# ========================================
# APLICACIÓN PRINCIPAL
# ========================================

function crear_app_completa()
    
    # ---------- Observables Compartidos ----------
    n  = Observable(5)
    a0 = Observable(0.7)
    d0 = Observable(0.007)
    rm = Observable(10.0)
    s  = Observable(1.0)
    c  = Observable(0.0)
    ω  = Observable(0.8)
    
    playing    = Observable(true)
    looping    = Observable(true)
    draw_speed = Observable(0.6)
    progress   = Observable(0.0)
    θt         = Observable(0.0)
    trajectory_points = Observable(2001)
    
    θg = range(0, 2π; length=2001)
    
    a = lift(a0, n) do a0_val, n_val
        a0_val / n_val
    end
    
    d = lift(d0, n) do d0_val, n_val
        d0_val / n_val
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
    
    xycurrent = lift(θt, rcur) do θt_val, rcur_val
        [Point2f(rcur_val * cos(θt_val), rcur_val * sin(θt_val))]
    end
    
    # Cinemática
    robot = Robot2R(16.0, 12.0, 15.0)
    
    theta1_data = Observable(Float64[])
    theta2_data = Observable(Float64[])
    alcanzable_data = Observable(Bool[])
    indices = Observable(Int[])
    
    puntos_x_trayectoria = Observable(Float64[])
    puntos_y_trayectoria = Observable(Float64[])
    
    configuracion = Observable("arriba")
    usar_grados = Observable(false)
    datos_cinematica_cargados = Observable(false)
    
    n_total = Observable(0)
    n_alcanzables = Observable(0)
    porcentaje_alcanzable = Observable(0.0)
    theta1_min = Observable(0.0)
    theta1_max = Observable(0.0)
    theta2_min = Observable(0.0)
    theta2_max = Observable(0.0)
    
    # Control de pestañas
    active_tab = Observable(1)
    
    # ---------- Crear Figura ----------
    fig = Figure(
        size = (1600, 1000),
        backgroundcolor = RGBf(0.98, 0.98, 0.98)
    )
    
    # Botones de pestañas
    tab_buttons = GridLayout(fig[1, 1])
    tab_trebol_btn = Button(tab_buttons[1, 1], label = "TRAYECTORIA TRÉBOL")
    tab_cinematica_btn = Button(tab_buttons[1, 2], label = "CINEMÁTICA INVERSA")
    
    # Actualizar colores de botones según pestaña activa
    on(active_tab) do tab
        if tab == 1
            tab_trebol_btn.buttoncolor[] = RGBf(0.2, 0.6, 0.8)
            tab_cinematica_btn.buttoncolor[] = RGBf(0.7, 0.7, 0.7)
        else
            tab_trebol_btn.buttoncolor[] = RGBf(0.7, 0.7, 0.7)
            tab_cinematica_btn.buttoncolor[] = RGBf(0.2, 0.6, 0.8)
        end
    end
    
    # Dos layouts separados que ocupan el mismo espacio
    trebol_layout = GridLayout(fig[2, 1])
    cinematica_layout = GridLayout(fig[2, 1])
    
    # ========================================
    # PESTAÑA 1: TRAYECTORIA TRÉBOL
    # ========================================
    
    graph_panel_t = trebol_layout[1, 1] = GridLayout()
    control_panel_t = trebol_layout[1, 2] = GridLayout()
    
    ax_trebol = Axis(
        graph_panel_t[1, 1],
        xlabel = "Coordenada X",
        ylabel = "Coordenada Y",
        title = "TRÉBOL ESTILIZADO",
        titlesize = 20,
        xlabelsize = 16,
        ylabelsize = 16,
        aspect = DataAspect(),
        backgroundcolor = :white
    )
    
    lines!(ax_trebol, xygrid;
           linewidth = 3,
           color = RGBf(0.1, 0.3, 0.7))
    
    lines!(ax_trebol, xypartial;
           linewidth = 4,
           color = RGBf(0.8, 0.2, 0.2))
    
    scatter!(ax_trebol, xycurrent;
             markersize = 25,
             color = RGBf(0.2, 0.7, 0.2),
             strokecolor = :black,
             strokewidth = 2)
    
    on(xygrid) do points
        xs = [p[1] for p in points]
        ys = [p[2] for p in points]
        padding = 0.15 * maximum(abs.(vcat(xs, ys)))
        xlims!(ax_trebol, minimum(xs) - padding, maximum(xs) + padding)
        ylims!(ax_trebol, minimum(ys) - padding, maximum(ys) + padding)
    end
    
    equation_text = "Ecuación del Trébol:\nr(θ) = s × rm × [1 + a × cos(n(θ - c - π)) - d × cos(2n(θ - c))] / max"
    Label(graph_panel_t[2, 1], equation_text,
        fontsize = 14,
        color = :black,
        font = :bold,
        tellwidth = false)
    
    # Panel de control trébol
    Label(control_panel_t[1, 1:2], "PANEL DE CONTROL",
          fontsize = 18,
          font = :bold,
          color = RGBf(0.1, 0.1, 0.4))
    
    Label(control_panel_t[2, 1:2], "PARÁMETROS DE FORMA",
          fontsize = 16,
          color = RGBf(0.2, 0.4, 0.6))
    
    s_n = Slider(control_panel_t[3, 2], range = 2:1:12, startvalue = n[])
    n_label = Label(control_panel_t[3, 1], "Número de hojas (n): $(Int(n[]))",
          tellwidth = false)
    
    s_a0 = Slider(control_panel_t[4, 2], range = 0:0.01:2, startvalue = a0[])
    a0_label = Label(control_panel_t[4, 1], "Forma base (a₀): $(round(a0[]; digits=3))",
          tellwidth = false)
    
    s_d0 = Slider(control_panel_t[5, 2], range = 0.0:0.001:0.5, startvalue = d0[])
    d0_label = Label(control_panel_t[5, 1], "Profundidad (d₀): $(round(d0[]; digits=4))",
          tellwidth = false)
    
    Label(control_panel_t[6, 1:2], "ESCALA Y TAMAÑO",
          fontsize = 16,
          color = RGBf(0.2, 0.4, 0.6))
    
    s_rm = Slider(control_panel_t[7, 2], range = 1:0.5:80, startvalue = rm[])
    rm_label = Label(control_panel_t[7, 1], "Radio máximo (rm): $(round(rm[]; digits=1))",
          tellwidth = false)
    
    s_s = Slider(control_panel_t[8, 2], range = 0.5:0.01:1.5, startvalue = s[])
    s_label = Label(control_panel_t[8, 1], "Factor escala (s): $(round(s[]; digits=2))",
          tellwidth = false)
    
    Label(control_panel_t[9, 1:2], "ROTACIÓN Y MOVIMIENTO",
          fontsize = 16,
          color = RGBf(0.2, 0.4, 0.6))
    
    s_c = Slider(control_panel_t[10, 2], range = -π:0.01:π, startvalue = c[])
    c_label = Label(control_panel_t[10, 1], "Rotación (c): $(round(c[]; digits=3)) rad",
          tellwidth = false)
    
    s_ω = Slider(control_panel_t[11, 2], range = 0.0:0.05:3.0, startvalue = ω[])
    ω_label = Label(control_panel_t[11, 1], "Velocidad (ω): $(round(ω[]; digits=2)) rad/s",
          tellwidth = false)
    
    Label(control_panel_t[12, 1:2], "CONTROL DE ANIMACIÓN",
          fontsize = 16,
          color = RGBf(0.2, 0.4, 0.6))
    
    s_draw = Slider(control_panel_t[13, 2], range = 0.01:0.01:3.0, startvalue = draw_speed[])
    draw_label = Label(control_panel_t[13, 1], "Velocidad dibujo: $(round(draw_speed[]; digits=2))",
          tellwidth = false)
    
    Label(control_panel_t[14, 1:2], "EXPORTACIÓN",
          fontsize = 16,
          color = RGBf(0.2, 0.4, 0.6))
    
    Label(control_panel_t[15, 1], "Puntos trayectoria:", tellwidth = false)
    s_points = Slider(control_panel_t[15, 2], range = 100:100:10000, startvalue = trajectory_points[])
    points_value_label = Label(control_panel_t[15, 3], "$(trajectory_points[])", tellwidth = false)
    
    export_buttons_layout = GridLayout(control_panel_t[16, 1:2])
    btn_export_traj = Button(export_buttons_layout[1, 1], label = "Exportar Trayectoria")
    btn_generar_cinematica = Button(export_buttons_layout[1, 2], label = "▶️ Analizar Cinemática")
    
    button_layout = GridLayout(control_panel_t[17, 1:2])
    btn_play = Button(button_layout[1, 1], label = "Pausar")
    btn_reset = Button(button_layout[1, 2], label = "Reiniciar")
    t_loop = Toggle(button_layout[1, 3], active = looping[])
    loop_label = Label(button_layout[1, 4], "Bucle: ON")
    
    # ========================================
    # PESTAÑA 2: CINEMÁTICA INVERSA
    # ========================================
    
    graph_panel_c = cinematica_layout[1, 1] = GridLayout()
    control_panel_c = cinematica_layout[1, 2] = GridLayout()
    
    ax1 = Axis(
        graph_panel_c[1, 1],
        xlabel = "Índice de punto",
        ylabel = "θ₁ [rad]",
        title = "Ángulo θ₁ a lo largo de la trayectoria",
        titlesize = 18,
        xlabelsize = 14,
        ylabelsize = 14,
        backgroundcolor = :white
    )
    
    lines!(ax1, indices, theta1_data,
        color = RGBf(0.2, 0.4, 0.8),
        linewidth = 2.5,
        label = "θ₁"
    )
    
    indices_no_alcanzables_1 = @lift(findall(.!$alcanzable_data))
    theta1_no_alcanzable = @lift(
        isempty($indices_no_alcanzables_1) ? Float64[] : $theta1_data[$indices_no_alcanzables_1]
    )
    indices_no_alc_vals_1 = @lift(
        isempty($indices_no_alcanzables_1) ? Int[] : $indices[$indices_no_alcanzables_1]
    )
    
    scatter!(ax1, indices_no_alc_vals_1, theta1_no_alcanzable,
        color = :red,
        marker = :xcross,
        markersize = 15,
        label = "No alcanzable"
    )
    
    ax2 = Axis(
        graph_panel_c[2, 1],
        xlabel = "Índice de punto",
        ylabel = "θ₂ [rad]",
        title = "Ángulo θ₂ a lo largo de la trayectoria",
        titlesize = 18,
        xlabelsize = 14,
        ylabelsize = 14,
        backgroundcolor = :white
    )
    
    lines!(ax2, indices, theta2_data,
        color = RGBf(0.8, 0.2, 0.2),
        linewidth = 2.5,
        label = "θ₂"
    )
    
    indices_no_alcanzables_2 = @lift(findall(.!$alcanzable_data))
    theta2_no_alcanzable = @lift(
        isempty($indices_no_alcanzables_2) ? Float64[] : $theta2_data[$indices_no_alcanzables_2]
    )
    indices_no_alc_vals_2 = @lift(
        isempty($indices_no_alcanzables_2) ? Int[] : $indices[$indices_no_alcanzables_2]
    )
    
    scatter!(ax2, indices_no_alc_vals_2, theta2_no_alcanzable,
        color = :red,
        marker = :xcross,
        markersize = 15,
        label = "No alcanzable"
    )
    
    on(usar_grados) do grados
        if grados
            ax1.ylabel = "θ₁ [°]"
            ax2.ylabel = "θ₂ [°]"
        else
            ax1.ylabel = "θ₁ [rad]"
            ax2.ylabel = "θ₂ [rad]"
        end
    end
    
    # Panel de control cinemática
    Label(control_panel_c[1, 1:2], "ANÁLISIS DE CINEMÁTICA INVERSA",
          fontsize = 18,
          font = :bold,
          color = RGBf(0.1, 0.1, 0.4))
    
    Label(control_panel_c[2, 1:2], "PARÁMETROS DEL ROBOT",
          fontsize = 16,
          color = RGBf(0.2, 0.4, 0.6))
    
    Label(control_panel_c[3, 1], "L1:", tellwidth = false)
    Label(control_panel_c[3, 2], "16.0 cm", tellwidth = false, font = :bold)
    
    Label(control_panel_c[4, 1], "L2:", tellwidth = false)
    Label(control_panel_c[4, 2], "12.0 cm", tellwidth = false, font = :bold)
    
    Label(control_panel_c[5, 1], "Offset Base X:", tellwidth = false)
    Label(control_panel_c[5, 2], "+15.0 cm", tellwidth = false, font = :bold)
    
    Label(control_panel_c[6, 1:2], "CONFIGURACIÓN",
          fontsize = 16,
          color = RGBf(0.2, 0.4, 0.6))
    
    menu_config = Menu(control_panel_c[7, 1:2],
                       options = ["arriba", "abajo"],
                       default = "arriba")
    
    toggle_grados = Toggle(control_panel_c[8, 1], active = usar_grados[])
    Label(control_panel_c[8, 2], "Mostrar en grados", tellwidth = false)
    
    Label(control_panel_c[9, 1:2], "ESTADÍSTICAS",
          fontsize = 16,
          color = RGBf(0.2, 0.4, 0.6))
    
    stats_label = Label(control_panel_c[10, 1:2], "No hay datos cargados",
          fontsize = 13,
          tellwidth = false,
          justification = :left)
    
    Label(control_panel_c[11, 1:2], "RANGO DE ÁNGULOS",
          fontsize = 16,
          color = RGBf(0.2, 0.4, 0.6))
    
    rango_label = Label(control_panel_c[12, 1:2], "---",
          fontsize = 13,
          tellwidth = false,
          justification = :left)
    
    status_label_c = Label(control_panel_c[13, 1:2], "Genera una trayectoria primero",
                          fontsize = 12,
                          color = :gray,
                          tellwidth = false)
    
    btn_exportar = Button(control_panel_c[14, 1:2], label = "Exportar Ángulos CSV")
    
    # ---------- Control de visibilidad de pestañas ----------
    # ---------- Control de visibilidad de pestañas ----------

# Hacer todos los elementos de cinematica invisibles inicialmente
for block in contents(cinematica_layout)
    block.visible = false
end

on(active_tab) do tab
    if tab == 1
        # Mostrar trébol, ocultar cinemática
        for block in contents(trebol_layout)
            block.visible = true
        end
        for block in contents(cinematica_layout)
            block.visible = false
        end
    else
        # Mostrar cinemática, ocultar trébol
        for block in contents(trebol_layout)
            block.visible = false
        end
        for block in contents(cinematica_layout)
            block.visible = true
        end
    end
end

on(tab_trebol_btn.clicks) do _
    active_tab[] = 1
end

on(tab_cinematica_btn.clicks) do _
    active_tab[] = 2
end
 
    # ---------- FUNCIONES ----------
    
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
        points_value_label.text[] = "$(trajectory_points[])"
    end
    
    function actualizar_stats_labels()
        if datos_cinematica_cargados[]
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
        else
            stats_label.text[] = "No hay datos cargados"
            rango_label.text[] = "---"
        end
    end
    
    function actualizar_graficas_cinematica()
        if !datos_cinematica_cargados[]
            return
        end
        
        theta1, theta2, alcanzable = calcular_cinematica_completa(
            robot,
            puntos_x_trayectoria[],
            puntos_y_trayectoria[];
            configuracion = configuracion[]
        )
        
        if usar_grados[]
            theta1 = rad2deg.(theta1)
            theta2 = rad2deg.(theta2)
        end
        
        theta1_data[] = theta1
        theta2_data[] = theta2
        alcanzable_data[] = alcanzable
        indices[] = collect(1:length(theta1))
        
        theta1_rad, theta2_rad, _ = calcular_cinematica_completa(
            robot,
            puntos_x_trayectoria[],
            puntos_y_trayectoria[];
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
        
        actualizar_stats_labels()
        
        autolimits!(ax1)
        autolimits!(ax2)
        
        if n_alcanzables[] == n_total[]
            status_label_c.text[] = "✅ Todos los puntos son alcanzables"
            status_label_c.color[] = RGBf(0.0, 0.6, 0.0)
        else
            status_label_c.text[] = "⚠️ Algunos puntos no son alcanzables"
            status_label_c.color[] = RGBf(1.0, 0.6, 0.0)
        end
    end
    
    function generar_y_analizar_cinematica()
        num_points = trajectory_points[]
        θ_export = range(0, 2π; length=num_points)
        
        f_vals = trebol_function(θ_export, n[], a[], d[], c[])
        f_max = maximum(f_vals)
        
        if f_max ≈ 0.0
            r_export = fill(s[] * rm[], length(θ_export))
        else
            r_export = s[] * rm[] .* (f_vals ./ f_max)
        end
        
        x_export = r_export .* cos.(θ_export .- pi)
        y_export = r_export .* sin.(θ_export .- pi)
        
        puntos_x_trayectoria[] = x_export
        puntos_y_trayectoria[] = y_export
        datos_cinematica_cargados[] = true
        
        actualizar_graficas_cinematica()
        
        active_tab[] = 2
        
        println("✅ Trayectoria generada y cargada en análisis de cinemática: $(length(x_export)) puntos")
    end
    
    function exportar_trayectoria()
        filename, x_exp, y_exp = export_trajectory(n[], a[], d[], rm[], s[], c[], trajectory_points[])
        println("✅ Trayectoria exportada")
    end
    
    function exportar_datos_cinematica()
        if !datos_cinematica_cargados[]
            status_label_c.text[] = "❌ No hay datos para exportar"
            status_label_c.color[] = RGBf(0.8, 0.0, 0.0)
            return
        end
        
        try
            theta1_rad, theta2_rad, alcanzable = calcular_cinematica_completa(
                robot,
                puntos_x_trayectoria[],
                puntos_y_trayectoria[];
                configuracion = configuracion[]
            )
            
            exportar_angulos(
                puntos_x_trayectoria[],
                puntos_y_trayectoria[],
                theta1_rad,
                theta2_rad,
                alcanzable,
                robot
            )
            
            status_label_c.text[] = "✅ Ángulos exportados exitosamente"
            status_label_c.color[] = RGBf(0.0, 0.6, 0.0)
        catch e
            error_msg = sprint(showerror, e)
            status_label_c.text[] = "❌ Error al exportar"
            status_label_c.color[] = RGBf(0.8, 0.0, 0.0)
            println("Error al exportar: $error_msg")
        end
    end
    
    function toggle_animation()
        playing[] = !playing[]
        update_labels()
    end
    
    function reset_animation()
        progress[] = 0.0
        θt[] = 0.0
        if !playing[]
            playing[] = true
        end
        update_labels()
    end
    
    # ---------- CONECTAR CALLBACKS ----------
    
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
    on(trajectory_points) do _; update_labels() end
    
    on(s_n.value) do v; n[] = Int(round(v)) end
    on(s_a0.value) do v; a0[] = v end
    on(s_d0.value) do v; d0[] = v end
    on(s_rm.value) do v; rm[] = v end
    on(s_s.value) do v; s[] = v end
    on(s_c.value) do v; c[] = v end
    on(s_ω.value) do v; ω[] = v end
    on(s_draw.value) do v; draw_speed[] = v end
    on(s_points.value) do v; trajectory_points[] = Int(round(v)) end
    
    on(btn_play.clicks) do _; toggle_animation() end
    on(btn_reset.clicks) do _; reset_animation() end
    on(t_loop.active) do val; looping[] = val; update_labels() end
    
    on(btn_export_traj.clicks) do _; exportar_trayectoria() end
    on(btn_generar_cinematica.clicks) do _; generar_y_analizar_cinematica() end
    
    on(menu_config.selection) do sel
        configuracion[] = sel
        actualizar_graficas_cinematica()
    end
    
    on(toggle_grados.active) do val
        usar_grados[] = val
        actualizar_graficas_cinematica()
    end
    
    on(btn_exportar.clicks) do _; exportar_datos_cinematica() end
    
    # Ajustar espaciado
    colgap!(trebol_layout, 30)
    colgap!(cinematica_layout, 30)
    rowgap!(control_panel_t, 12)
    rowgap!(control_panel_c, 10)
    rowgap!(graph_panel_c, 20)
    
    # Mostrar figura
    screen = display(fig)
    
    # Animación
    last_t = time()
    
    @async while isopen(screen)
        sleep(1/120)
        now = time()
        dt = now - last_t
        last_t = now
        
        if playing[]
            θt[] = (θt[] + ω[] * dt) % (2π)
            
            p = progress[]
            pnew = p + dt * draw_speed[]
            if pnew >= 1.0
                if looping[]
                    pnew = 0.0
                else
                    pnew = 1.0
                    playing[] = false
                    update_labels()
                end
            end
            progress[] = pnew
        end
    end
    
    return fig
end

# ---------- Lanzar Aplicación ----------
fig = crear_app_completa()

if !isinteractive()
    while isopen(fig.scene)
        sleep(0.1)
    end
end