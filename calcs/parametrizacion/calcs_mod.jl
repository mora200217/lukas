using GLMakie
GLMakie.activate!()
using Observables
using Printf
using CSV
using DataFrames
using Dates
using LinearAlgebra

# ========== ESTRUCTURAS Y FUNCIONES DEL ROBOT ==========

struct Robot2R
    L1::Float64      # Longitud eslabón 1
    L2::Float64      # Longitud eslabón 2
    base_offset_x::Float64  # Desplazamiento de la base en X
end

# ========== FUNCIONES DE CINEMÁTICA INVERSA ==========

function aplicar_offset_base(x, y, offset)
    return x .+ offset, y
end

function cinematica_inversa_punto(robot::Robot2R, x, y)
    # Calcular distancia al origen (base del robot)
    d = sqrt(x^2 + y^2)
    
    # Verificar si el punto es alcanzable
    d_max = robot.L1 + robot.L2
    d_min = abs(robot.L1 - robot.L2)
    
    if d > d_max || d < d_min
        return nothing, nothing, false  # Punto no alcanzable
    end
    
    # Cálculos de cinemática inversa
    c2 = (x^2 + y^2 - robot.L1^2 - robot.L2^2) / (2 * robot.L1 * robot.L2)
    
    # Manejar errores numéricos en arcos
    c2 = clamp(c2, -1.0, 1.0)
    
    s2_pos = sqrt(1 - c2^2)   # Solución codo arriba
    s2_neg = -sqrt(1 - c2^2)  # Solución codo abajo
    
    theta2_pos = atan(s2_pos, c2)
    theta2_neg = atan(s2_neg, c2)
    
    # Calcular theta1 para cada solución
    k1 = robot.L1 + robot.L2 * c2
    k2_pos = robot.L2 * s2_pos
    k2_neg = robot.L2 * s2_neg
    
    theta1_pos = atan(y, x) - atan(k2_pos, k1)
    theta1_neg = atan(y, x) - atan(k2_neg, k1)
    
    return [theta1_pos, theta2_pos], [theta1_neg, theta2_neg], true
end

function calcular_cinematica_completa(robot::Robot2R, puntos_x, puntos_y; configuracion="arriba")
    n_puntos = length(puntos_x)
    
    # Aplicar offset de la base
    x_robot, y_robot = aplicar_offset_base(puntos_x, puntos_y, robot.base_offset_x)
    
    # Arrays para almacenar resultados
    theta1_tray = zeros(n_puntos)
    theta2_tray = zeros(n_puntos)
    alcanzable = fill(false, n_puntos)
    
    # Validar cada punto de la trayectoria
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

function leer_csv_trayectoria(filename)
    if !isfile(filename)
        error("El archivo $filename no existe")
    end
    
    df = CSV.read(filename, DataFrame)
    
    # Verificar que tenga las columnas necesarias
    if !("x" in names(df)) || !("y" in names(df))
        error("El CSV debe contener columnas 'x' e 'y'")
    end
    
    return df.x, df.y
end

function exportar_angulos(filename, puntos_x, puntos_y, theta1, theta2, alcanzable, robot)
    x_robot, y_robot = aplicar_offset_base(puntos_x, puntos_y, robot.base_offset_x)
    
    # Crear carpeta Data si no existe
    if !isdir("Data")
        mkdir("Data")
    end
    
    # Redondear todos los valores a 2 decimales
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

# ========== FUNCIONES DEL TRÉBOL ==========

# Función base del trébol
function trebol_function(θ, n_val, a_val, d_val, c_val)
    return 1 .+ a_val .* cos.(n_val .* (θ .- c_val .- pi)) .- d_val .* cos.(2n_val .* (θ .- c_val .- pi))
end

function export_parameters_trebol(n, a0, d0, rm, s, c, θg)
    timestamp = Dates.format(now(), "yyyy-mm-dd_HH-MM-SS")
    filename = "Data/param_$(timestamp).csv"
    
    # Calcular f_max para exportación
    f_vals = trebol_function(θg, n, a0/n, d0/n, c)
    f_max = maximum(f_vals)
    
    params_dict = Dict(
        "n" => n,
        "a0" => a0,
        "d0" => d0,
        "rm" => rm,
        "s" => s,
        "c" => c,
        "f_max" => f_max
    )
    
    df = DataFrame(params_dict)
    CSV.write(filename, df)
    
    println("Parámetros exportados a: $filename")
    println("f_max exportado: $f_max")
    return filename
end

function export_trajectory_trebol(n, a0, d0, rm, s, c, trajectory_points)
    timestamp = Dates.format(now(), "yyyy-mm-dd_HH-MM-SS")
    filename = "Data/trajectory_$(timestamp).csv"
    
    # Crear malla angular con el número de puntos especificado
    num_points = trajectory_points
    θ_export = range(0, 2π; length=num_points)
    
    # Calcular la trayectoria completa
    f_vals = trebol_function(θ_export, n, a0/n, d0/n, c)
    f_max = maximum(f_vals)
    
    if f_max ≈ 0.0
        r_export = fill(s * rm, length(θ_export))
    else
        r_export = s * rm .* (f_vals ./ f_max)
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

# ========== APLICACIÓN PRINCIPAL INTEGRADA ==========

function crear_aplicacion_integrada()
    # Configuración de tema visual
    set_theme!(theme_light())
    update_theme!(
        fontsize = 14,
        Label = (textcolor = :black,),
        Slider = (color_active = RGBf(0.2, 0.6, 0.8), color_inactive = RGBf(0.7, 0.7, 0.7)),
        Button = (buttoncolor = RGBf(0.2, 0.6, 0.8), labelcolor = :white),
        Toggle = (buttoncolor = RGBf(0.2, 0.6, 0.8), labelcolor = :white)
    )

    # Crear carpeta Data si no existe
    if !isdir("Data")
        mkdir("Data")
    end

    # ========== VARIABLES COMPARTIDAS ==========
    
    # Robot para cinemática inversa
    robot = Robot2R(16.0, 12.0, 15.0)  # L1=16cm, L2=12cm, offset=15cm
    
    # ========== OBSERVABLES DEL TRÉBOL ==========
    
    n  = Observable(5)         # número de hojas
    a0 = Observable(0.7)       # estilización forma base
    d0 = Observable(0.007)     # estilización trébol
    rm = Observable(10.0)      # radio máximo objetivo
    s  = Observable(1.0)       # escala global (0.75..1.25)
    c  = Observable(0.0)       # rotación (rad)

    # animación
    progress   = Observable(0.0)   # 0..1 progreso del trazo
    θt         = Observable(0.0)   # ángulo del cursor

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

    # ---------- Expresión matemática de r(θ) ----------
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

    # ========== OBSERVABLES DE CINEMÁTICA INVERSA ==========
    
    theta1_data = Observable(Float64[])
    theta2_data = Observable(Float64[])
    alcanzable_data = Observable(Bool[])
    indices = Observable(Int[])
    
    puntos_x_original = Observable(Float64[])
    puntos_y_original = Observable(Float64[])
    
    configuracion = Observable("arriba")
    usar_grados = Observable(false)
    
    # CORREGIDO: Inicializar con string vacío en lugar de Observable
    csv_path_str = ""  # Ruta por defecto vacía
    datos_cargados = Observable(false)
    
    # Estadísticas
    n_total = Observable(0)
    n_alcanzables = Observable(0)
    porcentaje_alcanzable = Observable(0.0)
    theta1_min = Observable(0.0)
    theta1_max = Observable(0.0)
    theta2_min = Observable(0.0)
    theta2_max = Observable(0.0)

    # ========== CREAR FIGURA PRINCIPAL ==========
    
    fig = Figure(
        size = (1800, 1200),
        backgroundcolor = RGBf(0.98, 0.98, 0.98)
    )

    # Crear un GridLayout principal
    main_layout = GridLayout(fig[1, 1])

    # Título principal
    Label(fig[0, :], "SISTEMA INTEGRADO: GENERACIÓN DE TRAYECTORIA Y CINEMÁTICA INVERSA", 
          fontsize=24, font=:bold, color=RGBf(0.1, 0.1, 0.4))

    # ========== PESTAÑA 1: GENERACIÓN DE TRÉBOL ==========
    
    trebol_panel = GridLayout()
    main_layout[1, 1] = trebol_panel

    # Panel izquierdo para la gráfica (70% del ancho)
    graph_panel = trebol_panel[1, 1] = GridLayout()
    # Panel derecho para controles (30% del ancho)
    control_panel = trebol_panel[1, 2] = GridLayout()

    # ---------- Gráfica Principal del Trébol ----------
    ax = Axis(
        graph_panel[1, 1], 
        xlabel = "Coordenada X",
        ylabel = "Coordenada Y", 
        title = "TRÉBOL ESTILIZADO - TRAYECTORIA GENERADA",
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

    # ---------- Panel de Control del Trébol ----------

    # Título del panel de control
    Label(control_panel[1, 1:2], "CONTROL DEL TRÉBOL", 
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

    # Botón para cargar automáticamente en cinemática inversa
    btn_cargar_automatico = Button(control_panel[14, 1:2], label = "Generar y Cargar en Cinemática Inversa")

    # ========== PESTAÑA 2: CINEMÁTICA INVERSA ==========
    
    cinematica_panel = GridLayout()
    main_layout[1, 1] = cinematica_panel  # Se mostrará una u otra

    # Panel izquierdo para gráficas (70%)
    graph_panel_cin = cinematica_panel[1, 1] = GridLayout()
    # Panel derecho para controles (30%)
    control_panel_cin = cinematica_panel[1, 2] = GridLayout()

    # ---------- Gráficas de Cinemática ----------
    
    # Gráfica 1: θ₁
    ax1 = Axis(
        graph_panel_cin[1, 1],
        xlabel = "Índice de punto",
        ylabel = "θ₁ [rad]",
        title = "Ángulo θ₁ a lo largo de la trayectoria",
        titlesize = 18,
        xlabelsize = 14,
        ylabelsize = 14,
        backgroundcolor = :white
    )
    
    # Línea principal θ₁
    theta1_line = lines!(ax1, indices, theta1_data,
        color = RGBf(0.2, 0.4, 0.8),
        linewidth = 2.5,
        label = "θ₁"
    )
    
    # Marcadores para puntos no alcanzables en θ₁
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
    
    # Gráfica 2: θ₂
    ax2 = Axis(
        graph_panel_cin[2, 1],
        xlabel = "Índice de punto",
        ylabel = "θ₂ [rad]",
        title = "Ángulo θ₂ a lo largo de la trayectoria",
        titlesize = 18,
        xlabelsize = 14,
        ylabelsize = 14,
        backgroundcolor = :white
    )
    
    # Línea principal θ₂
    theta2_line = lines!(ax2, indices, theta2_data,
        color = RGBf(0.8, 0.2, 0.2),
        linewidth = 2.5,
        label = "θ₂"
    )
    
    # Marcadores para puntos no alcanzables en θ₂
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
    
    # Actualizar labels de ejes según unidades
    on(usar_grados) do grados
        if grados
            ax1.ylabel = "θ₁ [°]"
            ax2.ylabel = "θ₂ [°]"
        else
            ax1.ylabel = "θ₁ [rad]"
            ax2.ylabel = "θ₂ [rad]"
        end
    end
    
    # ---------- Panel de Control de Cinemática ----------
    
    # Título
    Label(control_panel_cin[1, 1:2], "ANÁLISIS DE CINEMÁTICA INVERSA", 
          fontsize = 18, 
          font = :bold,
          color = RGBf(0.1, 0.1, 0.4))
    
    # Parámetros del robot
    Label(control_panel_cin[2, 1:2], "PARÁMETROS DEL ROBOT", 
          fontsize = 16, 
          color = RGBf(0.2, 0.4, 0.6))
    
    Label(control_panel_cin[3, 1], "L1:", tellwidth = false)
    Label(control_panel_cin[3, 2], "16.0 cm", tellwidth = false, font = :bold)
    
    Label(control_panel_cin[4, 1], "L2:", tellwidth = false)
    Label(control_panel_cin[4, 2], "12.0 cm", tellwidth = false, font = :bold)
    
    Label(control_panel_cin[5, 1], "Offset Base X:", tellwidth = false)
    Label(control_panel_cin[5, 2], "+15.0 cm", tellwidth = false, font = :bold)
    
    # Archivo CSV - CORREGIDO: Usar TextField en lugar de Textbox
    Label(control_panel_cin[6, 1:2], "ARCHIVO CSV", 
          fontsize = 16, 
          color = RGBf(0.2, 0.4, 0.6))
    
    Label(control_panel_cin[7, 1], "Ruta:", tellwidth = false)
    # CORREGIDO: Usar TextField que es más estable
    csv_textfield = TextField(control_panel_cin[7, 2], placeholder = "Ruta del archivo CSV")
    
    btn_cargar = Button(control_panel_cin[8, 1:2], label = "Cargar Trayectoria")
    
    # Estado
    status_label = Label(control_panel_cin[9, 1:2], "Esperando cargar datos...", 
                        fontsize = 12, 
                        color = :gray,
                        tellwidth = false)
    
    # Configuración
    Label(control_panel_cin[10, 1:2], "CONFIGURACIÓN", 
          fontsize = 16, 
          color = RGBf(0.2, 0.4, 0.6))
    
    # Toggle codo arriba/abajo
    menu_config = Menu(control_panel_cin[11, 1:2], 
                       options = ["arriba", "abajo"],
                       default = "arriba")
    
    # Toggle radianes/grados
    toggle_grados = Toggle(control_panel_cin[12, 1], active = usar_grados[])
    Label(control_panel_cin[12, 2], "Mostrar en grados", tellwidth = false)
    
    # Estadísticas
    Label(control_panel_cin[13, 1:2], "ESTADÍSTICAS", 
          fontsize = 16, 
          color = RGBf(0.2, 0.4, 0.6))
    
    stats_label = Label(control_panel_cin[14, 1:2], "No hay datos cargados",
          fontsize = 13,
          tellwidth = false,
          justification = :left)
    
    # Rango de ángulos
    Label(control_panel_cin[15, 1:2], "RANGO DE ÁNGULOS", 
          fontsize = 16, 
          color = RGBf(0.2, 0.4, 0.6))
    
    rango_label = Label(control_panel_cin[16, 1:2], "---",
          fontsize = 13,
          tellwidth = false,
          justification = :left)
    
    # Botón exportar
    btn_exportar = Button(control_panel_cin[17, 1:2], label = "Exportar Ángulos CSV")

    # ========== BOTONES DE NAVEGACIÓN ==========
    
    nav_layout = GridLayout(fig[2, :])
    btn_ir_trebol = Button(nav_layout[1, 1], label = "← Ir a Generación de Trébol")
    btn_ir_cinematica = Button(nav_layout[1, 2], label = "Ir a Cinemática Inversa →")

    # Inicialmente mostrar solo el trébol
    cinematica_panel.visible = false

    # ========== FUNCIONES DE ACTUALIZACIÓN ==========

    # Función para actualizar labels del trébol
    function update_labels_trebol()
        n_label.text[] = "Número de hojas (n): $(Int(n[]))"
        a0_label.text[] = "Forma base (a₀): $(round(a0[]; digits=3))"
        d0_label.text[] = "Profundidad (d₀): $(round(d0[]; digits=4))"
        rm_label.text[] = "Radio máximo (rm): $(round(rm[]; digits=1))"
        s_label.text[] = "Factor escala (s): $(round(s[]; digits=2))"
        c_label.text[] = "Rotación (c): $(round(c[]; digits=3)) rad"
        points_value_label.text[] = "$(trajectory_points[])"
    end

    # Función para actualizar estadísticas de cinemática
    function actualizar_stats_labels()
        if datos_cargados[]
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
        if !datos_cargados[]
            return
        end
        
        # Calcular cinemática inversa
        theta1, theta2, alcanzable = calcular_cinematica_completa(
            robot, 
            puntos_x_original[], 
            puntos_y_original[];
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
        indices[] = collect(1:length(theta1))
        
        # Calcular estadísticas (siempre en radianes para consistencia)
        theta1_rad, theta2_rad, _ = calcular_cinematica_completa(
            robot, 
            puntos_x_original[], 
            puntos_y_original[];
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
        
        # Ajustar límites de las gráficas
        autolimits!(ax1)
        autolimits!(ax2)
        
        if n_alcanzables[] == n_total[]
            status_label.text[] = "✅ Todos los puntos son alcanzables"
            status_label.color[] = RGBf(0.0, 0.6, 0.0)
        else
            status_label.text[] = "⚠️ Algunos puntos no son alcanzables"
            status_label.color[] = RGBf(1.0, 0.6, 0.0)
        end
    end

    function cargar_trayectoria_cinematica()
        try
            path = csv_textfield.value[]
            puntos_x, puntos_y = leer_csv_trayectoria(path)
            
            puntos_x_original[] = puntos_x
            puntos_y_original[] = puntos_y
            datos_cargados[] = true
            
            actualizar_graficas_cinematica()
            
            println("✅ Trayectoria cargada: $(length(puntos_x)) puntos")
        catch e
            error_msg = sprint(showerror, e)
            status_label.text[] = "❌ Error al cargar archivo"
            status_label.color[] = RGBf(0.8, 0.0, 0.0)
            println("Error al cargar archivo: $error_msg")
        end
    end

    function exportar_datos_cinematica()
        if !datos_cargados[]
            status_label.text[] = "❌ No hay datos para exportar"
            status_label.color[] = RGBf(0.8, 0.0, 0.0)
            return
        end
        
        try
            theta1_rad, theta2_rad, alcanzable = calcular_cinematica_completa(
                robot, 
                puntos_x_original[], 
                puntos_y_original[];
                configuracion = configuracion[]
            )
            
            exportar_angulos(
                "angulos_export.csv",
                puntos_x_original[],
                puntos_y_original[],
                theta1_rad,
                theta2_rad,
                alcanzable,
                robot
            )
            
            status_label.text[] = "✅ Ángulos exportados exitosamente"
            status_label.color[] = RGBf(0.0, 0.6, 0.0)
        catch e
            error_msg = sprint(showerror, e)
            status_label.text[] = "❌ Error al exportar"
            status_label.color[] = RGBf(0.8, 0.0, 0.0)
            println("Error al exportar: $error_msg")
        end
    end

    function generar_y_cargar_automatico()
        # Exportar trayectoria actual
        traj_file = export_trajectory_trebol(n[], a0[], d0[], rm[], s[], c[], trajectory_points[])
        
        # Actualizar ruta en el textfield de cinemática
        csv_textfield.value[] = traj_file
        
        # Cambiar a pestaña de cinemática inversa
        trebol_panel.visible = false
        cinematica_panel.visible = true
        
        # Cargar automáticamente
        cargar_trayectoria_cinematica()
        
        println("✅ Trayectoria generada y cargada automáticamente en cinemática inversa")
    end

    # ========== CONEXIÓN DE EVENTOS ==========

    # Conexiones del trébol
    on(n) do _; update_labels_trebol() end
    on(a0) do _; update_labels_trebol() end
    on(d0) do _; update_labels_trebol() end
    on(rm) do _; update_labels_trebol() end
    on(s) do _; update_labels_trebol() end
    on(c) do _; update_labels_trebol() end
    on(trajectory_points) do _; update_labels_trebol() end

    on(s_n.value) do v; n[] = Int(round(v)) end
    on(s_a0.value) do v; a0[] = v end
    on(s_d0.value) do v; d0[] = v end
    on(s_rm.value) do v; rm[] = v end
    on(s_s.value) do v; s[] = v end
    on(s_c.value) do v; c[] = v end
    on(s_points.value) do v; trajectory_points[] = Int(round(v)) end

    on(btn_export_params.clicks) do _
        export_parameters_trebol(n[], a0[], d0[], rm[], s[], c[], θg)
    end
    on(btn_export_traj.clicks) do _
        export_trajectory_trebol(n[], a0[], d0[], rm[], s[], c[], trajectory_points[])
    end
    on(btn_export_all.clicks) do _
        export_parameters_trebol(n[], a0[], d0[], rm[], s[], c[], θg)
        export_trajectory_trebol(n[], a0[], d0[], rm[], s[], c[], trajectory_points[])
        println("Exportación completa")
    end
    on(btn_cargar_automatico.clicks) do _; generar_y_cargar_automatico() end

    # Conexiones de cinemática inversa
    on(btn_cargar.clicks) do _; cargar_trayectoria_cinematica() end
    on(menu_config.selection) do sel; configuracion[] = sel; actualizar_graficas_cinematica() end
    on(toggle_grados.active) do val; usar_grados[] = val; actualizar_graficas_cinematica() end
    on(btn_exportar.clicks) do _; exportar_datos_cinematica() end

    # Navegación entre pestañas
    on(btn_ir_trebol.clicks) do _
        trebol_panel.visible = true
        cinematica_panel.visible = false
    end

    on(btn_ir_cinematica.clicks) do _
        trebol_panel.visible = false
        cinematica_panel.visible = true
    end

    # ========== ANIMACIÓN DEL TRÉBOL ==========
    
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

    # Ajustar espaciado
    colgap!(main_layout, 30)
    rowgap!(control_panel, 12)
    colgap!(control_panel, 10)
    rowgap!(export_buttons_layout, 5)
    rowgap!(control_panel_cin, 10)
    colgap!(control_panel_cin, 10)
    rowgap!(graph_panel_cin, 20)

    return fig
end

# ========== EJECUCIÓN PRINCIPAL ==========

# Lanzar la aplicación integrada
println("Iniciando sistema integrado de generación de trayectoria y cinemática inversa...")
fig = crear_aplicacion_integrada()

# Mantener la ventana abierta si no es interactivo
if !isinteractive()
    wait(fig.scene)
end