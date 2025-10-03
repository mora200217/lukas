using GLMakie
GLMakie.activate!()
using CSV
using DataFrames
using LinearAlgebra
using Printf
using Dates

# ---------- Estructura del Robot ----------
struct Robot2R
    L1::Float64      # Longitud eslabón 1
    L2::Float64      # Longitud eslabón 2
    base_offset_x::Float64  # Desplazamiento de la base en X
end

# ---------- Funciones de Cinemática Inversa ----------

"""
Aplica el offset de la base del robot a las coordenadas de la trayectoria
"""
function aplicar_offset_base(x, y, offset)
    return x .+ offset, y
end

"""
Cinemática inversa para UN punto - versión robusta
Retorna: (sol_arriba, sol_abajo, alcanzable)
"""
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

"""
Calcular cinemática inversa completa para toda la trayectoria
"""
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

"""
Leer CSV de trayectoria exportada
"""
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

"""
Exportar ángulos calculados a CSV
"""
"""
Exportar ángulos calculados a CSV
"""
"""
Exportar ángulos calculados a CSV
"""
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
    output_filename = "Data/angulos_cinematica_$(timestamp).csv"  # AGREGADO: Data/
    CSV.write(output_filename, df)
    
    println("✅ Ángulos exportados a: $output_filename")
    return output_filename
end

# ---------- Aplicación Principal GLMakie ----------

function crear_app_cinematica_inversa()
    
    # Configuración de tema visual
    set_theme!(theme_light())
    update_theme!(
        fontsize = 14,
        Label = (textcolor = :black,),
        Button = (buttoncolor = RGBf(0.2, 0.6, 0.8), labelcolor = :white),
        Toggle = (buttoncolor = RGBf(0.2, 0.6, 0.8), labelcolor = :white)
    )
    
    # Crear robot con parámetros especificados
    robot = Robot2R(16.0, 12.0, 15.0)  # L1=16cm, L2=12cm, offset=15cm
    
    # Observables
    theta1_data = Observable(Float64[])
    theta2_data = Observable(Float64[])
    alcanzable_data = Observable(Bool[])
    indices = Observable(Int[])
    
    puntos_x_original = Observable(Float64[])
    puntos_y_original = Observable(Float64[])
    
    configuracion = Observable("arriba")
    usar_grados = Observable(false)
    
    csv_path = Observable("Data/trajectory_2025-10-02_05-19-02.csv")  # Ruta por defecto
    datos_cargados = Observable(false)
    
    # Estadísticas
    n_total = Observable(0)
    n_alcanzables = Observable(0)
    porcentaje_alcanzable = Observable(0.0)
    theta1_min = Observable(0.0)
    theta1_max = Observable(0.0)
    theta2_min = Observable(0.0)
    theta2_max = Observable(0.0)
    
    # ---------- Crear Figura ----------
    fig = Figure(
        size = (1600, 1000),
        backgroundcolor = RGBf(0.98, 0.98, 0.98)
    )
    
    main_layout = GridLayout(fig[1, 1])
    
    # Panel izquierdo para gráficas (70%)
    graph_panel = main_layout[1, 1] = GridLayout()
    # Panel derecho para controles (30%)
    control_panel = main_layout[1, 2] = GridLayout()
    
    # ---------- Gráficas ----------
    
    # Gráfica 1: θ₁
    ax1 = Axis(
        graph_panel[1, 1],
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
        graph_panel[2, 1],
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
    
    # ---------- Panel de Control ----------
    
    # Título
    Label(control_panel[1, 1:2], "ANÁLISIS DE CINEMÁTICA INVERSA", 
          fontsize = 18, 
          font = :bold,
          color = RGBf(0.1, 0.1, 0.4))
    
    # Parámetros del robot
    Label(control_panel[2, 1:2], "PARÁMETROS DEL ROBOT", 
          fontsize = 16, 
          color = RGBf(0.2, 0.4, 0.6))
    
    Label(control_panel[3, 1], "L1:", tellwidth = false)
    Label(control_panel[3, 2], "16.0 cm", tellwidth = false, font = :bold)
    
    Label(control_panel[4, 1], "L2:", tellwidth = false)
    Label(control_panel[4, 2], "12.0 cm", tellwidth = false, font = :bold)
    
    Label(control_panel[5, 1], "Offset Base X:", tellwidth = false)
    Label(control_panel[5, 2], "+15.0 cm", tellwidth = false, font = :bold)
    
    # Archivo CSV
    Label(control_panel[6, 1:2], "ARCHIVO CSV", 
          fontsize = 16, 
          color = RGBf(0.2, 0.4, 0.6))
    
    Label(control_panel[7, 1], "Ruta:", tellwidth = false)
    csv_textbox = Textbox(control_panel[7, 2], 
                          placeholder = "Ruta del archivo CSV",
                          stored_string = csv_path[])
    
    btn_cargar = Button(control_panel[8, 1:2], label = "Cargar Trayectoria")
    
    # CORREGIDO: Inicializar con texto fijo en lugar de observable vacío
    status_label = Label(control_panel[9, 1:2], "Esperando cargar datos...", 
                        fontsize = 12, 
                        color = :gray,
                        tellwidth = false)
    
    # Configuración
    Label(control_panel[10, 1:2], "CONFIGURACIÓN", 
          fontsize = 16, 
          color = RGBf(0.2, 0.4, 0.6))
    
    # Toggle codo arriba/abajo
    menu_config = Menu(control_panel[11, 1:2], 
                       options = ["arriba", "abajo"],
                       default = "arriba")
    
    # Toggle radianes/grados
    toggle_grados = Toggle(control_panel[12, 1], active = usar_grados[])
    Label(control_panel[12, 2], "Mostrar en grados", tellwidth = false)
    
    # Estadísticas
    Label(control_panel[13, 1:2], "ESTADÍSTICAS", 
          fontsize = 16, 
          color = RGBf(0.2, 0.4, 0.6))
    
    # CORREGIDO: Inicializar con texto fijo
    stats_label = Label(control_panel[14, 1:2], "No hay datos cargados",
          fontsize = 13,
          tellwidth = false,
          justification = :left)
    
    # Rango de ángulos
    Label(control_panel[15, 1:2], "RANGO DE ÁNGULOS", 
          fontsize = 16, 
          color = RGBf(0.2, 0.4, 0.6))
    
    # CORREGIDO: Inicializar con texto fijo
    rango_label = Label(control_panel[16, 1:2], "---",
          fontsize = 13,
          tellwidth = false,
          justification = :left)
    
    # Botón exportar
    btn_exportar = Button(control_panel[17, 1:2], label = "Exportar Ángulos CSV")
    
    # ---------- Funciones de Actualización ----------
    
    # NUEVA FUNCIÓN: Actualizar labels de estadísticas
    function actualizar_stats_labels()
        if datos_cargados[]
            # Actualizar texto de estadísticas
            stats_text = """
            Puntos totales: $(n_total[])
            Alcanzables: $(n_alcanzables[]) $(@sprintf("%.2f", porcentaje_alcanzable[]))%
            No alcanzables: $(n_total[] - n_alcanzables[])
            """
            stats_label.text[] = stats_text
            
            # Actualizar texto de rangos
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
    
    function actualizar_graficas()
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
    
    # AGREGADO: Actualizar labels
    actualizar_stats_labels()
    
    # Ajustar límites de las gráficas
    autolimits!(ax1)
    autolimits!(ax2)
    
    # Mensaje de estado - CORREGIDO: usar RGBf en lugar de símbolos
    if n_alcanzables[] == n_total[]
        status_label.text[] = "✅ Todos los puntos son alcanzables"
        status_label.color[] = RGBf(0.0, 0.6, 0.0)  # Verde
    else
        status_label.text[] = "⚠️ Algunos puntos no son alcanzables"
        status_label.color[] = RGBf(1.0, 0.6, 0.0)  # Naranja
    end
end

function cargar_trayectoria()
    try
        path = csv_textbox.stored_string[]
        puntos_x, puntos_y = leer_csv_trayectoria(path)
        
        puntos_x_original[] = puntos_x
        puntos_y_original[] = puntos_y
        datos_cargados[] = true
        
        actualizar_graficas()
        
        println("✅ Trayectoria cargada: $(length(puntos_x)) puntos")
    catch e
        error_msg = sprint(showerror, e)
        status_label.text[] = "❌ Error al cargar archivo"
        status_label.color[] = RGBf(0.8, 0.0, 0.0)  # CORREGIDO: Rojo
        println("Error al cargar archivo: $error_msg")
    end
end

function exportar_datos()
    if !datos_cargados[]
        status_label.text[] = "❌ No hay datos para exportar"
        status_label.color[] = RGBf(0.8, 0.0, 0.0)  # CORREGIDO: Rojo
        return
    end
    
    try
        # Obtener ángulos en radianes
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
        status_label.color[] = RGBf(0.0, 0.6, 0.0)  # CORREGIDO: Verde
    catch e
        error_msg = sprint(showerror, e)
        status_label.text[] = "❌ Error al exportar"
        status_label.color[] = RGBf(0.8, 0.0, 0.0)  # CORREGIDO: Rojo
        println("Error al exportar: $error_msg")
    end
end
    
    function exportar_datos()
        if !datos_cargados[]
            status_label.text[] = "❌ No hay datos para exportar"
            status_label.color[] = :red
            return
        end
    
        try
        # Obtener ángulos en radianes
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
            status_label.color[] = :green
        catch e
            error_msg = sprint(showerror, e)  # CORREGIDO: Convertir error a string
            status_label.text[] = "❌ Error al exportar"
            status_label.color[] = :red
            println("Error al exportar: $error_msg")
        end
    end
    
    # ---------- Conectar Callbacks ----------
    
    on(btn_cargar.clicks) do _
        cargar_trayectoria()
    end
    
    on(menu_config.selection) do sel
        configuracion[] = sel
        actualizar_graficas()
    end
    
    on(toggle_grados.active) do val
        usar_grados[] = val
        actualizar_graficas()
    end
    
    on(btn_exportar.clicks) do _
        exportar_datos()
    end
    
    # Actualizar path del CSV desde textbox
    on(csv_textbox.stored_string) do new_path
        csv_path[] = new_path
    end
    
    # Ajustar espaciado
    colgap!(main_layout, 30)
    rowgap!(control_panel, 10)
    colgap!(control_panel, 10)
    rowgap!(graph_panel, 20)
    
    # Mostrar figura
    display(fig)
    
    return fig
end

# ---------- Lanzar Aplicación ----------
fig = crear_app_cinematica_inversa()

# Mantener la ventana abierta si no es interactivo
if !isinteractive()
    wait(fig.scene)
end