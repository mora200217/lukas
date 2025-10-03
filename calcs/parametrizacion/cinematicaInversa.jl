using LinearAlgebra
using Plots
using Plots.PlotMeasures

struct Robot2R
    L1::Float64  # Longitud eslabón 1
    L2::Float64  # Longitud eslabón 2
end

# Cinemática directa (la misma que tenías)
function cinematica_directa(robot::Robot2R, theta1, theta2)
    x = robot.L1 * cos(theta1) + robot.L2 * cos(theta1 + theta2)
    y = robot.L1 * sin(theta1) + robot.L2 * sin(theta1 + theta2)
    return [x, y]
end

# Cinemática inversa para UN punto - versión robusta
function cinematica_inversa_punto(robot::Robot2R, x, y)
    # Calcular distancia al origen
    d = sqrt(x^2 + y^2)
    
    # Verificar si el punto es alcanzable
    if d > (robot.L1 + robot.L2) || d < abs(robot.L1 - robot.L2)
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

# Función principal: validar trayectoria completa
function validar_trayectoria(robot::Robot2R, puntos_x, puntos_y; configuracion="arriba")
    n_puntos = length(puntos_x)
    
    # Arrays para almacenar resultados
    theta1_tray = zeros(n_puntos)
    theta2_tray = zeros(n_puntos)
    alcanzable = fill(false, n_puntos)
    
    # Validar cada punto de la trayectoria
    for i in 1:n_puntos
        x, y = puntos_x[i], puntos_y[i]
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

# Generar reporte de validación
function generar_reporte_trayectoria(robot::Robot2R, puntos_x, puntos_y, theta1, theta2, alcanzable)
    n_alcanzables = sum(alcanzable)
    n_total = length(alcanzable)
    porcentaje = round(n_alcanzables / n_total * 100, digits=2)
    
    println("=== REPORTE DE VALIDACIÓN DE TRAYECTORIA ===")
    println("Robot: L1 = $(robot.L1), L2 = $(robot.L2)")
    println("Puntos totales: $n_total")
    println("Puntos alcanzables: $n_alcanzables ($porcentaje%)")
    println("Puntos no alcanzables: $(n_total - n_alcanzables)")
    
    if n_alcanzables < n_total
        println("\n⚠️  ADVERTENCIA: La trayectoria contiene puntos no alcanzables")
        puntos_no_alcanzables = findall(.!alcanzable)
        println("Puntos problemáticos: $puntos_no_alcanzables")
    else
        println("\n✅ TRAYECTORIA COMPLETAMENTE ALCANZABLE")
    end
end

# Visualización completa
function visualizar_trayectoria_completa(robot::Robot2R, puntos_x, puntos_y, theta1, theta2, alcanzable)
    # Crear figura con subplots
    p1 = plot(title="Trayectoria Deseada vs Alcanzable", aspect_ratio=:equal)
    
    # Dibujar espacio de trabajo
    theta_circle = range(0, 2π, length=100)
    r_max = robot.L1 + robot.L2
    r_min = abs(robot.L1 - robot.L2)
    
    # Puntos alcanzables vs no alcanzables
    idx_alcanzable = findall(alcanzable)
    idx_no_alcanzable = findall(.!alcanzable)
    
    # Plot puntos alcanzables
    if !isempty(idx_alcanzable)
        scatter!(p1, puntos_x[idx_alcanzable], puntos_y[idx_alcanzable], 
                color=:green, label="Alcanzable", markersize=3)
    end
    
    # Plot puntos no alcanzables
    if !isempty(idx_no_alcanzable)
        scatter!(p1, puntos_x[idx_no_alcanzable], puntos_y[idx_no_alcanzable], 
                color=:red, label="No Alcanzable", markersize=3)
    end
    
    # Dibujar límites del espacio de trabajo
    plot!(p1, r_max * cos.(theta_circle), r_max * sin.(theta_circle), 
          color=:blue, linestyle=:dash, label="Límite máximo", linewidth=1)
    plot!(p1, r_min * cos.(theta_circle), r_min * sin.(theta_circle), 
          color=:red, linestyle=:dash, label="Límite mínimo", linewidth=1)
    
    # Gráfico de ángulos vs tiempo/puntos
    p2 = plot(title="Evolución de Ángulos", xlabel="Punto", ylabel="Ángulo [rad]")
    plot!(p2, theta1, label="θ₁", linewidth=2, color=:blue)
    plot!(p2, theta2, label="θ₂", linewidth=2, color=:red)
    
    # Marcar puntos no alcanzables
    if !isempty(idx_no_alcanzable)
        vline!(p2, idx_no_alcanzable, linestyle=:dash, color=:gray, alpha=0.5, label="No alcanzable")
    end
    
    # Layout final
    plot(p1, p2, layout=(2,1), size=(800, 800), bottom_margin=10mm)
end

# Función todo-en-uno para validar y visualizar
function analizar_trayectoria(robot::Robot2R, puntos_x, puntos_y; configuracion="arriba")
    println("🔍 Analizando trayectoria...")
    
    # Validar trayectoria
    theta1, theta2, alcanzable = validar_trayectoria(robot, puntos_x, puntos_y; configuracion=configuracion)
    
    # Generar reporte
    generar_reporte_trayectoria(robot, puntos_x, puntos_y, theta1, theta2, alcanzable)
    
    # Visualizar resultados
    visualizar_trayectoria_completa(robot, puntos_x, puntos_y, theta1, theta2, alcanzable)
    
    return theta1, theta2, alcanzable
end

# Ejemplo de uso con trayectoria de ejemplo (círculo)
function ejemplo_trayectoria_circular()
    # Crear robot
    robot = Robot2R(1.0, 0.8)
    
    # Generar trayectoria circular de ejemplo
    t = range(0, 2π, length=50)
    radio = 1.5  # Probar con diferentes radios para ver límites
    puntos_x = radio * cos.(t)
    puntos_y = radio * sin.(t)
    
    # Analizar trayectoria
    theta1, theta2, alcanzable = analizar_trayectoria(robot, puntos_x, puntos_y)
    
    return theta1, theta2, alcanzable
end

# Ejecutar ejemplo
 theta1, theta2, alcanzable = ejemplo_trayectoria_circular()