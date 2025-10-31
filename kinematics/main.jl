module MainModule

using LinearAlgebra

export Robot2R, aplicar_offset_base, cinematica_inversa_punto, calcular_cinematica_completa

struct Robot2R
    L1::Float64
    L2::Float64
    base_offset_x::Float64
end

# Aplicar offset a los puntos
function aplicar_offset_base(x, y, offset)
    return x .+ offset, y
end

# Cinemática inversa de un punto
function cinematica_inversa_punto(robot::Robot2R, x, y)
    d = sqrt(x^2 + y^2)
    d_max = robot.L1 + robot.L2
    d_min = abs(robot.L1 - robot.L2)
    
    if d > d_max || d < d_min
        return nothing, nothing, false
    end
    
    c2 = clamp((x^2 + y^2 - robot.L1^2 - robot.L2^2) / (2 * robot.L1 * robot.L2), -1.0, 1.0)
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

# Cinemática inversa para toda la trayectoria
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

end # module
