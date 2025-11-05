# # Calculo de torques por planteamiento Lagrangiano 
# include("../kinematics/main.jl")
# using .MainModule  # El punto indica módulo local

# robot = Robot2R(16.0, 12.0, 15.0)

# puntos_x = [10.0, 12.0, 14.0]
# puntos_y = [5.0, 7.0, 9.0]

# theta1, theta2, alcanzable = calcular_cinematica_completa(robot, puntos_x, puntos_y)



# using Unitful, CairoMakie 

# # Parámetros =================================
# M1 = 12  # [kg] 
# M2 = 11

# L1 = 1 # m 
# L2 = 1 # m

# I1 = 12 
# I2 = 34

# Definición de Lagrangiano ================================= 
module Torque
    using Symbolics
    # Definicion de T y V 
    @variables t
    @variables m1 m2 l1 l2 I1 I2 θ1(t) θ2(t)

    D = Differential(t)
    
    T = m1/2 * l1 * I1 * D(θ1) + D(θ1)

    export torque_calculator    
    function torque_calculator(θ_1,θ_2)
        # Definicion de theta1 y theta2 

        


        return 1

    end

end 









