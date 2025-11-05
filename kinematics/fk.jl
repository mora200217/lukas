# Forward Kinematics
module FK

    export fk
    using ..Robot

    function fk(robot::Robot.Robot2R, θ1, θ2)
        # 2R - Robot Forward kinematics  =========
        # No se tiene en cuenta la orientación del efecto 
        # debido a que al ser un movimiento planar, no se modifica 
        # su orientacion inicial. Al tener un marcador apoyado en el 
        # tablero basta unicamente con la posición

        x = robot.L1 * cos(θ1) + robot.L2 * cos(θ1 + θ2); 
        y = robot.L1 * sin(θ1) + robot.L2* sin(θ1 + θ2); 
        
        return x, y
    end 
end