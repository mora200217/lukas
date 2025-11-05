include("robot/robot2R.jl")
include("kinetics/motor_torques_lagrange.jl")

include("kinematics/fk.jl")
include("kinematics/ik.jl")

# Import modules 
using .FK
using .IK

using .Robot
using .Torque

using Unitful
using PhysicalConstants.CODATA2018: g

# Parámetros 
L1 = 20.4u"cm"
L2 = 22.5u"cm"

M1 = 34u"g"
M2 = 34u"g"

g = 9.81u"m/s^2"
U = M1 * L1 * g

# Definición de propiedades del robot 
robot = Robot.Robot2R(L1, L2, M1, M2)

IK.ik(robot,0.25,0.10)
# Calculo de torque 
Torque.torque_calculator(0.3, 0.4)

