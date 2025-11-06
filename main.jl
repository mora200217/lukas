include("robot/robot2R.jl")
include("kinetics/torque_lagrange.jl")

include("kinematics/fk.jl")
include("kinematics/ik.jl")

include("kinematics/closed_ik.jl")

# Import modules 
using .FK
using .IK

using .Robot
using .Torque

using .SymIK2R

using Unitful
using PhysicalConstants.CODATA2018: g

using Symbolics

@variables t 
@variables t1(t) t2(t)

SymIK2R.inverse_kinematics_2R(cos(t1(t)),cos(t1(t)), 2, 3)

# Parámetros 
L1 = 20.4u"cm"
L2 = 22.5u"cm"

M1 = 34u"g"
M2 = 34u"g"

g = 9.81u"m/s^2"
U = M1 * L1 * g


# Torque example 
τ_1, τ_2 =  Torque.Euler_Lagrange_2R(2, 3, 2, 3, 2, 3, 3)

τ_1




# Definición de propiedades del robot 
robot = Robot.Robot2R(L1, L2, M1, M2, 1)


IK.ik(robot,0.25,0.10)

# Calculo de torque 
Torque.torque_calculator(0.3, 0.4)

