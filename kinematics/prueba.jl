include("../robot/robot2R.jl")
using .Robot

include("fk.jl")
using .FK

robot = Robot.Robot2R(3.0, 4.0, 2.0, 3.0, 4.0)

x, y = FK.fk(robot, 0.3, 0.4)
println("Posición del efector: ", x, ", ", y)

