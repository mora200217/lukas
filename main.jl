# ===============================================
#  LIBRERÍAS
# ===============================================
using GLMakie

include("robot/robot2R.jl")
include("kinetics/torque_lagrange.jl")
include("kinematics/fk.jl")
include("kinematics/ik.jl")
include("kinematics/closed_ik.jl")
include("kinematics/trebol.jl")
include("kinematics/splines_calculator.jl")

using .Robot
using .Torque
using .FK
using .IK
using .SymIK2R
using .Trebol
using .splines_calculator


# ===============================================
#  PARAMETROS DEL ROBOT (¡EN S.I.!)
# ===============================================
L1 = 0.25   # 20.4 cm → m
L2 = 0.15   # 22.5 cm → m

M1 = 0.300   # 34 g → kg
M2 = 0.030   # 34 g → kg

grav = 9.81  # m/s²

#robot = Robot.Robot2R(L1, L2, M1, M2, 1)


# ===============================================
#  GENERAR TRÉBOL EN (X,Y)
# ===============================================
N_points = 2000
offset_x = 0.13
offset_y = 0; 
vel = 0.03  # m/s

x, y = Trebol.trebol_points(N_points; rm=0.10, x0=offset_x, y0=offset_y)


# ===============================================
#  PLANIFICACIÓN DE TRAYECTORIA 2R (SPLINES)
# ===============================================
q1, q2, tf = splines_calculator.plan_2R_from_xy(x, y, vel)

println("\n--- Evaluación de trayectorias articulares y torques ---")



# ===============================================
# GRAFICAR EL TRÉBOL
# ===============================================
fig = Figure(resolution = (800, 600))

ax = Axis(fig[1, 1],
    title = "Trayectoria del efector - Trébol",
    xlabel = "X [m]",
    ylabel = "Y [m]",
    aspect = DataAspect()  # Mantener proporción
)

# Dibujar línea de trayectoria
lines!(ax, x, y, color = :blue, linewidth = 2, label = "Trayectoria EF")

# Opcional: marcar algunos puntos discretos
scatter!(ax, x[1:100:end], y[1:100:end], color = :red, markersize = 3, label = "Puntos")

axislegend(ax, position = :rt)
display(fig)



# ==========================================================
# 1. Obtener expresiones simbólicas (Euler-Lagrange)
# ==========================================================
Q1_sym, Q2_sym = Torque.Euler_Lagrange_2R(
    M1, M2,
    L1, L2,
    0.02, 0.02,   # Inercias de cada eslabón [kg·m²] (puedes ajustarlo)
    grav
)


# ==========================================================
# 2. Construcción del diccionario de trayectoria
# ==========================================================
N = length(x)
tvec = range(0, tf, length=N)

θ1_vec  = zeros(N)
θ2_vec  = zeros(N)
dθ1_vec = zeros(N)
dθ2_vec = zeros(N)
ddθ1_vec = zeros(N)
ddθ2_vec = zeros(N)

for i in 1:N
    # Obtener q, dq y ddq del spline
    θ1_vec[i], dθ1_vec[i], ddθ1_vec[i] = q1(tvec[i])
    θ2_vec[i], dθ2_vec[i], ddθ2_vec[i] = q2(tvec[i])
end


trayectoria = Dict(
    "t"        => collect(tvec),
    "theta1"   => θ1_vec,
    "theta2"   => θ2_vec,
    "dtheta1"  => dθ1_vec,
    "dtheta2"  => dθ2_vec,
    "ddtheta1" => ddθ1_vec,
    "ddtheta2" => ddθ2_vec
)


# ==========================================================
# 3. Calcular el perfil de torques
# ==========================================================
torques = Torque.torque_profile_from_Q(
    Q1_sym, Q2_sym,
    trayectoria;
    τ_max = 5.0,     # Límite visual para gráficos [N·m]
    export_csv = false,
    debug = true
)

println("\n✅ TORQUES CALCULADOS!")
println("τ1 size: ", length(torques.tau1))
println("τ2 size: ", length(torques.tau2))

using Statistics

τ1_rms = sqrt(mean(torques.tau1 .^ 2))
τ2_rms = sqrt(mean(torques.tau2 .^ 2))

println("Torque RMS articulación 1: $(round(τ1_rms, digits=4)) mN·m")
println("Torque RMS articulación 2: $(round(τ2_rms, digits=4)) mN·m")

println("✓ Torque RMS: τ1 = $(round(τ1_rms,digits=4)) N·m, τ2 = $(round(τ2_rms,digits=4)) N·m")


using GLMakie
fig = Figure()
ax = Axis(fig[1,1], title="Aceleraciones articulares", xlabel="t", ylabel="ddθ [rad/s²]")
lines!(ax, tvec, ddθ1_vec, label="ddθ1")
lines!(ax, tvec, ddθ2_vec, label="ddθ2")
axislegend(ax)
display(fig)