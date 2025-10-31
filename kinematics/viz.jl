module Viz

using GLMakie, Observables, Printf, Dates, CSV, DataFrames
using .IK
using .Trebol

export fig, ax_trebol, ax_theta1, ax_theta2,
       theta1_data, theta2_data, alcanzable_data, indices,
       actualizar_cinematica_en_vivo, update_trebol_labels

GLMakie.activate!()

# --- Observables de cinemática ---
theta1_data = Observable(Float64[])
theta2_data = Observable(Float64[])
alcanzable_data = Observable(Bool[])
indices = Observable(Int[])

configuracion = Observable("arriba")
unidad_angulos = Observable("radianes")

# Estadísticas
n_total = Observable(0)
n_alcanzables = Observable(0)
porcentaje_alcanzable = Observable(0.0)
theta1_min = Observable(0.0)
theta1_max = Observable(0.0)
theta2_min = Observable(0.0)
theta2_max = Observable(0.0)

# Función de actualización
function actualizar_cinematica_en_vivo()
    puntos_actuales = Trebol.xygrid[]
    puntos_x = [p[1] for p in puntos_actuales]
    puntos_y = [p[2] for p in puntos_actuales]
    
    theta1, theta2, alcanzable = IK.calcular_cinematica_completa(IK.robot, puntos_x, puntos_y;
        configuracion = configuracion[])
    
    if unidad_angulos[] == "grados"
        theta1 = rad2deg.(theta1)
        theta2 = rad2deg.(theta2)
    end
    
    theta1_data[] = theta1
    theta2_data[] = theta2
    alcanzable_data[] = alcanzable
    indices[] = collect(1:length(puntos_x))
    
    # Estadísticas
    theta1_rad, theta2_rad, _ = IK.calcular_cinematica_completa(IK.robot, puntos_x, puntos_y;
        configuracion = configuracion[])
    
    valid_theta1 = filter(!isnan, theta1_rad)
    valid_theta2 = filter(!isnan, theta2_rad)
    
    n_total[] = length(alcanzable)
    n_alcanzables[] = sum(alcanzable)
    porcentaje_alcanzable[] = n_total[] > 0 ? (n_alcanzables[] / n_total[]) * 100 : 0.0
    
    theta1_min[] = isempty(valid_theta1) ? 0.0 : minimum(valid_theta1)
    theta1_max[] = isempty(valid_theta1) ? 0.0 : maximum(valid_theta1)
    theta2_min[] = isempty(valid_theta2) ? 0.0 : minimum(valid_theta2)
    theta2_max[] = isempty(valid_theta2) ? 0.0 : maximum(valid_theta2)
end

# --- Creación de figura ---
fig = Figure(resolution = (1700, 1150))
ax_trebol = Axis(fig[1,1], title="Trébol", aspect = DataAspect())
ax_theta1 = Axis(fig[2,1], title="Theta1")
ax_theta2 = Axis(fig[3,1], title="Theta2")

end # module
