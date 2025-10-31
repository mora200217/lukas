using Dierckx

# datos de muestra
t_vals = [0.0, 0.1, 0.2, 0.3, 0.4] 
θ_vals = [0.0, 0.05, 0.2, 0.45, 0.8]

# crea el spline cúbico
spl = Spline1D(t_vals, θ_vals, k=3)  # k=3 para cúbico

# función para θ̈
θ_ddot = t -> derivative(spl, t, 2)  # segundo argumento = orden de derivada

println(θ_ddot(0.15))
