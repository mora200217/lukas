module Trebol

using Observables, LinearAlgebra
using .IK: Robot2R

export n, a0, d0, rm, s, c, n_puntos_obs, θg, rgrid, xygrid, 
       θpartial, rpartial, xypartial, rcur, xycurrent,
       trebol_function

# Observables del trébol
n  = Observable(5)
a0 = Observable(0.7)
d0 = Observable(0.007)
rm = Observable(10.0)
s  = Observable(1.0)
c  = Observable(0.0)
n_puntos_obs = Observable(1000)

# Función del trébol
function trebol_function(θ, n_val, a_val, d_val, c_val)
    return 1 .+ a_val .* cos.(n_val .* (θ .- c_val .- pi)) .- d_val .* cos.(2n_val .* (θ .- c_val .- pi))
end 

# θg depende del número de puntos
θg = lift(n_puntos_obs) do n_puntos
    range(0, 2π; length=n_puntos)
end

a = lift(a0, n) do a0_val, n_val
    a0_val / n_val
end

d = lift(d0, n) do d0_val, n_val
    d0_val / n_val
end

rgrid = lift(n, a, d, rm, s, c, θg) do n_val, a_val, d_val, rm_val, s_val, c_val, θg_val
    f_vals = trebol_function(θg_val, n_val, a_val, d_val, c_val)
    f_max = maximum(f_vals)
    if f_max ≈ 0.0
        fill(s_val * rm_val, length(θg_val))
    else
        s_val * rm_val .* (f_vals ./ f_max)
    end
end

xygrid = lift(rgrid, θg) do rg, θg_val
    [Point2f(r * cos(θ), r * sin(θ)) for (r, θ) in zip(rg, θg_val)]
end

θpartial = lift(n, a, d, rm, s, c, θg, n_puntos_obs) do n_val, a_val, d_val, rm_val, s_val, c_val, θg_val, _
    θg_val
end

rpartial = lift(n, a, d, rm, s, c, θpartial, θg) do n_val, a_val, d_val, rm_val, s_val, c_val, θv, θg_full
    f_vals = trebol_function(θv, n_val, a_val, d_val, c_val)
    f_max = maximum(trebol_function(θg_full, n_val, a_val, d_val, c_val))
    if f_max ≈ 0.0
        fill(s_val * rm_val, length(θv))
    else
        s_val * rm_val .* (f_vals ./ f_max)
    end
end

xypartial = lift(rpartial, θpartial) do rg, θv
    [Point2f(r * cos(θ), r * sin(θ)) for (r, θ) in zip(rg, θv)]
end

rcur = lift(n, a, d, rm, s, c, θg) do n_val, a_val, d_val, rm_val, s_val, c_val, θt_val
    f_val = trebol_function([θt_val], n_val, a_val, d_val, c_val)[1]
    f_vals = trebol_function(θg_val, n_val, a_val, d_val, c_val)
    f_max = maximum(f_vals)
    if f_max ≈ 0.0
        s_val * rm_val
    else
        s_val * rm_val * f_val / f_max
    end
end

xycurrent = lift(θg, rcur) do θt_val, rcur_val
    [Point2f(rcur_val * cos(θt_val), rcur_val * sin(θt_val))]
end

end # module
