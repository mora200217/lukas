module splines_calculator

using LinearAlgebra

# ===============================================================
#      S P L I N E   C U B I C O   (natural + derivadas)
# ===============================================================

"""
    spline_coeffs(t, y)

Retorna las segundas derivadas `y2` del spline cúbico natural.
"""
function spline_coeffs(t::Vector{Float64}, y::Vector{Float64})
    n = length(t)
    y2 = zeros(n)
    u  = zeros(n-1)

    for i in 2:n-1
        sig = (t[i] - t[i-1]) / (t[i+1] - t[i-1])
        p = sig * y2[i-1] + 2.0
        y2[i] = (sig - 1.0) / p
        dy1 = (y[i+1] - y[i]) / (t[i+1] - t[i])
        dy0 = (y[i] - y[i-1]) / (t[i] - t[i-1])
        u[i] = (6.0*(dy1 - dy0) / (t[i+1] - t[i-1]) - sig * u[i-1]) / p
    end

    for k in (n-1):-1:1
        y2[k] = y2[k] * y2[k+1] + u[k]
    end

    return y2
end


# búsqueda binaria de intervalo
function find_interval(x, xi)
    lo = 1; hi = length(x)
    while hi - lo > 1
        m = (hi + lo) >>> 1
        if x[m] > xi; hi = m else; lo = m end
    end
    return lo
end


"""
    spline_eval(t, y, y2, ti)

Evalúa y(t) + ẏ(t) + ÿ(t)
"""
function spline_eval(t, y, y2, ti)
    if ti ≤ t[1]; return y[1], 0.0, 0.0 end
    if ti ≥ t[end]; return y[end], 0.0, 0.0 end

    i = find_interval(t, ti)
    h = t[i+1] - t[i]
    a = (t[i+1] - ti)/h
    b = (ti - t[i])/h

    # q(t)
    yi = a*y[i] + b*y[i+1] + ((a^3 - a)*y2[i] + (b^3 - b)*y2[i+1])*(h^2)/6

    # q̇(t)
    dydt = (y[i+1] - y[i])/h + ((-3a^2+1)*y2[i] + (3b^2-1)*y2[i+1])*h/6

    # q̈(t)
    dy2dt = a*y2[i] + b*y2[i+1]

    return yi, dydt, dy2dt
end


# ===============================================================
#           C I N E M A T I C A   I N V E R S A   2 R
# ===============================================================

"""
    IK_2R(x,y; L1, L2)

Cinemática inversa de un brazo planar 2R.
"""
function IK_2R(x,y; L1=0.12, L2=0.10)
    d2 = x^2 + y^2
    cosq2 = clamp((d2 - L1^2 - L2^2)/(2L1*L2), -1, 1)
    q2 = acos(cosq2)

    k1 = L1 + L2*cosq2
    k2 = L2*sin(q2)
    q1 = atan(y,x) - atan(k2,k1)

    return q1, q2
end


# ===============================================================
#     MAIN FUNCTION
# ===============================================================

"""
    plan_2R_from_xy(x, y, v;
                    L1=0.12, L2=0.10)

Recibe vectores de puntos `(x,y)` del end–effector y una **velocidad lineal v (m/s)**.

Devuelve funciones cerradas:

    q1(t), q̇1(t), q̈1(t), q2(t), q̇2(t), q̈2(t)
"""
function plan_2R_from_xy(x::Vector, y::Vector, v; L1=0.12, L2=0.10)
    N = length(x)

    # Longitud de arco → tiempo
    s = zeros(Float64, N)
    for i in 2:N
        s[i] = s[i-1] + hypot(x[i]-x[i-1], y[i]-y[i-1])
    end
    t = s ./ v

    # obtener q1(t), q2(t)
    q1 = similar(t); q2 = similar(t)
    for i in 1:N
        q1[i], q2[i] = IK_2R(x[i], y[i]; L1=L1, L2=L2)
    end

    # splines
    y2_q1 = spline_coeffs(t, q1)
    y2_q2 = spline_coeffs(t, q2)

    # funciones evaluables
    q1_t(ti) = spline_eval(t, q1, y2_q1, ti)
    q2_t(ti) = spline_eval(t, q2, y2_q2, ti)

    return q1_t, q2_t, t[end]   # también retorno duración total
end

end # module
