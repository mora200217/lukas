#############################################
#  trebol_2R_spline.jl  (Autocontenido mejorado)
#############################################

using LinearAlgebra

# ---------------------------
#  Parametrización del trébol
# ---------------------------
function trebol_r(θ, n, a, d, c)
    1 + a*cos(n*(θ - c - π)) - d*cos(2n*(θ - c - π))    # Parametrización trébol estilizado
end

function trebol_xy(θ)
    # Parámetros del trébol

    n  = 5              # Número de hojas del trébol
    a  = 0.7/n          # 
    d  = 0.007/n        # Ondulación secundaria (suavizado fino)
    c  = 0.0            # Desfase angular (rotación del trébol)
    rm = 0.10           # Radio base o escala física = 10 cm
    s  = 1.0            # Factor de escala dinámico

    r = rm * s * trebol_r(θ, n, a, d, c)  # rm convierte la forma adimensional en centímetros.
    return (r*cos(θ), r*sin(θ))           # Regresa las coordenadas en (x,y) 
end

# ---------------------------
#  Longitud de arco + tiempo
# ---------------------------
function curva_parametrizada(N=2000, v=0.05)
    # v = velocidad constante en m/s  (0.01 a 0.1)

    θs = collect(range(0, 2π, length=N))    # Conunto de
    pts = [trebol_xy(θ) for θ in θs]        #

    # Longitud de arco acumulada
    s = zeros(Float64, N)
    for i in 2:N
        s[i] = s[i-1] + hypot(pts[i][1] - pts[i-1][1], pts[i][2] - pts[i-1][2])
    end

    # tiempo t(s) = s / v
    t = s ./ v

    return θs, pts, s, t
end

# ---------------------------
#  Cinemática inversa 2R
# ---------------------------
function IK_2R(x, y; L1=0.12, L2=0.10)
    d2 = x^2 + y^2
    cosq2 = clamp((d2 - L1^2 - L2^2) / (2*L1*L2), -1.0, 1.0)
    q2 = acos(cosq2)

    k1 = L1 + L2*cosq2
    k2 = L2*sin(q2)

    q1 = atan(y, x) - atan(k2, k1)
    return q1, q2
end

# ---------------------------
#  SPLINE CÚBICO NATURAL (autocontenido)
#  - spline_coeffs(x,y) -> devuelve vector de segundas derivadas y2
#  - spline_eval(x, y, y2, xi) -> evalúa spline en xi
# ---------------------------
function spline_coeffs(x::Vector{Float64}, y::Vector{Float64})
    n = length(x)
    @assert n == length(y) "x,y deben tener igual longitud"
    y2 = zeros(n)
    u  = zeros(n-1)

    # condiciones de frontera: natural spline (segunda derivada cero)
    y2[1] = 0.0
    u[1]  = 0.0

    for i in 2:n-1
        sig = (x[i] - x[i-1]) / (x[i+1] - x[i-1])
        p = sig * y2[i-1] + 2.0
        y2[i] = (sig - 1.0) / p
        dy1 = (y[i+1] - y[i]) / (x[i+1] - x[i])
        dy0 = (y[i] - y[i-1]) / (x[i] - x[i-1])
        u[i] = (6.0*(dy1 - dy0) / (x[i+1] - x[i-1]) - sig * u[i-1]) / p
    end

    y2[end] = 0.0

    for k in (n-1):-1:1
        y2[k] = y2[k] * y2[k+1] + u[k]
    end

    return y2
end

# búsqueda binaria del intervalo
function find_interval(x::Vector{Float64}, xi::Float64)
    lo = 1
    hi = length(x)
    if xi <= x[1]
        return 1
    elseif xi >= x[end]
        return length(x)-1
    end
    # binsearch clásico
    while hi - lo > 1
        mid = (hi + lo) >>> 1
        if x[mid] > xi
            hi = mid
        else
            lo = mid
        end
    end
    return lo
end

function spline_eval(x::Vector{Float64}, y::Vector{Float64}, y2::Vector{Float64}, xi::Float64)
    n = length(x)
    # clamp para evitar extrapolación salvaje
    if xi <= x[1]
        return y[1]
    elseif xi >= x[n]
        return y[n]
    end
    klo = find_interval(x, xi)
    khi = klo + 1
    h = x[khi] - x[klo]
    @assert h > 0 "Nodos no estrictamente crecientes"
    a = (x[khi] - xi) / h
    b = (xi - x[klo]) / h
    yi = a*y[klo] + b*y[khi] + ((a^3 - a)*y2[klo] + (b^3 - b)*y2[khi])*(h^2)/6.0
    return yi
end

# helper que crea función evaluadora
function make_cubic_spline(x::Vector{Float64}, y::Vector{Float64})
    y2 = spline_coeffs(x, y)
    return (t -> spline_eval(x, y, y2, t))
end

# ---------------------------
#  Construcción de q1(t), q2(t)
# ---------------------------
θs, pts, s, t = curva_parametrizada(2000, 0.06)  # 6 cm/s

q1_arr = zeros(Float64, length(t))
q2_arr = zeros(Float64, length(t))

for i in eachindex(t)
    x, y = pts[i]
    q1_arr[i], q2_arr[i] = IK_2R(x, y)
end

# corregir saltos de rama en q1 y q2: unwrap angular discontinuities
function unwrap_phase!(arr::Vector{Float64})
    for i in 2:length(arr)
        delta = arr[i] - arr[i-1]
        if delta > π
            arr[i:end] .-= 2π
        elseif delta < -π
            arr[i:end] .+= 2π
        end
    end
    return arr
end

unwrap_phase!(q1_arr)
unwrap_phase!(q2_arr)

# construir splines
q1_spline = make_cubic_spline(t, q1_arr)
q2_spline = make_cubic_spline(t, q2_arr)

# funciones públicas (clamp dentro del rango)
tmin = t[1]; tmax = t[end]
q1(ti) = ti <= tmin ? q1_arr[1] : (ti >= tmax ? q1_arr[end] : q1_spline(ti))
q2(ti) = ti <= tmin ? q2_arr[1] : (ti >= tmax ? q2_arr[end] : q2_spline(ti))

# -----------
#  DEMO / IMPRIME
# -----------
if  true 
    println("Ejemplo q1(t), q2(t) con spline cúbico natural:")
    for ti in range(0, tmax, length=7)
        println("t=$(round(ti, digits=3))  ->  q1=$(round(q1(ti),digits=4)), q2=$(round(q2(ti),digits=4))")
    end
end



