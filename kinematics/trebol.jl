module Trebol

export trebol_xy, trebol_points

"""
    trebol_xy(θ; n=5, a0=0.7, d0=0.007, rm=0.10, s=1.0, c=0.0, x0=0.0, y0=0.0)

Evalúa el trébol polar en ángulo θ y devuelve (x, y) desplazado por (x0, y0).
`rm` está en metros (ej: 0.10 = 10 cm).
"""
function trebol_xy(θ; n=5, a0=0.7, d0=0.007, rm=0.10, s=1.0, c=0.0, x0=0.0, y0=0.0)
    a = a0 / n
    d = d0 / n
    r = rm * s * (1 + a*cos(n*(θ - c - π)) - d*cos(2n*(θ - c - π)))

    x = r*cos(θ) + x0
    y = r*sin(θ) + y0
    return (x, y)
end


"""
    trebol_points(N; kwargs...)

Genera N puntos (x,y) igualmente espaciados en θ ∈ [0, 2π]
Puede recibir desplazamiento (x0, y0)

Retorna:
    xs, ys  :: Vectores con las coordenadas del trébol
"""
function trebol_points(N; 
        n=10,           # <-- TUS VALORES
        a0=0.5,         # <-- TUS VALORES
        d0=0.05,        # <-- TUS VALORES
        rm=0.09,        # <-- TUS VALORES
        s=1.0,          # <-- TUS VALORES
        c=0.0,          # <-- TUS VALORES
        x0=0.0,
        y0=0.0
    )
    θs = range(0, 2π; length=N)
    xs = Vector{Float64}(undef, N)
    ys = Vector{Float64}(undef, N)

    for (i, θ) in enumerate(θs)
        xs[i], ys[i] = trebol_xy(θ; n=n, a0=a0, d0=d0, rm=rm, s=s, c=c, x0=x0, y0=y0)
    end

    return xs, ys
end

end # module
