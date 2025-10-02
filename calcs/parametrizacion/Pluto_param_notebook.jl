### A Pluto.jl notebook ###
# v0.19.32

using Markdown
using InteractiveUtils

# This Pluto notebook uses @bind for interactivity. When running this notebook outside of Pluto, the following 'mock version' of @bind gives bound variables a default value (instead of an error).
macro bind(def, element)
    quote
        local iv = try Base.loaded_modules[Base.PkgId(Base.UUID("6e696c72-6542-2067-7265-42206c756150"), "AbstractPlutoDingetjes")].Bonds.initial_value catch; b -> missing; end
        local el = $(esc(element))
        global $(esc(def)) = Core.applicable(Base.get, el) ? Base.get(el) : iv(el)
        el
    end
end

# ╔═╡ 1a2b3c4d-5e6f-7g8h-9i0j-1k2l3m4n5o6p
begin
    import Pkg
    Pkg.activate(mktempdir())
    Pkg.add([
        Pkg.PackageSpec(name="Plots"),
        Pkg.PackageSpec(name="PlutoUI"),
        Pkg.PackageSpec(name="Printf")
    ])
    using Plots
    using PlutoUI
    using Printf
end

# ╔═╡ 2b3c4d5e-6f7g-8h9i-0j1k-2l3m4n5o6p7q
md"""
# 🌸 Trébol Paramétrico con Radio Máximo Controlado

Este notebook implementa una parametrización de trébol estilizado donde `b(n)` se ajusta automáticamente para mantener un radio máximo constante.

## Ecuación paramétrica:
$$r(\theta) = b(n) \cdot \left[(1 + a(n)\cos(n(\theta - c - \pi))) - d(n)\cos(2n(\theta - c))\right]$$

Donde `b(n)` se calcula automáticamente para mantener $r_{max} = r_m$
"""

# ╔═╡ 3c4d5e6f-7g8h-9i0j-1k2l-3m4n5o6p7q8r
md"""
## 🎛️ Controles Interactivos

Ajusta los parámetros para ver cómo cambia la forma del trébol:
"""

# ╔═╡ 4d5e6f7g-8h9i-0j1k-2l3m-4n5o6p7q8r9s
begin
    md"""
    **Radio Máximo ($r_m$):** $(@bind rm PlutoUI.Slider(5:0.5:15, default=10, show_value=true))
    
    **Parámetro a(n):** $(@bind a PlutoUI.Slider(0:0.01:1, default=0.6, show_value=true))
    
    **Parámetro d(n):** $(@bind d PlutoUI.Slider(0:0.01:0.5, default=0.198, show_value=true))
    
    **Número de pétalos (n):** $(@bind n PlutoUI.Slider(3:1:8, default=5, show_value=true))
    
    **Rotación (c):** $(@bind c PlutoUI.Slider(0:0.1:2π, default=0, show_value=true))
    """
end

# ╔═╡ 5e6f7g8h-9i0j-1k2l-3m4n-5o6p7q8r9s0t
md"""
## 📊 Funciones de Cálculo
"""

# ╔═╡ 6f7g8h9i-0j1k-2l3m-4n5o-6p7q8r9s0t1u
"""
    find_actual_max_radius(a, b, n, c, d; steps=1000)

Encuentra el radio máximo real de la curva evaluándola numéricamente.
"""
function find_actual_max_radius(a, b, n, c, d; steps=1000)
    max_r = 0.0
    for i in 0:steps
        θ = (i / steps) * 2π
        r = b * ((1 + a * cos(n * (θ - c - π))) - d * cos(n * 2 * (θ - c)))
        max_r = max(max_r, abs(r))
    end
    return max_r
end

# ╔═╡ 7g8h9i0j-1k2l-3m4n-5o6p-7q8r9s0t1u2v
"""
    calculate_b(rm, a, n, c, d; tolerance=0.001, max_iterations=50)

Calcula el valor de b(n) usando búsqueda binaria para mantener el radio máximo rm.
"""
function calculate_b(rm, a, n, c, d; tolerance=0.001, max_iterations=50)
    b_low = 0.0
    b_high = 20.0
    b = rm / (1 + a + d)  # Estimación inicial
    
    for iter in 1:max_iterations
        b = (b_low + b_high) / 2
        actual_max = find_actual_max_radius(a, b, n, c, d)
        
        if abs(actual_max - rm) < tolerance
            break
        end
        
        if actual_max < rm
            b_low = b
        else
            b_high = b
        end
    end
    
    return b
end

# ╔═╡ 8h9i0j1k-2l3m-4n5o-6p7q-8r9s0t1u2v3w
md"""
## 🌺 Visualización del Trébol
"""

# ╔═╡ 9i0j1k2l-3m4n-5o6p-7q8r-9s0t1u2v3w4x
begin
    # Calcular b(n) para mantener el radio máximo
    b_calc = calculate_b(rm, a, n, c, d)
    
    # Verificar el radio máximo real
    actual_max = find_actual_max_radius(a, b_calc, n, c, d)
    error_pct = (actual_max - rm) / rm * 100
    
    # Generar la curva
    θ_range = range(0, 2π, length=1000)
    r_values = @. b_calc * ((1 + a * cos(n * (θ_range - c - π))) - 
                            d * cos(n * 2 * (θ_range - c)))
    
    # Convertir a coordenadas cartesianas
    x_values = @. r_values * cos(θ_range)
    y_values = @. r_values * sin(θ_range)
    
    # Crear el gráfico
    plot(x_values, y_values,
        label="Trébol (n=$n)",
        linewidth=2.5,
        color=RGB(0.9, 0.2, 0.3),
        aspect_ratio=:equal,
        xlims=(-rm*1.3, rm*1.3),
        ylims=(-rm*1.3, rm*1.3),
        grid=true,
        gridstyle=:dot,
        gridalpha=0.3,
        framestyle=:box,
        title="Trébol Paramétrico con Radio Máximo Controlado",
        titlefontsize=14,
        background_color=:white,
        size=(600, 600)
    )
    
    # Agregar círculos de referencia
    for r_ref in 2:2:floor(Int, rm)
        circle_θ = range(0, 2π, length=100)
        circle_x = @. r_ref * cos(circle_θ)
        circle_y = @. r_ref * sin(circle_θ)
        plot!(circle_x, circle_y,
            label="",
            color=:gray,
            alpha=0.2,
            linewidth=0.5
        )
    end
    
    # Agregar círculo de radio máximo
    circle_θ = range(0, 2π, length=200)
    circle_x = @. rm * cos(circle_θ)
    circle_y = @. rm * sin(circle_θ)
    plot!(circle_x, circle_y,
        label="Radio máximo (rm=$rm)",
        color=RGB(0.2, 0.4, 0.9),
        linestyle=:dash,
        linewidth=2,
        alpha=0.8
    )
    
    # Agregar ejes
    plot!([-rm*1.3, rm*1.3], [0, 0],
        color=:gray,
        alpha=0.3,
        label="",
        linewidth=1
    )
    plot!([0, 0], [-rm*1.3, rm*1.3],
        color=:gray,
        alpha=0.3,
        label="",
        linewidth=1
    )
    
    # Marcar el origen
    scatter!([0], [0],
        color=:black,
        markersize=3,
        label=""
    )
end

# ╔═╡ 0j1k2l3m-4n5o-6p7q-8r9s-0t1u2v3w4x5y
md"""
## 📈 Información Calculada
"""

# ╔═╡ 1k2l3m4n-5o6p-7q8r-9s0t-1u2v3w4x5y6z
begin
    info_html = """
    <div style='background-color: #f0f8ff; padding: 15px; border-radius: 10px; 
                border-left: 4px solid #4169e1; font-family: system-ui;'>
        <h3 style='margin-top: 0; color: #2c3e50;'>📐 Parámetros y Resultados</h3>
        
        <div style='display: grid; grid-template-columns: 1fr 1fr; gap: 20px;'>
            <div>
                <h4 style='color: #34495e; margin-bottom: 10px;'>Parámetros de Entrada:</h4>
                <ul style='list-style-type: none; padding: 0;'>
                    <li>📏 <b>Radio máximo (rm):</b> $(rm)</li>
                    <li>📊 <b>a(n):</b> $(@sprintf("%.3f", a))</li>
                    <li>📈 <b>d(n):</b> $(@sprintf("%.3f", d))</li>
                    <li>🌸 <b>n (pétalos):</b> $(n)</li>
                    <li>🔄 <b>c (rotación):</b> $(@sprintf("%.2f", c)) rad</li>
                </ul>
            </div>
            
            <div>
                <h4 style='color: #34495e; margin-bottom: 10px;'>Resultados Calculados:</h4>
                <ul style='list-style-type: none; padding: 0;'>
                    <li>🎯 <b>b(n) calculado:</b> 
                        <span style='color: #e74c3c; font-weight: bold;'>
                            $(@sprintf("%.4f", b_calc))
                        </span>
                    </li>
                    <li>📍 <b>Radio máximo real:</b> 
                        <span style='color: #27ae60; font-weight: bold;'>
                            $(@sprintf("%.3f", actual_max))
                        </span>
                    </li>
                    <li>⚠️ <b>Error:</b> 
                        <span style='color: $(abs(error_pct) < 0.1 ? "#27ae60" : "#e67e22"); 
                                     font-weight: bold;'>
                            $(@sprintf("%.3f", error_pct))%
                        </span>
                    </li>
                </ul>
            </div>
        </div>
        
        <div style='margin-top: 15px; padding-top: 15px; border-top: 1px solid #bdc3c7;'>
            <p style='color: #7f8c8d; margin: 0; font-size: 0.9em;'>
                💡 <b>Nota:</b> El algoritmo ajusta automáticamente b(n) mediante búsqueda binaria 
                para mantener el radio máximo deseado. El error típico es < 0.1%.
            </p>
        </div>
    </div>
    """
    
    HTML(info_html)
end

# ╔═╡ 2l3m4n5o-6p7q-8r9s-0t1u-2v3w4x5y6z7a
md"""
## 🎨 Galería de Formas

Exploremos diferentes configuraciones del trébol:
"""

# ╔═╡ 3m4n5o6p-7q8r-9s0t-1u2v-3w4x5y6z7a8b
begin
    plots_gallery = []
    
    # Configuraciones predefinidas
    configs = [
        (n=3, a=0.4, d=0.15, title="Trébol de 3 hojas"),
        (n=4, a=0.5, d=0.2, title="Cruz estilizada"),
        (n=5, a=0.6, d=0.198, title="Estrella de 5 puntas"),
        (n=6, a=0.7, d=0.25, title="Hexagonal")
    ]
    
    for config in configs
        b_config = calculate_b(10, config.a, config.n, 0, config.d)
        θ_config = range(0, 2π, length=500)
        r_config = @. b_config * ((1 + config.a * cos(config.n * θ_config)) - 
                                  config.d * cos(config.n * 2 * θ_config))
        x_config = @. r_config * cos(θ_config)
        y_config = @. r_config * sin(θ_config)
        
        p = plot(x_config, y_config,
            title=config.title,
            label="",
            linewidth=2,
            color=RGB(rand(), rand()*0.7, rand()*0.9),
            aspect_ratio=:equal,
            xlims=(-12, 12),
            ylims=(-12, 12),
            grid=false,
            framestyle=:box,
            titlefontsize=10
        )
        push!(plots_gallery, p)
    end
    
    plot(plots_gallery..., layout=(2,2), size=(600, 600))
end

# ╔═╡ 4n5o6p7q-8r9s-0t1u-2v3w-4x5y6z7a8b9c
md"""
## 💻 Código para Exportar

Si deseas usar esta función fuera de Pluto:

```julia
function plot_trebol(rm, a, d, n, c)
    b = calculate_b(rm, a, n, c, d)
    θ = range(0, 2π, length=1000)
    r = @. b * ((1 + a * cos(n * (θ - c - π))) - d * cos(n * 2 * (θ - c)))
    x = @. r * cos(θ)
    y = @. r * sin(θ)
    return x, y, b
end
```
"""

# ╔═╡ 5o6p7q8r-9s0t-1u2v-3w4x-5y6z7a8b9c0d
md"""
---
*Desarrollado con Pluto.jl - Notebook interactivo para Julia* 🎉
"""

# ╔═╡ Cell order:
# ╟─1a2b3c4d-5e6f-7g8h-9i0j-1k2l3m4n5o6p
# ╟─2b3c4d5e-6f7g-8h9i-0j1k-2l3m4n5o6p7q
# ╟─3c4d5e6f-7g8h-9i0j-1k2l-3m4n5o6p7q8r
# ╟─4d5e6f7g-8h9i-0j1k-2l3m-4n5o6p7q8r9s
# ╟─5e6f7g8h-9i0j-1k2l-3m4n-5o6p7q8r9s0t
# ╠═6f7g8h9i-0j1k-2l3m-4n5o-6p7q8r9s0t1u
# ╠═7g8h9i0j-1k2l-3m4n-5o6p-7q8r9s0t1u2v
# ╟─8h9i0j1k-2l3m-4n5o-6p7q-8r9s0t1u2v3w
# ╠═9i0j1k2l-3m4n-5o6p-7q8r-9s0t1u2v3w4x
# ╟─0j1k2l3m-4n5o-6p7q-8r9s-0t1u2v3w4x5y
# ╟─1k2l3m4n-5o6p-7q8r-9s0t-1u2v3w4x5y6z
# ╟─2l3m4n5o-6p7q-8r9s-0t1u-2v3w4x5y6z7a
# ╠═3m4n5o6p-7q8r-9s0t-1u2v-3w4x5y6z7a8b
# ╟─4n5o6p7q-8r9s-0t1u-2v3w-4x5y6z7a8b9c
# ╟─5o6p7q8r-9s0t-1u2v-3w4x-5y6z7a8b9c0d