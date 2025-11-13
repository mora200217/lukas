

module Torque
    export Euler_Lagrange_2R 

    using Symbolics
    using Latexify
    using DataFrames
    using CairoMakie
    using LinearAlgebra
    using Statistics
    using CSV
    using Dates




        function Euler_Lagrange_2R(m1R, m2R, L1R, L2R, W1R, W2R, gR)
        # ============================ ============================ ============================ ============================
        # DEFINICIÓN DE VARIABLES DEL SISTEMA Y OPERACIONES
        # ============================ ============================ ============================ ============================ 
        @variables  t
        @variables  θ1(t) θ2(t) m1 m2 L1 L2 W1 W2 g

        ∂θ1 =  Differential(θ1);   ∂θ2 = Differential(θ2)       # Derivada con respecto a θ1 y con respecto a θ2      

        ∂t =  Differential(t)        # Derivada con respecto al tiempo

        dθ1 = ∂t(θ1);  dθ2 = ∂t(θ2)  #  ω1 y  ω2

        ∂2_θ1 =  Differential(dθ1);  ∂2_θ2 =  Differential(dθ2)  # Derivada con respecto a ω1 y con respecto a ω2

        # ============================ ============================ ============================ ============================
        # DEFINICIÓN DE MATRICES PARA LOS CENTROS DE MASA DE LOS DOS BRAZOS
        # ============================ ============================ ============================ ============================

        X1 = [ L1/2 * cos(θ1) ;  L1/2 * sin(θ1) ] 
        X2 = [ L1 *cos(θ1) + L2/2 * cos(θ1 + θ2) ; L1 * sin(θ1) + L2/2 * sin(θ1 + θ2) ]
        
        # ============================ ============================ ============================ ============================
        # CÁLCULO Y DECLARACIÓN DE MATRICES 
        # ============================ ============================ ============================ ============================

        # 1. ELEMENTO DIFERENCIAL
        dx11 = ∂θ1.(X1) ; dx12 = ∂θ2.(X1)
        dx21 = ∂θ1.(X2) ; dx22 = ∂θ2.(X2)


        # 2. Construcción del jacobiano
        Jc1 = Symbolics.jacobian(X1, [θ1, θ2]);   Jc2 = Symbolics.jacobian(X2, [θ1, θ2])

        # 3. Cálculo de theta punto o q_{dot} 
        q = [ dθ1  ; dθ2 ]

        # 4. Cálculo de la derivada completa
        Dx1T = Jc1*q;  Dx2T = Jc2*q

        # 5. MAGNITUD AL CUADRADO
        DX1 = Dx1T'*Dx1T;  DX2 = Dx2T'*Dx2T

        # ============================ ============================ ============================ ============================
        # CÁLCULO DE ENERGÍA CINÉTICA DE LOS BRAZOS
        # ============================ ============================ ============================ ============================

        I1 = (m1/12) * (L1^2 + W1^2) + m1*(L1/2)^2 * 0.6
        I2 = (m2/12) * (L2^2 + W2^2) 

        T1 = (I1/2) * dθ1^2

        T2t = (m2/2) * DX2
        T2R = (I2/2) * ( dθ1 + dθ2 )^2

        Ttotal = T1 + T2t + T2R

        # ============================ ============================ ============================ ============================
        # CÁLCULO DE ENERGÍA POTENCIAL DE LOS BRAZOS
        # ============================ ============================ ============================ ============================

        Vx1 = m1 * g * (L1/2)*sin(θ1)

        Vx2 = m2 * g * ( (L1)*sin(θ1) + (L2/2)*sin(θ1 + θ2) )

        Vtotal = Vx1 + Vx2

        # ============================ ============================ ============================ ============================
        # CÁLCULO DE LAGRANGIANO
        # ============================ ============================ ============================ ============================

        L = Ttotal - Vtotal

        # ============================ ============================ ============================ ============================
        # CÁLCULO DE TORQUE L1
        # ============================ ============================ ============================ ============================

        # 1. Derivada con respecto a la velocidad
        dLspeed = ∂2_θ1(L)

        dLspeed_exp = Symbolics.expand_derivatives(dLspeed)

        # 2. Derivada con respecto al tiempo
        dLtime = ∂t(dLspeed_exp)

        dLtime_exp = Symbolics.expand_derivatives(dLtime)

        # 3. Derivada con respecto a la posición
        dLposition = ∂θ1(L)

        dLposition_exp = Symbolics.expand_derivatives(dLposition)

        # 4. Cálculo de torque
        Q1 = dLtime_exp - dLposition_exp

        # ============================ ============================ ============================ ============================
        # CÁLCULO DE TORQUE L2
        # ============================ ============================ ============================ ============================

        # 1. Derivada con respecto a la velocidad
        dLspeed2 = ∂2_θ2(L)

        dLspeed_exp2 = Symbolics.expand_derivatives(dLspeed2)

        # 2. Derivada con respecto al tiempo
        dLtime2 = ∂t(dLspeed_exp2)

        dLtime_exp2 = Symbolics.expand_derivatives(dLtime2)

        # 3. Derivada con respecto a la posición
        dLposition2 = ∂θ2(L)

        dLposition_exp2 = Symbolics.expand_derivatives(dLposition2)

        # 4. Cálculo de torque
        Q2 = dLtime_exp2 - dLposition_exp2

        vals = Dict(m1=>m1R, m2=>m2R, L1=>L1R, L2=>L2R, g=>gR, W1=>W1R, W2=>W2R)
        
        # Sustituir valores numéricos para verificar
        params = Dict(
            m1 => m1R, m2 => m2R,
            L1 => L1R, L2 => L2R,
            W1 => W1R, W2 => W2R,
            g => gR
        )

        # Evaluar en una configuración específica
        
        
        Q1_num = substitute(Q1, params)
        Q2_num = substitute(Q2, params)

        return simplify(Q1_num), simplify(Q2_num)
    end

    # ============================ ============================ ============================ 
    # FUNCIONES AUXILIARES COMPLETAS
    # ============================ ============================ ============================ 

    function procesar_estructura_entrada(trayectoria, Ts)
        if typeof(trayectoria) <: DataFrame
            # Entrada como DataFrame
            required_cols = ["t", "theta1", "theta2", "dtheta1", "dtheta2", "ddtheta1", "ddtheta2"]
            if all(in.(required_cols, Ref(names(trayectoria))))
                return (trayectoria.t, trayectoria.theta1, trayectoria.theta2,
                    trayectoria.dtheta1, trayectoria.dtheta2, 
                    trayectoria.ddtheta1, trayectoria.ddtheta2)
            else
                error("DataFrame no tiene las columnas requeridas")
            end
        elseif typeof(trayectoria) <: Dict
            # Entrada como diccionario
            t = get(trayectoria, :t, get(trayectoria, "t", nothing))
            θ1 = get(trayectoria, :theta1, get(trayectoria, "theta1", nothing))
            θ2 = get(trayectoria, :theta2, get(trayectoria, "theta2", nothing))
            dθ1 = get(trayectoria, :dtheta1, get(trayectoria, "dtheta1", nothing))
            dθ2 = get(trayectoria, :dtheta2, get(trayectoria, "dtheta2", nothing))
            ddθ1 = get(trayectoria, :ddtheta1, get(trayectoria, "ddtheta1", nothing))
            ddθ2 = get(trayectoria, :ddtheta2, get(trayectoria, "ddtheta2", nothing))
            
            if any(x === nothing for x in [t, θ1, θ2, dθ1, dθ2, ddθ1, ddθ2])
                error("Faltan columnas requeridas en el diccionario")
            end
            return (t, θ1, θ2, dθ1, dθ2, ddθ1, ddθ2)
        elseif typeof(trayectoria) <: Tuple && length(trayectoria) == 7
            # Entrada como tupla (t, θ1, θ2, dθ1, dθ2, ddθ1, ddθ2)
            return trayectoria
        else
            error("Estructura de entrada no reconocida. Use DataFrame, Dict o Tuple de 7 elementos")
        end
    end

    function validar_dimensiones(vectores, N_esperado)
        for (i, vec) in enumerate(vectores)
            if length(vec) != N_esperado
                error("Vector $i tiene longitud $(length(vec)), se esperaba $N_esperado")
            end
        end
    end

    function validar_datos_numericos(θ1, θ2, dθ1, dθ2, ddθ1, ddθ2)
        # Verificar NaN/Inf
        all_arrays = [θ1, θ2, dθ1, dθ2, ddθ1, ddθ2]
        for (i, arr) in enumerate(all_arrays)
            if any(isnan.(arr)) || any(isinf.(arr))
                error("Vector $i contiene NaN o Inf")
            end
        end
        
        # Verificar rangos razonables (ángulos en radianes)
        if any(abs.(θ1) .> 4π) || any(abs.(θ2) .> 4π)
            @warn "Algunos ángulos exceden 4π radianes - verificar unidades (¿radianes?)"
        end
        
        println("✓ Validación numérica: OK")
    end

    function reportar_estadisticas(tau1, tau2, label1, label2)
        stats1 = (minimum(tau1), maximum(tau1), std(tau1), sqrt(mean(tau1.^2)))
        stats2 = (minimum(tau2), maximum(tau2), std(tau2), sqrt(mean(tau2.^2)))
        
        println("Estadísticas $label1: min=$(round(stats1[1], digits=3)), max=$(round(stats1[2], digits=3)), σ=$(round(stats1[3], digits=3)), RMS=$(round(stats1[4], digits=3)) N·m")
        println("Estadísticas $label2: min=$(round(stats2[1], digits=3)), max=$(round(stats2[2], digits=3)), σ=$(round(stats2[3], digits=3)), RMS=$(round(stats2[4], digits=3)) N·m")
    end

    function verificar_saturacion(tau1, tau2, τ_max)
        sat1 = sum(abs.(tau1) .> τ_max)
        sat2 = sum(abs.(tau2) .> τ_max)
        
        if sat1 > 0
            @warn "Saturación en tau1: $sat1 puntos exceden ±$τ_max N·m"
        end
        if sat2 > 0
            @warn "Saturación en tau2: $sat2 puntos exceden ±$τ_max N·m"
        end
        if sat1 == 0 && sat2 == 0
            println("✓ Sin saturación: todos los torques dentro de ±$τ_max N·m")
        end
    end

    function detectar_discontinuidades(tau1, tau2, t)
        # Calcular derivadas numéricas
        dt = diff(t)
        dtau1_dt = diff(tau1) ./ dt
        dtau2_dt = diff(tau2) ./ dt
        
        # Umbral para discontinuidad (3σ)
        umbral1 = 3 * std(dtau1_dt)
        umbral2 = 3 * std(dtau2_dt)
        
        disc1 = sum(abs.(dtau1_dt) .> umbral1)
        disc2 = sum(abs.(dtau2_dt) .> umbral2)
        
        if disc1 > 0 || disc2 > 0
            @warn "Posibles discontinuidades detectadas: $disc1 en tau1, $disc2 en tau2"
        else
            println("✓ Perfiles de torque suaves: sin discontinuidades significativas")
        end
    end

    using GLMakie

function generar_graficas(t, tau1, tau2, τ_max)
    # Crear figura con 2 subplots
    fig = Figure(size = (800, 600))

    # ---------------- Plot τ1 ----------------
    ax1 = Axis(fig[1, 1],
        title = "Torque en articulación 1",
        xlabel = "Tiempo [s]",
        ylabel = "Torque [mN·m]"
    )
    lines!(ax1, t, tau1, linewidth = 2, label = "τ₁(t)")

    if τ_max !== nothing
        lines!(ax1, t,  fill( τ_max, length(t)), linestyle = :dash)
        lines!(ax1, t,  fill(-τ_max, length(t)), linestyle = :dash)
    end
    axislegend(ax1, position = :rt)

    # ---------------- Plot τ2 ----------------
    ax2 = Axis(fig[2, 1],
        title = "Torque en articulación 2",
        xlabel = "Tiempo [s]",
        ylabel = "Torque [mN·m]"
    )
    lines!(ax2, t, tau2, linewidth = 2, label = "τ₂(t)")

    if τ_max !== nothing
        lines!(ax2, t,  fill( τ_max, length(t)), linestyle = :dash)
        lines!(ax2, t,  fill(-τ_max, length(t)), linestyle = :dash)
    end
    axislegend(ax2, position = :rt)

    display(fig)
end


    # ============================ ============================ ============================ 
    # FUNCIÓN PRINCIPAL CORREGIDA
    # ============================ ============================ ============================ 

    function torque_profile_from_Q(Q1_sym, Q2_sym, trayectoria; 
                                τ_max=nothing, 
                                export_csv=false, 
                                timestamp = Dates.format(now(), "yyyy-mm-dd_HHMMSS"),
                                filename = "torque_profile_$(timestamp).csv",
                                output_dir="./Data",   
                                debug=false,
                                Ts=nothing)
        # ============================ ============================ ============================ 
        # 1. VALIDACIONES INICIALES Y PREPROCESAMIENTO
        # ============================ ============================ ============================ 
        @variables  t
        @variables  θ1(t) θ2(t)

        ∂t =  Differential(t)

        dθ1 = ∂t(θ1)
        dθ2 = ∂t(θ2)


        println("=== INICIALIZANDO CÁLCULO DE PERFIL DE TORQUES ===")
        
        # Procesar la estructura de entrada
        t, θ1_vec, θ2_vec, dθ1_vec, dθ2_vec, ddθ1_vec, ddθ2_vec = procesar_estructura_entrada(trayectoria, Ts)
        
        N = length(t)
        println("✓ Perfil procesado: N = $N puntos")
        println("✓ Rango temporal: t ∈ [$(minimum(t)), $(maximum(t))] s")
        
        # Validar coherencia de dimensiones
        validar_dimensiones([θ1_vec, θ2_vec, dθ1_vec, dθ2_vec, ddθ1_vec, ddθ2_vec], N)
        
        # Validar datos numéricos 
        validar_datos_numericos(θ1_vec, θ2_vec, dθ1_vec, dθ2_vec, ddθ1_vec, ddθ2_vec)
        
        # ============================ ============================ ============================ 
        # 2. SIMPLIFICACIÓN PREVIA
        # ============================ ============================ ============================ 
        
        println("Simplificando expresiones...")
        
        # Simplificar expresiones una sola vez
        Q1_simple = Symbolics.simplify(Q1_sym)
        Q2_simple = Symbolics.simplify(Q2_sym)
        
        # ============================ ============================ ============================ 
        # 3. EVALUACIÓN PUNTO A PUNTO CON SUBSTITUTE - CORREGIDA
        # ============================ ============================ ============================ 
        
        println("Evaluando torques en $N puntos...")
        
        tau1 = zeros(N)
        tau2 = zeros(N)
        
        tiempo_inicio = time()
        
        # Evaluar punto a punto usando substitute
        for i in 1:N
            # Crear configuración para este punto
            config = Dict(
                θ1 => θ1_vec[i],
                θ2 => θ2_vec[i],
                dθ1 => dθ1_vec[i],
                dθ2 => dθ2_vec[i],
                ∂t(dθ1) => ddθ1_vec[i],  # ∂t(dθ1) representa ddθ1
                ∂t(dθ2) => ddθ2_vec[i]   # ∂t(dθ2) representa ddθ2
            )
            
            # Evaluar sustituyendo valores y extraer valor numérico
            tau1_val = substitute(Q1_simple, config)
            tau2_val = substitute(Q2_simple, config)
            
            # Convertir de Num a Float64
            τ_lim = 400; 
            tau1[i] = -clamp(Symbolics.value(tau1_val)*1000, -τ_lim, τ_lim)
            tau2[i] = -clamp(Symbolics.value(tau2_val)*1000, -τ_lim/2, τ_lim/2)
            
            # Mostrar progreso para trayectorias largas
            if debug && (i % 100 == 0 || i == N)
                println("  Procesado punto $i/$N - τ1: $(round(tau1[i], digits=3)), τ2: $(round(tau2[i], digits=3))")
            end
        end
        
        tiempo_ejecucion = time() - tiempo_inicio
        println("✓ Evaluación completada en $(round(tiempo_ejecucion, digits=4)) s")
        println("✓ Tasa: $(round(N/tiempo_ejecucion, digits=2)) puntos/s")
        
        # ============================ ============================ ============================ 
        # 4. VALIDACIONES POST-CÁLCULO
        # ============================ ============================ ============================ 
        
        println("\n=== VALIDACIONES POST-CÁLCULO ===")
        
        # Estadísticas básicas
        reportar_estadisticas(tau1, tau2, "tau1", "tau2")
        
        # Verificar saturación
        if τ_max !== nothing
            verificar_saturacion(tau1, tau2, τ_max)
        end
        
        # Detectar discontinuidades
        detectar_discontinuidades(tau1, tau2, t)
        
        # ============================ ============================ ============================ 
        # 5. GENERACIÓN DE GRÁFICAear 
        # ============================ ============================ ============================ 
        
        println("\nGenerando gráficas...")
        generar_graficas(t, tau1, tau2, τ_max)
        
        # ============================ ============================ ============================ 
        # 6. PREPARACIÓN DE SALIDA
        # ============================ ============================ ============================ 
        
        # Crear DataFrame completo
        df_resultado = DataFrame(
            t = t,
            tau1 = tau1,
            tau2 = tau2,
            theta1 = θ1_vec,
            theta2 = θ2_vec,
            dtheta1 = dθ1_vec,
            dtheta2 = dθ2_vec,
            ddtheta1 = ddθ1_vec,
            ddtheta2 = ddθ2_vec
        )
        
        # Exportar CSV si se solicita
        if export_csv
            mkpath(output_dir)                               # crea la carpeta si no existe
            filepath = joinpath(output_dir, filename)        # arma la ruta de forma portable
            CSV.write(filepath, df_resultado)
            println("✓ Datos exportados a: $filepath")
        end

        println("\n=== CÁLCULO COMPLETADO ===")
        
        return (t=t, tau1=tau1, tau2=tau2, dataframe=df_resultado)
    end

    # Función auxiliar para crear configuraciones de prueba
    function crear_configuracion_punto(θ1_val, θ2_val, dθ1_val=0.0, dθ2_val=0.0, ddθ1_val=0.0, ddθ2_val=0.0)
        return Dict(
            θ1 => θ1_val,
            θ2 => θ2_val,
            dθ1 => dθ1_val,
            dθ2 => dθ2_val,
            ∂t(dθ1) => ddθ1_val,
            ∂t(dθ2) => ddθ2_val
        )
    end

    # Función de prueba estática actualizada
    function prueba_modo_estatico(Q1_sym, Q2_sym, θ1_test, θ2_test)
        """
        Función de depuración: evalúa torques en configuración estática
        """
        println("\n=== PRUEBA MODO ESTÁTICO ===")
        println("Configuración: θ1 = $θ1_test rad, θ2 = $θ2_test rad")
        println("Velocidades y aceleraciones: 0")
        
        # Simplificar primero
        Q1_simple = Symbolics.simplify(Q1_sym)
        Q2_simple = Symbolics.simplify(Q2_sym)
        
        # Crear configuración estática
        config = crear_configuracion_punto(θ1_test, θ2_test, 0.0, 0.0, 0.0, 0.0)
        
        # Evaluar y extraer valores numéricos
        torque1_estatico = Symbolics.value(substitute(Q1_simple, config))
        torque2_estatico = Symbolics.value(substitute(Q2_simple, config))
        
        println("τ₁ (estático) = $(round(torque1_estatico, digits=4)) N·m")
        println("τ₂ (estático) = $(round(torque2_estatico, digits=4)) N·m")
        
        return torque1_estatico, torque2_estatico
    end

    # Función para evaluación rápida de un solo punto
    function evaluar_torque_punto(Q1_sym, Q2_sym, θ1_val, θ2_val, dθ1_val=0.0, dθ2_val=0.0, ddθ1_val=0.0, ddθ2_val=0.0)
        """
        Evalúa los torques en un solo punto específico
        """
        Q1_simple = Symbolics.simplify(Q1_sym)
        Q2_simple = Symbolics.simplify(Q2_sym)
        
        config = crear_configuracion_punto(θ1_val, θ2_val, dθ1_val, dθ2_val, ddθ1_val, ddθ2_val)
        
        tau1 = Symbolics.value(substitute(Q1_simple, config))
        tau2 = Symbolics.value(substitute(Q2_simple, config))
        
        return tau1, tau2
    end

end
