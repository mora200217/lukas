function [Kt, Kb, tau_in] = motor_dc_parameters(t, theta, motor, graph)
    % MOTOR_DC_PARAMETERS Calcula Kt y Kb de un motor DC
    % a partir de la respuesta angular del eje.
    %
    % Uso:
    %   [Kt, Kb, tau] = motor_dc_parameters(t, theta, motor, true)
    %
    % Entradas:
    %   t      : vector de tiempo [s]
    %   theta  : vector de posición angular [rad]
    %   motor  : struct con campos:
    %              J  -> inercia [kg·m²]
    %              Ra -> resistencia [Ω]
    %              La -> inductancia [H]
    %              B  -> fricción viscosa [kg/s]
    %              A  -> voltaje aplicado [V]
    %   graph  : (opcional) booleano para graficar resultados
    %
    % Salidas:
    %   Kt     : constante de torque [Nm/A]
    %   Kb     : constante de FEM [V·s/rad]
    %   tau_in : constante de tiempo estimada [s]

    if nargin < 4
        graph = false;
    end

    %% Extraer parámetros
    J = motor.J;
    Ra = motor.Ra;
    B = motor.B;
    A = motor.A;

    %% Pendiente final y ajuste
    m = (theta(end) - theta(end-1)) / (t(end) - t(end-1)); 
    [tau_in, b] = get_tau(m, t(end), theta(end));

    %% Cálculo de constantes
    tau = tau_in;
    Kt = (m / A) * J * Ra / tau;
    Kb = (1 - B * tau / J) / m * A;

    %% Mostrar resultados
    fprintf("Kt = %.6f Nm/A\n", Kt);
    fprintf("Kb = %.6f V·s/rad (%.1f rpm/V)\n", Kb, 1 / (Kb * pi / 30));

    %% Gráficos opcionales
    if graph
        figure; hold on; grid on;
        plot(t, theta, 'b', 'LineWidth', 1.5);
        xlabel("Tiempo [s]"); ylabel("\theta [rad]");
        title("Respuesta del motor DC");
        xline(tau_in, '--r', 'Constante de tiempo');
        y = m * t + b;
        plot(t, y, '--k');
        primer_ref = 0.5 * max(y) * (1 - exp(-t / tau));
        plot(t, primer_ref, '--g');
        legend("θ(t)", "τ", "Tangente final", "Referencia 1er orden");
        hold off;
    end
end
