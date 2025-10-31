%% Simulación de Motor DC 
clc; clear; close all; 

%% Parámetros  ============================================
J   =  45.3e-7; % [kg * m^2]
Ra  =  3.99; % [Ohms]
La  = 556e-6; % [H]
B   = 0.0001; % [Kg / s]

A   = 12; % [V]
%% Resultados de simulación ============================================
simr = sim("motor_dc_verification.slx", 1); 

Fs = 1e3; 
Ts = 1 / Fs; 


t     = simr.tout; 
theta = simr.simout.Data;  


 
m = (theta(end-1) - theta(end-2)) / (t(end -1) - t(end -2)); 
[tau_in, b] = get_tau(m, t(end), theta(end)); 

% Verificacion de tiempo transitorio 
plot(t, theta); grid on; 
xlabel("Tiempo [s]"); ylabel("\theta [rad]")
xline(tau_in); 
title("Respuesta posición eje Motor DC"); 

hold on; 
y = m * t + b; 
plot(t, y); 
yaxis([0, max(y)])
hold on; 


% Cálculo de características ============================================
tau  = tau_in; 
Kt = m/A * J * Ra/tau; 
Kb = (1- B * tau/J) / m * A;  % [V / rad/s ]

clc; 
disp("Kt: " + round(Kt, 4) + " Nm/A"); 
disp("Kb: " + round(Kb, 5) + " V/rad/s | Kb = " + round(1/(Kb * pi / 30)) + " [rpm/V]"); 

% Comentario: jasdjasjdj no podemos ignorar tan duro el amortiguamiento 
% Tiene mejor aproximación 

% C /( ts + 1)

primer_ref = 1/2 * max(y) * (1- exp(-t/tau)); 
plot(t, primer_ref)

hold off; 
