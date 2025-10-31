% Parametros de Motor 
clc; clear; close all; 

J  = 45.3e-7;
Ra = 3.99;
La = 556e-7;
B  = 0.0001;
A  = 12;

%%

% Definir el modelo del motor DC
motor = struct('J', J, 'Ra', Ra, 'La', La, 'B', B, 'A', A);
%%
% Correr simulación
sim_time = 0.8;  % [s]
simout = sim("motor_dc_verification.slx", sim_time);
%%
% Extraer señales
t = simout.tout;
theta = simout.simout.Data;


[Kt, Kb, tau] = motor_dc_parameters(t, theta, motor, true);
