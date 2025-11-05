% Prueba de conexión de encoder via UART 
clc; clear ; close all
delete(serialportfind); 
serialportlist("available")

%% 
serialObj = serialport("/dev/tty.usbmodem1301",9600); 

%% Serial 
configureTerminator(serialObj,"CR/LF");
flush(serialObj);
serialObj.UserData = struct("Data",[],"Count",1); 

Fs = 1e3; % [Hz]
Ts = 1/Fs; 
T = 4; % [s]
maxDataPoints = 1000; 

configureCallback(serialObj, "terminator", @(src,event) read_encoder(src,event,maxDataPoints))


%% Parametros 
theta = serialObj.UserData.Data; 
t = linspace(0, maxDataPoints * 1e-3, length(theta)); 

J  = 45.3e-7;
Ra = 3.99;
La = 556e-7;
B  = 0.0001;
A  = 12;

motor = struct('J', J, 'Ra', Ra, 'La', La, 'B', B, 'A', A);

[Kt, Kb, tau] = motor_dc_parameters(t, theta, motor, true);


%% 
save( motor, Kt, Kb, tau, t, theta); 

