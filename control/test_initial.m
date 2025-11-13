%% Control prueba
clear; clc; close all; 

A = [0 1; 3, 4]; 
B = [0 1]'; 
C = [1 0]; 
D =  0; 

G = ss(A, B, C, D); 
%% PLanta ampliada 

Aa = [A - L*C']
Ba = 0 
Ca = [C]'
D = 0; 

%% LQR SOlucion 
Q = 5000; 
R = 6; 

[L, S] = lqr(G, Q, R); 

%% 




 



