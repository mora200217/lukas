%% Controller Design 
clear
load("ft.mat")

s = tf('s'); 
%%  Root Locus design 

Csrl = (1/(0.1 * s) + 4); % Controlador de diseño rl 

Gamp = Csrl * G1tf; 
rlocus(Gamp) % K = 0.6 

%% 
Cs = Csrl * 0.5
Cs = pid(Cs)

%%
Cs = pid(10, 0.8)
Gl = Cs * G1tf; 

T1 = feedback(Gl, 1); 

step(T1); 

%% Freq
bode(T1)
