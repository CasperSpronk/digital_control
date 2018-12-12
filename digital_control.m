%% Setup
clc
clear all
close all
s = tf('s');
Gs = (-1.8 * (s + 0.02) * (s + 0.5)) / ((s^2 + 1.2 * s + 12) * (s^2 + 0.01 * s + 0.0025));
Gtest = (s + 0.02) / (s^2 + 1.2 * s + 12);
%% Creating a PID controller
% goals:  minimal settling time
%         overshoot < 5%
%         steady-state error = 0
close all
pidTuner(Gs,"pid")

%%
step(C*Gs)



%%
Kp = -19.1409;
Ki = -18.9803;
Kd = -4.7947;
Tf = 0.00086513;
PID = pid(Kp,Ki,Kd,Tf);
%PD = pid(1,0,Kd2);
C = PID ;%+ PD;
sys = C * Gs;
step(sys)
S = stepinfo(sys)
P = pole(Gs);

