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
P = pole(Gs);
Z = zero(Gs);
if abs(P(1)/P(3)) > 5
    disp("P(1)/P(3) > 5")
    Gsnew = (-1.8 * (s + 0.02) * (s + 0.5)) / (s^2 + 0.01 * s + 0.0025);
end
%%
close all
figure("name","step response Gs")
step(Gs)
figure("name","step response Gsnew without controller")
step(Gsnew)
% figure("name","Bode plot Gs without controller")
% bode(Gs)
% figure("name","Bode plot Gsnew without controller")
% bode(Gsnew)
Kp = 8;     %ideal 8
Ki = 10;    %ideal 10
Kd = 6;     %ideal 6
% test = pidTuner(Gsnew,"PIDF")
PID = pid(Kp,Ki,Kd);
C = PID ;
sysPID = feedback(C*Gs,1);
% figure("name","step response controller")
% step(C)
[yPID,t] = step(sysPID);
sserror=abs(1-yPID(end)) %get the steady state error
S = stepinfo(sysPID)
figure("name","step response Gsnew with PID controller")
step(sysPID)
