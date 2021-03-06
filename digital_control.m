%% Setup
clc
clear all
close all
s = tf('s');
Gs = (-1.8 * (s + 0.02) * (s + 0.5)) / ((s^2 + 1.2 * s + 12) * (s^2 + 0.01 * s + 0.0025));
Gtest = (s + 0.02) / (s^2 + 1.2 * s + 12);
Gsnum = [-1.8 -0936 -0.018];
Gsdenom = [1 1.21 12.01 0.123 0.03];
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
    Gsnewnum = [-1.8 -0936 -0.018];
    Gsnewdenom = [1 0.01 0.0025];
end
%% Step Response of Gs and Gs new
close all
figure("name","step response Gs")
step(Gs)
% figure("name","step response Gsnew without controller")
hold on
step(Gsnew)
legend("GS","Gsnew")
hold off

%% PID with removed poles
close all
figure("name","Bode plot Gs without controller")
bode(Gs)
% figure("name","Bode plot Gsnew without controller")
% bode(Gsnew)
Kp = 8;     %ideal 8
Ki = 10;    %ideal 10
Kd = 6;     %ideal 6
% test = pidTuner(Gsnew,"PIDF")
PID = pid(Kp,Ki,Kd);
C = PID ;
sysNewPID = feedback(C*Gsnew,1);
% figure("name","step response controller")
% step(C)
[yPID,t] = step(sysNewPID);
sserror=abs(1-yPID(end)) %get the steady state error
S = stepinfo(sysNewPID)
sysPID = feedback(C*Gs,1);
figure("name","step response Gsnew with PID controller")
step(sysNewPID)
figure("name","step response Gs with PID controller")
step(sysPID)

%% Lead lag with pid
close all
% figure("name","Bode plot Gs without controller")
% bode(Gs)
C_lead = (s+100)/(s+20);
sysLead = C_lead * Gs;
% figure("name","Bode plot Gs with controller")
% bode(sysLead)
Kp = 1;     %ideal 8
Ki = 0;    %ideal 10
Kd = 0;     %ideal 6
% test = pidTuner(Gsnew,"PIDF")
PID = pid(Kp,Ki,Kd);
C = PID ;
sysNewPID = feedback(sysLead,1);
figure("name","step response Gs with lead and PID controller")
step(sysNewPID)
figure("name","ramp response")
step(sysNewPID/s)
%% rlocus
figure
rlocus(Gsnew)
zeta = 0.7;
wn = 1.8;
sgrid(zeta,wn)
[k,poles] = rlocfind(Gsnew)

sys_cl = feedback(k*Gs,1)
step(sys_cl)

