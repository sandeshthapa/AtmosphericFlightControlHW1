
%Problem 4 
% Part a 

clc; 
close all;
clear all; 

% P Controller 
Kp = 550;
% Kp = 500
num = [0.25*Kp];
den = [0.0475 0.05 0.735];

sys = tf(num,den)
figure 
title('Open Loop Plot with a P Controller')
x1 = .01:.01:2*pi*1;
y1 = 26*ones(size(x1));
y2 = 0:0.01:26; 
x2 = 2*pi*1*ones(size(y2));
x3 = 2*pi*100:0.01:10000;
y3 = -20*ones(size(x3));
y4 = -20:0.01:100; 
x4 = 2*pi*100*ones(size(y4));

plot(x1,y1,'r',x2,y2,'r','LineWidth',1.5)
hold on 
plot(x3,y3,'r','LineWidth',1.5)
hold on 
plot(x4,y4,'r','LineWidth',1.5)
% plot(x3,y3)
hold on 
margin(sys)
opts = bodeoptions('cstprefs');
% opts.PhaseVisible = 'off';
% opts.FreqUnits = 'Hz';
opts.Xlim = [0.01 2*pi*100]; 
% opts.Ylim = [-40 80]
% axis([0:01 2*pi*100], [-40 80])
grid on 
hold on
bodeplot(sys,opts)
[Gm,Pm,Wgm,Wpm] = margin(sys)

%% Lead Compensator

alpha = 0.07; 
tau = 0.0527; 
k = 0.65; 

% alpha = 0.136; 
% tau = 0.0527; 
% k = 1.0; 


C_s = tf([k*tau k],[alpha*tau 1])

L_s = C_s*sys
figure 
% x1 = .01:.0001:1;
% y1 = 40*ones(size(x1));
plot(x1,y1,'r',x2,y2,'r','LineWidth',1.5)
hold on 
plot(x3,y3,'r','LineWidth',1.5)
hold on 
plot(x4,y4,'r','LineWidth',1.5)
% plot(x3,y3)
hold on 
margin(L_s)
hold on 
%%

figure 
cl_sys = feedback(sys*C_s, 1)
margin(cl_sys)
hold on 
figure 
step(cl_sys)
stepinfo(cl_sys)

%%
t = [0:0.01: 10];
w = 0.16;
u = sin(2*pi*w*t); 

y = lsim(cl_sys,u,t);
ylabel('Amplitude');
xlabel('Time (s)')
figure 
plot(t,y,t,u); 
legend('y', 'u')
ylabel('Amplitude');
xlabel('Time (s)')

hold on 

e = (u' - y) ; 
figure 
plot(t,e);
legend('eror')
