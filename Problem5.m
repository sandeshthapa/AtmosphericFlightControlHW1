%Problem 5 
% Part a 
clc; close all; clear all; 

num = [0.038 0];
den = [1 0.039 0.053];

Air_dyns = tf(num,den)

Eng_dyns = tf([10],[10 1])

Servo_dyns = tf([1],[0.1 1])

Sen_dyns = tf([10 1],[1])

sys = Servo_dyns*Eng_dyns*Air_dyns*Sen_dyns
sys1 = Servo_dyns*Eng_dyns*Air_dyns;

figure 
hold on 
x1 = .0001:.0001:2*pi*1;
y1 = 26*ones(size(x1));
y2 = 0:0.01:26; 
x2 = 2*pi*1*ones(size(y2));
% plot(x1,y1,'k')
plot(x1,y1,'r',x2,y2,'r','LineWidth',1.5)
hold on 
margin(sys)
hold on 

%% PI Controller 
% Kp = 400; 
% Ki = 50;
K = 300;
% Ki = 1;

w_pi = 4; % rad/s 
tau_pi = 1/w_pi; 

% Kp = 1;
Ki = K/tau_pi; 

C1 = tf([K*tau_pi K],[tau_pi 0])
L_1 = C1*sys

figure 
hold on 
% x1 = .0001:.0001:2*pi*1;
% y1 = 26*ones(size(x1));
% y2 = 0:0.01:26; 
% x2 = 2*pi*1*ones(size(y2));
% plot(x1,y1,'k')
plot(x1,y1,'r',x2,y2,'r','LineWidth',1.5)
hold on 
margin(L_1)

%% Lead Compensator
[Gm,Pm,Wgm,Wpm] = margin(C1*sys)

del_Pm = 40.4 - Pm + 5
% alpha = (1-sind(del_Pm)/(1+ sind(del_Pm)))
alpha = 0.067
% alpha = 0.8
k = 0.01; 
% k = 1.02;
% syms k real  
Wc = Wpm; 
% tau = 1/(Wc*sqrt(alpha))
tau = 0.082
C2 = tf([k*tau k],[alpha*tau 1])

L_s = C1*C2*L_1
figure 
hold on 
% x1 = .01:.0001:1;
% y1 = 26*ones(size(x1));
plot(x1,y1,'k');
plot(x1,y1,'r',x2,y2,'r','LineWidth',1.5)
hold on 
margin(L_s)
figure 
cl_sysLPI = feedback(C1*C2*Servo_dyns*Eng_dyns*Air_dyns, Sen_dyns);
margin(cl_sysLPI)
 hold on 

%%
figure 
step(cl_sysLPI)
stepinfo(cl_sysLPI)
bandwidth(cl_sysLPI)
t = [0:0.1: 1000];
w = 0.16/100;
u = sin(2*pi*w*t); 

y1 = lsim(cl_sysLPI,u,t);
figure 
plot(t,y1,t,u); 
legend('y', 'u')
ylabel('Amplitude');
xlabel('Time (s)')
hold on 

e = (u' - y1) ; 
figure 
plot(t,e);
legend('eror')


