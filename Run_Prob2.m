% Sandesh Thapa 
% Homework 1 
% MAE 5010 Flight Controls 
% Problem 2 

%% Part a 
clc; 
clear all; 
close all; 

x0 = [0;0;0;0];

u0 = 272; 
Lv = 580; 
alv = 10; 

num = alv*sqrt(Lv/u0)*[0.3398*(Lv/u0)^2 2.7478*(Lv/u0) 1];
den = [0.1539*(Lv/u0)^3 1.9754*(Lv/u0)^2 2.9958*(Lv/u0) 1];

sys1 = tf(num,den)

sim('Prob2')

figure
plot(time,Noise); 
xlabel('$Time$ $(sec)$','interpreter','latex','FontSize', 20); 
ylabel('$Noise$ $n(t)$','interpreter','latex','FontSize', 20);
hold on 

figure
plot(time,vg); 
xlabel('$Time$ $(sec)$','interpreter','latex','FontSize', 20); 
ylabel('$V_g(t)$ $(m/s)$','interpreter','latex','FontSize', 20);
hold on 

figure
plot(time,y); 
xlabel('$Time$ $(sec)$','interpreter','latex','FontSize', 20); 
ylabel('$a_y$ $(m/s^2)$','interpreter','latex','FontSize', 20);

%% Part B 
% From numerical caluclations we got from state space
% Controllabe Cannoical form 
Ag = [-6.20 1 0; 
      -4.218 0 1 ; 
      -0.6702 0 0];
Bg = [15.120;57.246;9.786]; 
Cg = [1 0 0];
Dg = 0;

A = [-0.5,  0, -1, 0.02;
     -150, -7, -0.15, 0;
     30, 0.1, -1, 0; 
     0 1 0 0]; 
 
B = [-0.02, 56, 1, 0]';

G = [0.5; 150; -30; 0]; 
u0 = 272; % m/s 
C = [-0.5*u0 0 0 0];

M1 = [A ;zeros(3,4)];
M2 = [G*Cg; Ag];
M = [M1,M2];

N = [zeros(4,1); Bg]

Al = M; 
Ql = N*N';

X_cap = lyap(Al,Ql);

F = [C zeros(1,3)]

rms = sqrt(F*X_cap*F')
%%

% Check using matlab 
[Agg,Bgg,Cgg,Dgg] = tf2ss(num,den)
% Matlab gives the matrics in observable cannonical form 

M1g = [A ;zeros(3,4)];
M2g = [G*Cgg; Agg];
Mgg = [M1g,M2g];

Ngg = [zeros(4,1); Bgg]

Alg = Mgg; 
Qg1 = Ngg*Ngg';

X_capg = lyap(Alg,Qg1);

Fg = [C zeros(1,3)]

rms = sqrt(Fg*X_capg*Fg')

