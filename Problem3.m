% Sandesh Thapa 
% Homework 1 
% MAE 5010 Flight Controls 
% Problem 3

clc; 
close all;
clear all; 

%% Part a

syms x real
solx = solve (0.3739*x^4 + 1.3343*x^3+ 0.5280*x^2+ 0.0381*x  -0.0235 ==0,x);

r = roots([0.3739 1.3343 0.5280 0.0381  -0.0235])

%% Inner Loop 
num1 = [0.0602 4.4326 0.2008  0.3103];
num2 = [4.4642 9.3994 0.4775 0];

den = [0.3739 1.3343 0.5280 0.0381  -0.0235];

sys1 = tf(num1,den);
figure
rlocus(sys1)
title('Open Loop Root Locus Inside Loop')
% [Wno,Zo,Po]= damp(sys1)
hold on 

Ka = 0.3;
% Ka = 0.25;
sys_cl = feedback(Ka*sys1,1)
[Wn,Z,P] = damp(sys_cl)
damp(sys_cl)

figure 
rlocus(sys_cl);
title('Close Loop Root Locus Inside Loop')
hold on 
figure 
step(sys_cl)
hold on
stepinfo(sys_cl)
%% Outer loop
sys2 = tf(num2,num1)

% K_ss = 0.3103/0.0235;
figure 
rlocus(sys2)
title('Open Loop Root Locus Outside Loop')
[Wn2o,Z2o,P2o]= damp(sys2)
hold on 
% K_ss = 1.1;

%open loop system 
Kq = 0.75;
sys_2OL = sys_cl*Kq*sys2
[Wn2Open,Z2open,P2open] = damp(sys_2OL)
figure 
rlocus(sys_2OL);
hold on 


Kq = 1;
sys_cl2 = feedback(sys_cl*Kq*sys2,1)
[Wn2,Z2,P2] = damp(sys_cl2)
figure 
rlocus(sys_cl2);
hold on 
figure
step(sys_cl2); 
stepinfo(sys_cl2)
