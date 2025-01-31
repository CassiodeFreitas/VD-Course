clc
clear all
addpath(genpath('C:\Users\cassi\Documents\VD-Course'));

%% Constants
g = 9.81;

%% Setup
V0 = 300;

%% Vehicle Parameters
m = 850;
h = 0.2;
L = 3.5;
a = 1.8;
b = L-a;

%% Aero Parameters
A = 1.25;
cz = 1;
cd = 0.4;
rho = 1.225;

%% Brake System Parameters
r = 0.375;
Iwheel = 0.9;
rbrake = 3800;
balance = 0.8;

%% Tyre Parameters;
C = 1.7180;
B = 4.1576;
E = -22.1100;
a1 = -1e-5;
a2 = 1.25;
 
%% Run Simulation
sim('brake_model',3)
figure(1)
plot(longacc)
figure(2)
plot(V)
figure(3)
plot(SRF)
hold on
plot(SRR)