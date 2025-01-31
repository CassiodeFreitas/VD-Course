clc
clear all
addpath(genpath('C:\Users\cassi\Documents\VD-Course'));

%% Constants
% Gravity
g = 9.81;

%% Vehicle Parameters 
%Mass
m = 850;
% Wheelbase
L = 3.5;
% CoG Position
h = 0.2;
a = 1.8;
b = L-a;

%% Aero Parameters
% Frontal Area
A = 1.25;
% Downforce Coefficient
cz = 1;
% Drag Coefficient
cd = 0.4;
% Air density
rho = 1.225;

%% Powertrain Parameters
% Tyre Radius
r = 0.375;
% Wheel Inertia
Iwheel = 0.9;
% Gear Ratios
igear = [23.10, 17.64, 13.86, 10.92, 8.82, 7.13, 5.88, 4.33]';
% Maximum Enginer Torque
Tengmax = 250;
% Throttle Map
throttlepos = [0, 0.25, 0.5, 0.75, 1]';
% Engine RPM 
enginerpm = linspace(1000,15000,15)';
% Engine Map
enginemap = [-0.15, 0.23, 0.60, 0.68, 0.8;
    -0.17, 0.26, 0.62, 0.69, 0.82;
    -0.2, 0.23, 0.63, 0.71, 0.86;
    -0.3, 0.2, 0.62, 0.72, 0.89;
    -0.4, 0.18, 0.61, 0.73, 0.91;
    -0.5, 0.17, 0.58, 0.74, 0.93;
    -0.58, 0.16, 0.54, 0.76, 0.95;
    -0.68, 0.15, 0.50, 0.78, 0.98;
    -0.75, 0.10, 0.45, 0.77, 1.00;
    -0.83, 0.06, 0.40, 0.76, 1.00;
    -0.93, -0.03, 0.32, 0.70, 0.98;
    -1.02, -0.08, 0.23, 0.62, 0.96;
    -1.10, -0.15, 0.16, 0.52, 0.93;
    -1.20, -0.20, 0.08, 0.42, 0.88;
    -1.25, -0.30, 0.00, 0.034, 0.84];
% Idle RPM
rpm_idling = 1000;
% RPM Limit
rpm_limit = 15000;
% RPM Shift Up
rpm_up = 13500;
% RPM Shift Down
rpm_down = 7000;

%% Rear Tyre Parameters
% Shape Factor
CR = 1.2421;
% Stiffness Factor
BR = 5.7507;
% Curvature Factor
ER = -6.8251;
% Peak Value
a1R = -1e-5;
a2R = 1.25;

%% Front Tyre Parameters
% Shape Factor
CF = 1.8391;
% Stiffness Factor
BF = 0.2719;
% Curvature Factor
EF = -2.5276;
% Peak Value
a1F = -1e-5;
a2F = 1.25;

%% Run Simulation
% Initial Gear
i_initial = 5;
% Initial Velocity
V0 = 120;
% Throttle Position
TPS = 1;
sim('driving_dynamics_model',20)

plot(V)