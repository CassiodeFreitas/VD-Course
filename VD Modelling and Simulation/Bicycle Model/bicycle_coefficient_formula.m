%% Constants
% Gravity
g = 9.81;

%% Setup
V = 120; %kph
SA = 11; %deg
slope = 2;
simulation_time = 20; %s
line = 'r';

%% Vehicle Parameters 
%Mass
m = 850;
%Intertial
I = 1500;
% Wheelbase
L = 3.5;
% CoG Position
a = 1.8;
b = L-a;
% Steering Ratio
SR = 10;
%% Aero Parameters
% Frontal Area
A = 1.25;
% Downforce Coefficient
cz = 1;
% Air density
rho = 1.225;

%% Rear Tyre Parameters
% Axle Cornering Stiffness
% CaR = -7000;
% % Maximum Force
% ymR = -1000;
% % Slip @ Maximum Force
% xmR = 3;
% % Horizontal Asimptote
% yaR = -2500;
% Shape Factor
CR = 1.7631;
% Stiffness Factor
BR = 0.3609;
% Curvature Factor
ER = -1.989;
% Peak Value
a1R = -1e-5;
a2R = 1.25;

%% Front Tyre Parameters
% Axle Cornering Stiffness
% CaF = -5000;
% % Maximum Force
% ymF = -1000;
% % Slip @ Maximum Force
% xmF = 3;
% % Horizontal Asimptote
% yaF = -2500;
% Shape Factor
CF = 1.8391;
% Stiffness Factor
BF = 0.2719;
% Curvature Factor
EF = -2.5276;
% Peak Value
a1F = -1e-5;
a2F = 1.25;

save('formulacar_params.mat')