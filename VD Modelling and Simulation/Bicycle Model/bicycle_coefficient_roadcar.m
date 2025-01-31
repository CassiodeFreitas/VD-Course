%% Constants
% Gravity
g = 9.81;

%% Setup
V = 120; %kph
SA = 18; %deg
slope = 2;
simulation_time = 40; %s
line = 'b--';

%% Vehicle Parameters 
%Mass
m = 1650;
%Intertial
I = 3500;
% Wheelbase
L = 2.8;
% CoG Position
a = 1.1;
b = L-a;
% Steering Ratio
SR = 20;
%% Aero Parameters
% Frontal Area
A = 1.8;
% Downforce Coefficient
cz = 0;
% Air density
rho = 1.225;

%% Rear Tyre Parameters
% Axle Cornering Stiffness
CaR = -700;
% Maximum Force
ymR = -1000;
% Slip @ Maximum Force
xmR = 3;
% Horizontal Asimptote
yaR = -2500;
% Shape Factor
CR = 1.2617;
% Stiffness Factor
BR = 0.0925;
% Curvature Factor
ER = -8.7012;
% Peak Value
a1R = -1e-5;
a2R = 0.95;

%% Front Tyre Parameters
% Axle Cornering Stiffness
CaF = -1000;
% Maximum Force
ymF = -1000;
% Slip @ Maximum Force
xmF = 3;
% Horizontal Asimptote
yaF = -2500;
% Shape Factor
CF = 1.2947;
% Stiffness Factor
BF = 0.0813;
% Curvature Factor
EF = -8.3966;
% Peak Value
a1F = -1e-5;
a2F = 0.95;

save('roadcar_params.mat')