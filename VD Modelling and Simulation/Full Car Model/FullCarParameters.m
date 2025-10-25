% xCoG
% yCoG
% zCoG
% Roll
% Pitch
% Yaw
% zFL
% zFR
% zRL
% zRR
% aSteer
% Engine Model
% Brake Model
% Wheel Rotational Speeds
% Circuit Model (xyz)
% Driver Model
% Ackerman geometry
% Toe Alignment
% Pacejka Lateral Tyre Model
% Pacejka Longitudinal Tyre Model

clc
clear all

%% TO DO 
% Create surface characterisation distance based;

%% Constants
% Gravity [m/s^2]
constants.g = 9.81;
%% Driver Inputs
% Constant Velocity [m/s]
driver.V = 20;
% Constant Lateral Acceleration [m/s^2]
driver.ay = 0;
% Steering Angle [deg]
driver.SA = 11;
%% Surface Inputs
% Simulation Time
simulation_time = 5;
% Step size [s]
dt = 5e-3;
% Surface time [s]
road.z_r_time = [0:dt:simulation_time];
% Surface Data
road.z_r_data = zeros(length(road.z_r_time),1);
% Create Surface Profile Timeseries
road.z_r = timeseries(road.z_r_data,road.z_r_time);

% Edit Surface Timeseries
% Add a step_z m step bump @ step_t s;
for i = 1:length(road.z_r.time)
    road.step_z = 0.0;
    road.step_t = 1;
    if road.z_r.time(i) >= road.step_t
        road.z_r.data(i) = road.step_z;
    end
end
% Surface Distance [m]
road.x_r_distance = (road.z_r_time * driver.V)';
% Create Surface Distance Timeseries
road.x_r = timeseries(road.x_r_distance,road.z_r_time);
%% Chassis Parameters
% Wheelbase [m]
chassis.L = 3.5;
% Track Width [m]
chassis.tf = 1.75;
chassis.tr = 1.85;
% CoG Position [m]
chassis.a = 1.8;
chassis.b = 1.7;
chassis.h = 0.2;
% Total mass [kg]
chassis.m = 850;
% Unsprung Mass [kg]
chassis.m_usFL = 15;
chassis.m_usFR = chassis.m_usFL;
chassis.m_usRL = 17.5;
chassis.m_usRR = chassis.m_usRL;
% Sprung mass [kg]
chassis.m_s = 850 - chassis.m_usFL - chassis.m_usFR - chassis.m_usRL- chassis.m_usRR;
% Moment of Inertia [kg*m^2]
chassis.Iyy = 1350;
chassis.Ixx = 300;
% Tyre Nominal Radius [m]
r = 0.375;
% Roll Centre Height [m]
chassis.hrcF = 0.04;
chassis.hrcR = 0.06;
chassis.hrcCoG = (chassis.hrcF + chassis.hrcR)/2;
chassis.hrcusF = r;
chassis.hrcusR = r;
%% Aero Parameters
% Frontal Area [m^2]
aero.Af = 1.25;
% Drag Coefficient [-]
aero.Cx = 0;
% Downforce Coefficient [-]
aero.Cz = 1;
% Air Density [kg/m^3]
aero.rho = 1.225;
% Aero Balance
aero.a = chassis.a;
aero.b = chassis.b;
%% Suspension Parameters
% Spring Stiffness [N/m] (f_rideF = 3.00 Hz & f_rideR = 3.60 Hz)
suspension.k_springFL = 67500;
suspension.k_springFR = suspension.k_springFL;
suspension.k_springRL = 103000;
suspension.k_springRR = suspension.k_springRL;
% Spring Preload [N]
suspension.F0FL = (chassis.m_s/2 * (chassis.b / chassis.L) * constants.g);
suspension.F0FR = suspension.F0FL;
suspension.F0RL = (chassis.m_s/2 * (chassis.a / chassis.L) * constants.g);
suspension.F0RR = suspension.F0RL;
% Damping Coefficient [Ns/m] (c_ratio = 0.2)
suspension.c_damperFL = 1440;
suspension.c_damperFR = suspension.c_damperFL;
suspension.c_damperRL = 1825;
suspension.c_damperRR = suspension.c_damperRL;
% Anti Roll Bar Stiffness [Nm/rad]
suspension.kARBf = 5000; %5000;
suspension.kARBr = 1000; %1000;
%% Steering Parameters
% Steering Ratio [-]
steering.SR = 10;
%% Front Tyres Parameters
% Tyre Vertical Stiffness [N/m] (F_wheel_hub = 16.83 Hz)
suspension.k_tyreFL = 200000;
suspension.k_tyreFR = suspension.k_tyreFL;
% Tyre Deflection @ Equilibrium
suspension.z0_tyreFL = (chassis.m_s/2 * chassis.b / chassis.L + chassis.m_usFL) * constants.g / suspension.k_tyreFL;
suspension.z0_tyreFR = (chassis.m_s/2 * chassis.b / chassis.L + chassis.m_usFR) * constants.g / suspension.k_tyreFR;
% Shape Factor
suspension.CF = 1.8391;
% Stiffness Factor
suspension.BF = 0.2719;
% Curvature Factor
suspension.EF = -2.5276;
% Peak Value
suspension.a1F = -1e-5;
suspension.a2F = 1.25;
%% Rear Tyres Parameters
% Tyre Vertical Stiffness [N/m] (F_wheel_hub = 16.83 Hz)
suspension.k_tyreRL = suspension.k_tyreFL;
suspension.k_tyreRR = suspension.k_tyreRL;
% Tyre Deflection @ Equilibrium
suspension.z0_tyreRL = (chassis.m_s/2 * chassis.a / chassis.L + chassis.m_usRL) * constants.g / suspension.k_tyreRL;
suspension.z0_tyreRR = (chassis.m_s/2 * chassis.a / chassis.L + chassis.m_usRR) * constants.g / suspension.k_tyreRR;
% Shape Factor
suspension.CR = 1.7631;
% Stiffness Factor
suspension.BR = 0.3609;
% Curvature Factor
suspension.ER = -1.989;
% Peak Value
suspension.a1R = -1e-5;
suspension.a2R = 1.25;
%% Run Simulation
z_r_type = questdlg('Which road input type would you like to apply?','Choose Road Input','Step Input','Chirp Input','Cancel');
switch z_r_type
    case "Step Input"
        z_r_type = 0;
        simulation_time = 5;
    case "Chirp Input"
        z_r_type = 1;
        simulation_time = 100;
end
simout = sim('FullCarModel.slx',simulation_time);

%% Run FFT
dt = 5e-3;
[fft.amp_s,fft.phase_s,fft.freq_s] = fft_VD(simout.gVert.data,dt);
[fft.amp_usFL,fft.phase_usFL,fft.freq_usFL] = fft_VD(simout.gVertHubFL.data,dt);
[fft.amp_usFR,fft.phase_usFR,fft.freq_usFR] = fft_VD(simout.gVertHubFR.data,dt);
[fft.amp_usRL,fft.phase_usRL,fft.freq_usRL] = fft_VD(simout.gVertHubRL.data,dt);
[fft.amp_usRR,fft.phase_usRR,fft.freq_usRR] = fft_VD(simout.gVertHubRR.data,dt);

%% Post Simulation Results
% Ride Frequency Front Left [Hz]
postsim.f_rideFL = (2*pi)^-1*(suspension.k_springFL/(chassis.m_s/2 * (chassis.b / chassis.L)))^(1/2);
% Wheel Hub Frequency Front Left [Hz]
postsim.f_hubFL = (2*pi)^-1*((suspension.k_tyreFL+suspension.k_springFL)/chassis.m_usFL)^(1/2);
% Critical Damping Front Left [Ns/m]
postsim.c_criticalFL = 2*(suspension.k_springFL*(chassis.m_s/2 * (chassis.b / chassis.L)))^(1/2);
% Damping Ratio Front Left [-]
postsim.c_ratioFL = suspension.c_damperFL/postsim.c_criticalFL;

% Ride Frequency Rear Left [Hz]
postsim.f_rideRL = (2*pi)^-1*(suspension.k_springRL/(chassis.m_s/2 * (chassis.a / chassis.L)))^(1/2);
% Wheel Hub Frequency Rear Left [Hz]
postsim.f_hubRL = (2*pi)^-1*((suspension.k_tyreRL+suspension.k_springRL)/chassis.m_usRL)^(1/2);
% Critical Damping Rear Left [Ns/m]
postsim.c_criticalRL = 2*(suspension.k_springRL*(chassis.m_s/2 * (chassis.a / chassis.L)))^(1/2);
% Damping Ratio Rear Left [-]
postsim.c_ratioRL = suspension.c_damperRL/postsim.c_criticalRL;

%% Plot Figures
clf

figure(1)

tab1 = uitab("Title","Plots");

axes(tab1)

subplot(3,2,1)
plot(simout.zCar,'r','LineWidth',1.5,'DisplayName','zCar')
hold on
plot(simout.zHubFL,'b','LineWidth',1.5,'DisplayName','zHubFL')
hold on
plot(simout.zHubFR,'b--','LineWidth',1.5,'DisplayName','zHubFR')
hold on
plot(simout.zHubRL,'g','LineWidth',1.5,'DisplayName','zHubRL')
hold on
plot(simout.zHubRR,'g--','LineWidth',1.5,'DisplayName','zHubRR')
hold on
plot(simout.zTrackFL,'black','LineWidth',1.5,'DisplayName','zTrackFL')
hold on
plot(simout.zTrackRL,'black--','LineWidth',1.5,'DisplayName','zTrackRL')
title('Sprung and Unsprung Masses Vertical Displacements')
xlabel('Time [s]')
ylabel('Vertical Displacement [m]')
ylim([0 0.2])
lgd = legend;
grid on

subplot(3,2,2)
plot(simout.FzTyreFL,'b','LineWidth',1.5,'DisplayName','FzTyreFL')
hold on
plot(simout.FzTyreFR,'b--','LineWidth',1.5,'DisplayName','FzTyreFR')
hold on
plot(simout.FzTyreRL,'g','LineWidth',1.5,'DisplayName','FzTyreRL')
hold on
plot(simout.FzTyreRR,'g--','LineWidth',1.5,'DisplayName','FzTyreRR')
title('Tyre Forces')
xlabel('Time [s]')
ylabel('Tyre Force [N]')
ylim([-5e4 5e4])
lgd = legend;
grid on

subplot(3,2,3)
plot(simout.vzCar,'r','LineWidth',1.5,'DisplayName','vzCar')
hold on
plot(simout.vHubFL,'b','LineWidth',1.5,'DisplayName','vHubFL')
hold on
plot(simout.vHubFR,'b--','LineWidth',1.5,'DisplayName','vHubFR')
hold on
plot(simout.vHubRL,'g','LineWidth',1.5,'DisplayName','vHubRL')
hold on
plot(simout.vHubRR,'g--','LineWidth',1.5,'DisplayName','vHubRR')
title('Sprung and Unsprung Masses Vertical Velocities')
xlabel('Time [s]')
ylabel('Vertical Velocity [m/s]')
ylim([-10 10])
lgd = legend;
grid on

subplot(3,2,4)
plot(simout.FSpringFL,'b','LineWidth',1.5,'DisplayName','FSpringFL')
hold on
plot(simout.FSpringFR,'b--','LineWidth',1.5,'DisplayName','FSpringFR')
hold on
plot(simout.FSpringRL,'g','LineWidth',1.5,'DisplayName','FSpringRL')
hold on
plot(simout.FSpringRR,'g--','LineWidth',1.5,'DisplayName','FSpringRR')
title('Spring Forces')
xlabel('Time [s]')
ylabel('Spring Force [N]')
ylim([-1e4 1e4])
lgd = legend;
grid on

subplot(3,2,5)
plot(simout.gVert,'r','LineWidth',1.5,'DisplayName','Sprung Mass')
hold on
plot(simout.gVertHubFL,'b','LineWidth',1.5,'DisplayName','gVertHubFL')
hold on
plot(simout.gVertHubFR,'b--','LineWidth',1.5,'DisplayName','gVertHubFR')
hold on
plot(simout.gVertHubRL,'g','LineWidth',1.5,'DisplayName','gVertHubRL')
hold on
plot(simout.gVertHubRR,'g--','LineWidth',1.5,'DisplayName','gVertHubRR')
title('Sprung and Unsprung Masses Vertical Accelerations')
xlabel('Time [s]')
ylabel('Vertical Acceleration [m/s^2]')
ylim([-1.5e3 1.5e3])
lgd = legend;
grid on

subplot(3,2,6)
plot(simout.FDamperFL,'b','LineWidth',1.5,'DisplayName','FDamperFL')
hold on
plot(simout.FDamperFR,'b--','LineWidth',1.5,'DisplayName','FDamperFR')
hold on
plot(simout.FDamperRL,'g','LineWidth',1.5,'DisplayName','FDamperRL')
hold on
plot(simout.FDamperRR,'g--','LineWidth',1.5,'DisplayName','FDamperRR')
title('Damper Forces')
xlabel('Time [s]')
ylabel('Damper Force [N]')
ylim([-1.5e4 1.5e4])
lgd = legend;
grid on

tab2 = uitab("Title","Aero Forces");
axes(tab2)
subplot(2,1,1)
plot(simout.vCar.data,simout.FSpringFL.data,'b','LineWidth',1.5,'DisplayName','FSpringFL')
hold on
plot(simout.vCar.data,simout.FSpringFR.data,'b--','LineWidth',1.5,'DisplayName','FSpringFR')
hold on
plot(simout.vCar.data,simout.FSpringRL.data,'g','LineWidth',1.5,'DisplayName','FSpringRL')
hold on
plot(simout.vCar.data,simout.FSpringRR.data,'g--','LineWidth',1.5,'DisplayName','FSpringRR')
title('Spring Forces')
xlabel('vCar [m/s]')
ylabel('Spring Force [N]')
xlim([0 28])
ylim([1500 2500])
lgd = legend;
grid on

subplot(2,1,2)
plot(simout.vCar.data,simout.FzAeroF.data,'b','LineWidth',1.5,'DisplayName','FzAeroF')
hold on
plot(simout.vCar.data,simout.FzAeroR.data,'g','LineWidth',1.5,'DisplayName','FzAeroR')
hold on
plot(simout.vCar.data,simout.FzAeroTotal.data,'r','LineWidth',1.5,'DisplayName','FzAeroTotal')
title('Aero Forces')
xlabel('vCar [m/s]')
ylabel('Downforce [N]')
xlim([0 28])
ylim([0 1000])
lgd = legend;
grid on

tab3 = uitab("Title","Lateral Load Transfer");
axes(tab3)
subplot(3,1,1)
plot(simout.FSpringFL,'b','LineWidth',1.5,'DisplayName','FSpringFL')
hold on
plot(simout.FSpringFR,'b--','LineWidth',1.5,'DisplayName','FSpringFR')
hold on
plot(simout.FSpringRL,'g','LineWidth',1.5,'DisplayName','FSpringRL')
hold on
plot(simout.FSpringRR,'g--','LineWidth',1.5,'DisplayName','FSpringRR')
title('Spring Forces')
xlabel('Time [s]')
ylabel('Spring Force [N]')
ylim([1500 2500])
lgd = legend;
grid on

subplot(2,1,2)
plot(simout.gLat,'r','LineWidth',1.5,'DisplayName','Lateral Acceleration')
title('Lateral Acceleration')
xlabel('Time [s]')
ylabel('Lateral Acceleration [m/s^2]')
ylim([0 4])
lgd = legend;
grid on

tab4 = uitab("Title","FFT");
axes(tab4)
subplot(1,1,1)
plot(fft.freq_s,fft.amp_s,'r','LineWidth',1.5,'DisplayName','Sprung Mass')
hold on
plot(fft.freq_usFL,fft.amp_usFL,'b','LineWidth',1.5,'DisplayName','FL')
hold on
plot(fft.freq_usFR,fft.amp_usFR,'b--','LineWidth',1.5,'DisplayName','FR')
hold on
plot(fft.freq_usRL,fft.amp_usRL,'g','LineWidth',1.5,'DisplayName','RL')
hold on
plot(fft.freq_usRL,fft.amp_usRR,'g--','LineWidth',1.5,'DisplayName','RR')
title('Mass Vertical Acceleration')
xlabel('Frequency [Hz]')
ylabel('Vertical Acceleration [m/s^2]')
lgd = legend;
xlim([0 50])
grid on