clc
clear all

%% TO DO 
% Create surface characterisation distance based;

%% Constants
% Gravity [m/s^2]2
constants.g = 9.81;

%% Vehicle Parameters
% Wheelbase [m]
chassis.L = 3.5;
% CoG Position [m]
chassis.a = 1.8;
chassis.b = 1.7;
% Total mass [kg]
chassis.m = 850;
% Unsprung Mass [kg]
chassis.m_usF = 30;
chassis.m_usR = 35;
% Sprung mass [kg]
chassis.m_s = 850 - chassis.m_usF - chassis.m_usR;
% Constant Velocity [m/s]
chassis.V = 20;

%% Suspension Parameters
% Spring Stiffness [N/m] (f_rideF = 3.00 Hz & f_rideR = 3.60 Hz)
suspension.k_springF = 135000;
suspension.k_springR = 206000;
% Spring Preload [N]
% F0F = 4107.1 - 366.68715;
suspension.F0F = chassis.m_s * (chassis.b / chassis.L) * constants.g;
% F0R = 3593.7 + 317.69;
suspension.F0R = chassis.m_s * (chassis.a / chassis.L) * constants.g;
% Damping Coefficient [Ns/m] (c_ratio = 0.2)
suspension.c_damperF = 2880;
suspension.c_damperR = 3650;
% Moment of Inertia [kg*m^2]
chassis.I = 1350;

%% Tyre Parameters
% Tyre Vertical Stiffness [N/m] (F_wheel_hub = 16.83 Hz)
suspension.k_tyreF = 400000;
suspension.k_tyreR = suspension.k_tyreF;
% Tyre Deflection @ Equilibrium
suspension.z0_tyreF = (chassis.m_s * chassis.b / chassis.L + chassis.m_usF) * constants.g / suspension.k_tyreF;
suspension.z0_tyreR = (chassis.m_s * chassis.a / chassis.L + chassis.m_usR) * constants.g / suspension.k_tyreR;

%% Surface Characterisation
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
    road.step_z = 0.1;
    road.step_t = 1;
    if road.z_r.time(i) >= road.step_t
        road.z_r.data(i) = road.step_z;
    end
end
% Surface Distance [m]
road.x_r_distance = (road.z_r_time * chassis.V)';
% Create Surface Distance Timeseries
road.x_r = timeseries(road.x_r_distance,road.z_r_time);

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
simout = sim('DOF4_model_v3.slx',simulation_time);

%% Run FFT
dt = 5e-3;
[fft.amp_s,fft.phase_s,fft.freq_s] = fft_VD(simout.z_s_dotdot.data,dt);
[fft.amp_usF,fft.phase_usF,fft.freq_usF] = fft_VD(simout.z_us_dotdotF.data,dt);
[fft.amp_usR,fft.phase_usR,fft.freq_usR] = fft_VD(simout.z_us_dotdotR.data,dt);

%% Post Simulation Results
% Ride Frequency Front [Hz]
postsim.f_rideF = (2*pi)^-1*(suspension.k_springF/(chassis.m_s * (chassis.b / chassis.L)))^(1/2);
% Wheel Hub Frequency Front [Hz]
postsim.f_hubF = (2*pi)^-1*((suspension.k_tyreF+suspension.k_springF)/chassis.m_usF)^(1/2);
% Critical Damping Front [Ns/m]
postsim.c_criticalF = 2*(suspension.k_springF*(chassis.m_s * (chassis.b / chassis.L)))^(1/2);
% Damping Ratio Front [-]
postsim.c_ratioF = suspension.c_damperF/postsim.c_criticalF;

% Ride Frequency Rear [Hz]
postsim.f_rideR = (2*pi)^-1*(suspension.k_springR/(chassis.m_s * (chassis.a / chassis.L)))^(1/2);
% Wheel Hub Frequency Rear [Hz]
postsim.f_hubR = (2*pi)^-1*((suspension.k_tyreR+suspension.k_springR)/chassis.m_usR)^(1/2);
% Critical Damping Rear [Ns/m]
postsim.c_criticalR = 2*(suspension.k_springR*(chassis.m_s * (chassis.a / chassis.L)))^(1/2);
% Damping Ratio Rear [-]
postsim.c_ratioR = suspension.c_damperR/postsim.c_criticalR;

%% Plot Figures
clf
figure(1)
tab1 = uitab("Title","Plots");
axes(tab1)
subplot(3,2,1)
plot(simout.z_s,'r','LineWidth',1.5,'DisplayName','Sprung Mass')
hold on
plot(simout.z_usF,'b','LineWidth',1.5,'DisplayName','Front Unsprung Mass')
hold on
plot(simout.z_usR,'g','LineWidth',1.5,'DisplayName','Rear Unsprung Mass')
hold on
plot(simout.z_rF,'b--','LineWidth',1.5,'DisplayName','Front Surface')
hold on
plot(simout.z_rR,'g--','LineWidth',1.5,'DisplayName','Rear Surface')
title('Sprung and Unsprung Masses Vertical Displacements')
xlabel('Time [s]')
ylabel('Vertical Displacement [m]')
ylim([0 0.2])
lgd = legend;
grid on
subplot(3,2,2)
plot(simout.F_tyreF,'b','LineWidth',1.5,'DisplayName','Front Tyre')
hold on
plot(simout.F_tyreR,'g','LineWidth',1.5,'DisplayName','Rear Tyre')
title('Tyre Forces')
xlabel('Time [s]')
ylabel('Tyre Force [N]')
lgd = legend;
grid on
ylim([-5e4 5e4])
subplot(3,2,3)
plot(simout.z_s_dot,'r','LineWidth',1.5,'DisplayName','Sprung Mass')
hold on
plot(simout.z_us_dotF,'b','LineWidth',1.5,'DisplayName','Front Unsprung Mass')
hold on
plot(simout.z_us_dotR,'g','LineWidth',1.5,'DisplayName','Rear Unsprung Mass')
title('Sprung and Unsprung Masses Vertical Velocities')
xlabel('Time [s]')
ylabel('Vertical Velocity [m/s]')
lgd = legend;
grid on
ylim([-10 10])
subplot(3,2,4)
plot(simout.F_springF,'b','LineWidth',1.5,'DisplayName','Front Spring')
hold on
plot(simout.F_springR,'g','LineWidth',1.5,'DisplayName','Rear Spring')
title('Spring Forces')
xlabel('Time [s]')
ylabel('Spring Force [N]')
lgd = legend;
grid on
ylim([-1e4 1e4])
subplot(3,2,5)
plot(simout.z_s_dotdot,'r','LineWidth',1.5,'DisplayName','Sprung Mass')
hold on
plot(simout.z_us_dotdotF,'b','LineWidth',1.5,'DisplayName','Front Unsprung Mass')
hold on
plot(simout.z_us_dotdotR,'g','LineWidth',1.5,'DisplayName','Rear Unsprung Mass')
title('Sprung and Unsprung Masses Vertical Accelerations')
xlabel('Time [s]')
ylabel('Vertical Acceleration [m/s^2]')
lgd = legend;
grid on
ylim([-1.5e3 1.5e3])
subplot(3,2,6)
plot(simout.F_damperF,'b','LineWidth',1.5,'DisplayName','Front Damper')
hold on
plot(simout.F_damperR,'g','LineWidth',1.5,'DisplayName','Rear Damper')
title('Damper Forces')
xlabel('Time [s]')
ylabel('Damper force [N]')
lgd = legend;
grid on
ylim([-1.5e4 1.5e4])

tab2 = uitab("Title","FFT");
axes(tab2)
subplot(1,1,1)
plot(fft.freq_s,fft.amp_s,'r','LineWidth',1.5,'DisplayName','Sprung Mass')
hold on
plot(fft.freq_usF,fft.amp_usF,'b','LineWidth',1.5,'DisplayName','Unsprung Mass Front')
plot(fft.freq_usR,fft.amp_usR,'g','LineWidth',1.5,'DisplayName','Unsprung Mass Rear')
title('Sprung Mass Vertical Acceleration')
xlabel('Frequency [Hz]')
ylabel('Vertical Acceleration [m/s^2]')
lgd = legend;
xlim([ 0 50])
grid on

% fprintf('z_us_dotdot = %f /n',z_us_dotdot.data(1))