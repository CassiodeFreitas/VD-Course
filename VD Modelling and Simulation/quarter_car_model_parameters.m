clc
clear all

%% Constants
% Gravity [m/s^2]
g = 9.81;

%% Vehicle Parameters
% Sprung mass [kg]
m_s = 400;
% Unsprung Mass [kg]
m_us = 30;

%% Suspension Parameters
% Spring Stiffness [N/m] (f_ride = 1.50 Hz)
k_spring = 35500;
% Spring Preload [N]
F0 = (m_s * g)-1814.85;
% Damping Coefficient [Ns/m] (c_ratio = 0.2)
c_damper = 1500;

%% Tyre Parameters
% Tyre Vertical Stiffness [N/m] (F_wheel_hub = 16.83 Hz)
k_tyre = 300000;
% Tyre Deflection @ Equilibrium
z0_tyre = (m_s + m_us) * g / k_tyre;

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
sim('quarter_car_model.slx',simulation_time)

%% Run FFT
dt = 1e-3;
[amp1,phase1,freq1] = fft_VD(z_s_dotdot.data,dt);
[amp2,phase2,freq2] = fft_VD(z_us_dotdot.data,dt);
%% Plot Figures
% figure(1)
% plot(z_s,'r','LineWidth',1.5,'DisplayName','Sprung Mass')
% hold on
% plot(z_us,'b','LineWidth',1.5,'DisplayName','Unsprung Mass')
% title('Sprung and Unsprung Masses Vertical Displacements')
% xlabel('Time [s]')
% ylabel('Vertical Displacement [m]')
% lgd = legend;
% grid on

figure(2)
plot(freq1,amp1,'r','LineWidth',1.5,'DisplayName','Sprung Mass')
hold on
plot(freq2,amp2,'b','LineWidth',1.5,'DisplayName','Unsprung Mass')
title('Sprung Mass Vertical Acceleration')
xlabel('Frequency [Hz]')
ylabel('Vertical Acceleration [m/s^2]')
lgd = legend;
xlim([ 0 30])
grid on

% fprintf('z_us_dotdot = %f',z_us_dotdot.data(1))