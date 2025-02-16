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
% z_r_type = questdlg('Which road input type would you like to apply?','Choose Road Input','Step Input','Chirp Input','Cancel');
% switch z_r_type
%     case "Step Input"
        z_r_type = 0;
        simulation_time = 5;
%     case "Chirp Input"
%         z_r_type = 1;
%         simulation_time = 100;
% end
sim('quarter_car_model.slx',simulation_time)

%% Run FFT
dt = 1e-3;
[amp_s,phase_s,freq_s] = fft_VD(z_s_dotdot.data,dt);
[amp_us,phase_us,freq_us] = fft_VD(z_us_dotdot.data,dt);

%% Post Simulation Results
% Ride Frequency [Hz]
f_ride = (2*pi)^-1*(k_spring/m_s)^(1/2);
% Wheel Hub Frequency [Hz]
f_hub = (2*pi)^-1*((k_tyre+k_spring)/m_us)^(1/2);
% Critical Damping [Ns/m]
c_critical = 2*(k_spring*m_s)^(1/2);
% Damping Ratio [-]
c_ratio = c_damper/c_critical;

%% Plot Figures
figure
tab1 = uitab("Title","Plots");
axes(tab1)
subplot(3,2,1)
plot(z_s,'r','LineWidth',1.5,'DisplayName','Sprung Mass')
hold on
plot(z_us,'b','LineWidth',1.5,'DisplayName','Unsprung Mass')
hold on
plot(z_r,'black--','LineWidth',1.5,'DisplayName','Surface')
title('Sprung and Unsprung Masses Vertical Displacements')
xlabel('Time [s]')
ylabel('Vertical Displacement [m]')
lgd = legend;
grid on
subplot(3,2,3)
plot(z_s_dot,'r','LineWidth',1.5,'DisplayName','Sprung Mass')
hold on
plot(z_us_dot,'b','LineWidth',1.5,'DisplayName','Unsprung Mass')
title('Sprung and Unsprung Masses Vertical Velocities')
xlabel('Time [s]')
ylabel('Vertical Velocity [m/s]')
lgd = legend;
grid on
subplot(3,2,5)
plot(z_s_dotdot,'r','LineWidth',1.5,'DisplayName','Sprung Mass')
hold on
plot(z_us_dotdot,'b','LineWidth',1.5,'DisplayName','Unsprung Mass')
title('Sprung and Unsprung Masses Vertical Accelerations')
xlabel('Time [s]')
ylabel('Vertical Acceleration [m/s^2]')
lgd = legend;
grid on
subplot(3,2,2)
plot(F_tyre,'black','LineWidth',1.5,'DisplayName','Tyre')
title('Tyre Force')
xlabel('Time [s]')
ylabel('Tyre Force [N]')
lgd = legend;
grid on
subplot(3,2,4)
plot(F_spring,'black','LineWidth',1.5,'DisplayName','Spring')
title('Spring Force')
xlabel('Time [s]')
ylabel('Spring Force [N]')
lgd = legend;
grid on
subplot(3,2,6)
plot(F_damper,'black','LineWidth',1.5,'DisplayName','Damper')
title('Damper Force')
xlabel('Time [s]')
ylabel('Damper force [N]')
lgd = legend;
grid on

tab2 = uitab("Title","FFT");
axes(tab2)
subplot(1,1,1)
plot(freq_s,amp_s,'r','LineWidth',1.5,'DisplayName','Sprung Mass')
hold on
plot(freq_us,amp_us,'b','LineWidth',1.5,'DisplayName','Unsprung Mass')
title('Sprung Mass Vertical Acceleration')
xlabel('Frequency [Hz]')
ylabel('Vertical Acceleration [m/s^2]')
lgd = legend;
xlim([ 0 30])
grid on

% fprintf('z_us_dotdot = %f',z_us_dotdot.data(1))