clc
clear all

%% TO DO
% Replace variable names by structs identifying each subgroup i.e.
% parameters.vehicle.wheelbase
% Check x_r data types;
% Create surface characterisation distance based;

%% Constants
% Gravity [m/s^2]
g = 9.81;

%% Vehicle Parameters
% Wheelbase [m]
L = 3.5;
% CoG Position [m]
a = 1.8;
b = 1.7;
% Sprung mass [kg]
m_s = 785;
% Unsprung Mass [kg]
m_usF = 30;
m_usR = 35;
% Total mass [kg]
m = m_s + m_usF + m_usR;
% Constant Velocity [m/s]
V = 20;

%% Suspension Parameters
% Spring Stiffness [N/m] (f_ride = 1.50 Hz)
k_springF = 37000;
k_springR = 47000;
% Spring Preload [N]
% F0F = 4107.1 - 366.68715;
F0F = m_s * (b / L) * g;
% F0R = 3593.7 + 317.69;
F0R = m_s * (a / L) * g;
% Damping Coefficient [Ns/m] (c_ratio = 0.2)
c_damperF = 1580;
c_damperR = 1650;
% Moment of Inertia [kg*m^2]
I = 1350;

%% Tyre Parameters
% Tyre Vertical Stiffness [N/m] (F_wheel_hub = 16.83 Hz)
k_tyreF = 400000;
k_tyreR = k_tyreF;
% Tyre Deflection @ Equilibrium
z0_tyreF = (m_s*b/L + m_usF) * g / k_tyreF;
z0_tyreR = (m_s*a/L + m_usF) * g / k_tyreR;

%% Surface Characterisation
% Simulation Time
simulation_time = 5;
% Step size [s]
dt = 1e-3;
% Surface time [s]
z_r_time = [0:dt:simulation_time];
% Surface Data
z_r_data = zeros(length(z_r_time),1);
% Create Surface Profile Timeseries
z_r = timeseries(z_r_data,z_r_time);

% Edit Surface Timeseries
% Add a step_z m step bump @ step_t s;
for i = 1:length(z_r.time)
    step_z = step_z;
    if z_r.time(i) >= step_t
        z_r.data(i) = step_z;
    end
end
% Surface Distance [m]
x_r_distance = z_r_time * V;
% Create Surface Distance Timeseries
x_r = timeseries(x_r_distance,z_r_time);

%% Run Simulation
% z_r_type = questdlg('Which road input type would you like to apply?','Choose Road Input','Step Input','Chirp Input','Cancel');
% switch z_r_type
%     case "Step Input"
        % z_r_type = 0;
        simulation_time = 5;
%     case "Chirp Input"
%         z_r_type = 1;
%         simulation_time = 100;
% end
sim('DOF4_model_v2.slx',simulation_time)

% %% Run FFT
% dt = 1e-3;
% [amp_s,phase_s,freq_s] = fft_VD(z_s_dotdot.data,dt);
% [amp_usF,phase_usF,freq_usF] = fft_VD(z_us_dotdotF.data,dt);
% [amp_usR,phase_usR,freq_usR] = fft_VD(z_us_dotdotR.data,dt);
% 
% %% Post Simulation Results
% % Ride Frequency [Hz]
% f_ride = (2*pi)^-1*(k_spring/m_s)^(1/2);
% % Wheel Hub Frequency [Hz]
% f_hub = (2*pi)^-1*((k_tyre+k_spring)/m_us)^(1/2);
% % Critical Damping [Ns/m]
% c_critical = 2*(k_spring*m_s)^(1/2);
% % Damping Ratio [-]
% c_ratio = c_damper/c_critical;
% 
%% Plot Figures
clf
figure(1)
tab1 = uitab("Title","Plots");
axes(tab1)
subplot(3,2,1)
plot(z_s,'r','LineWidth',1.5,'DisplayName','Sprung Mass')
hold on
plot(z_usF,'b','LineWidth',1.5,'DisplayName','Front Unsprung Mass')
hold on
plot(z_usR,'g','LineWidth',1.5,'DisplayName','Rear Unsprung Mass')
hold on
plot(z_rF,'b--','LineWidth',1.5,'DisplayName','Front Surface')
hold on
plot(z_rR,'g--','LineWidth',1.5,'DisplayName','Rear Surface')
title('Sprung and Unsprung Masses Vertical Displacements')
xlabel('Time [s]')
ylabel('Vertical Displacement [m]')
ylim([0 0.2])
lgd = legend;
grid on
subplot(3,2,2)
plot(F_tyreF,'b','LineWidth',1.5,'DisplayName','Front Tyre')
hold on
plot(F_tyreR,'g','LineWidth',1.5,'DisplayName','Rear Tyre')
title('Tyre Forces')
xlabel('Time [s]')
ylabel('Tyre Force [N]')
lgd = legend;
grid on
ylim([-5e4 5e4])
subplot(3,2,3)
plot(z_s_dot,'r','LineWidth',1.5,'DisplayName','Sprung Mass')
hold on
plot(z_us_dotF,'b','LineWidth',1.5,'DisplayName','Front Unsprung Mass')
hold on
plot(z_us_dotR,'g','LineWidth',1.5,'DisplayName','Rear Unsprung Mass')
title('Sprung and Unsprung Masses Vertical Velocities')
xlabel('Time [s]')
ylabel('Vertical Velocity [m/s]')
lgd = legend;
grid on
ylim([-10 10])
subplot(3,2,4)
plot(F_springF,'b','LineWidth',1.5,'DisplayName','Front Spring')
hold on
plot(F_springR,'g','LineWidth',1.5,'DisplayName','Rear Spring')
title('Spring Forces')
xlabel('Time [s]')
ylabel('Spring Force [N]')
lgd = legend;
grid on
ylim([-1e4 1e4])
subplot(3,2,5)
plot(z_s_dotdot,'r','LineWidth',1.5,'DisplayName','Sprung Mass')
hold on
plot(z_us_dotdotF,'b','LineWidth',1.5,'DisplayName','Front Unsprung Mass')
hold on
plot(z_us_dotdotR,'g','LineWidth',1.5,'DisplayName','Rear Unsprung Mass')
title('Sprung and Unsprung Masses Vertical Accelerations')
xlabel('Time [s]')
ylabel('Vertical Acceleration [m/s^2]')
lgd = legend;
grid on
ylim([-1.5e3 1.5e3])
subplot(3,2,6)
plot(F_damperF,'b','LineWidth',1.5,'DisplayName','Front Damper')
hold on
plot(F_damperR,'g','LineWidth',1.5,'DisplayName','Rear Damper')
title('Damper Forces')
xlabel('Time [s]')
ylabel('Damper force [N]')
lgd = legend;
grid on
ylim([-1.5e4 1.5e4])
% 
% tab2 = uitab("Title","FFT");
% axes(tab2)
% subplot(1,1,1)
% plot(freq_s,amp_s,'r','LineWidth',1.5,'DisplayName','Sprung Mass')
% hold on
% plot(freq_us,amp_us,'b','LineWidth',1.5,'DisplayName','Unsprung Mass')
% title('Sprung Mass Vertical Acceleration')
% xlabel('Frequency [Hz]')
% ylabel('Vertical Acceleration [m/s^2]')
% lgd = legend;
% xlim([ 0 30])
% grid on
% 
% % fprintf('z_us_dotdot = %f',z_us_dotdot.data(1))