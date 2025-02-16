clc
clear all

%% Choose Car & Load Parameters into Workspace
car_type = questdlg('Which car would you like to use?','Choose Car','Formula Car','Road Car','None');
switch car_type
    case "Formula Car"
        bicycle_coefficient_formula
        load('formulacar_params.mat')
        car_type = "Formula Car";
    case "Road Car"
        bicycle_coefficient_roadcar
        load('roadcar_params.mat')
        car_type = "Road Car";
end

%% Setup
input = questdlg('Which steering wheel input would you like to apply?','Choose Input','Step Input','Ramp','None');
switch input
    case "Step Input"
        input = 0;
        simulation_time = 10;
    case "Ramp"
        input = 1;
end
% simulation_time = 100;
% slope = 1; %deg/s
% line = 'green--';

%% Modify Parameters
% SA = 10;
% V = 90;
% L = 3.5;
% a = 2;
% b = L-a;
% CF = 1.7631;
% BF = 0.3609;
% EF = -1.989;

%% Run Simulink Model
sim('bicycle_model.slx',simulation_time)
%% Post Simulation Results
mF = m*b/L;
mR = m*a/L;
CsF = FyF/SAF;
CsR = FyR/SAR;
FyFC = FyF.data./FzF.data;
FyRC = FyR.data./FzR.data;
FzF = unique(FzF.data);
FzR = unique(FzR.data);
% Kus = mF./CsF.data-mR./CsR.data; %For linear tyre models!
Kus = SWA.data./(latacc.data*SR)-180*L/(pi()*(V/3.6)^2);
if mean(Kus) < 0
    balance = 'oversteer';
%     Vcrit = 3.6 * sqrt(g*L/abs(Kus.data));
else
    balance = 'understeer';
%     Vchar = 3.6 * sqrt(g*L/Kus.data);
end

%% Save Parameters
if car_type == "Formula Car"
    save('formulacar_params.mat')
else
    save('roadcar_params.mat')
end

%% Ramp Input Plot Figures
if input == 1
% figure(1)
% subplot(2,3,1)
% plot(X.data,Y.data,line,'DisplayName',car_type)
% xlabel('X [m]')
% ylabel('Y [m]')
% lgd = legend;
% hold on
% subplot(2,3,2)
% plot(yawrate.time,yawrate.data,line,'DisplayName',car_type)
% xlabel('Time [s]')
% ylabel('Yaw Rate [deg/s]')
% lgd = legend;
% hold on
% subplot(2,3,3)
% plot(sideslip.time,sideslip.data,line,'DisplayName',car_type)
% xlabel('Time [s]')
% ylabel('Sideslip (Attitude) [deg]')
% lgd = legend;
% hold on
% subplot(2,3,4)
% plot(latacc.time,latacc.data,line,'DisplayName',car_type)
% xlabel('Time [s]')
% ylabel('Lateral Acceleration [m/s^2]')
% lgd = legend;
% hold on
% subplot(2,3,5)
% plot(SWA.time,SWA.data,line,'DisplayName',car_type)
% xlabel('Time [s]')
% ylabel('Steering Wheel Angle [deg]')
% lgd = legend;
% hold on
% 
% figure(2)
% subplot(2,2,1)
% plot(X.data,Y.data,line,'DisplayName',car_type)
% xlabel('X [m]')
% ylabel('Y [m]')
% lgd = legend;
% grid on
% hold on
% subplot(2,2,2)
% plot(SWA.data,latacc.data,line,'DisplayName',car_type)
% xlabel('Steering Angle [deg]')
% ylabel('Lateral Acceleration [m/s^2]')
% lgd = legend;
% grid on
% hold on
% subplot(2,2,3)
% plot(sideslip.data,latacc.data,line,'DisplayName',car_type)
% xlabel('Side Slip (Attitude) [deg]')
% ylabel('Lateral Acceleration [m/s^2]')
% lgd = legend;
% grid on
% hold on
% subplot(2,2,4)
% plot(sideslip.time,Kus,line,'DisplayName',car_type)
% xlabel('Time [s]')
% ylabel('US Gradient [m/(s^2*deg)]')
% lgd = legend;
% grid on
% hold on
% 
% figure(3)
% subplot(2,1,1)
% plot(SAF.time,SAF.data,line,'DisplayName',car_type)
% xlabel('Time [m]')
% ylabel('Slip Angle Front [deg]')
% ylim([-11 0])
% lgd = legend;
% grid on
% hold on
% subplot(2,1,2)
% plot(SAR.time,SAR.data,line,'DisplayName',car_type)
% xlabel('Time [m]')
% ylabel('Slip Angle Rear [deg]')
% ylim([-11 0])
% lgd = legend;
% grid on
% hold on
% 
% 
% figure(4)
% subplot(2,2,1)
% plot(SAF.data,FyF.data,line,'DisplayName',car_type)
% xlabel('Slip Angle Front [deg]')
% xlim([-11 0])
% ylim([0 8500])
% ylabel('Lateral Force Front [N]')
% lgd = legend;
% grid on
% hold on
% subplot(2,2,2)
% plot(SAF.data,FyFC,line,'DisplayName',car_type)
% xlabel('Slip Angle Front [deg]')
% ylabel('Lateral Force Coefficient Front [N/N]')
% xlim([-11 0])
% ylim([0 1.3])
% lgd = legend;
% grid on
% hold on
% subplot(2,2,3)
% plot(SAR.data,FyR.data,line,'DisplayName',car_type)
% xlabel('Slip Angle Rear [deg]')
% ylabel('Lateral Force Rear [N]')
% xlim([-8 0])
% ylim([0 8500])
% lgd = legend;
% grid on
% hold on
% subplot(2,2,4)
% plot(SAR.data,FyRC,line,'DisplayName',car_type)
% xlabel('Slip Angle Rear [deg]')
% ylabel('Lateral Force Coefficient Rear [N/N]')
% xlim([-8 0])
% ylim([0 1.3])
% lgd = legend;
% grid on
% hold on

else
%% Step Input Plot Figures
figure(1)
plot(latacc.time,latacc.data,line,'DisplayName',car_type)
xlabel('Time [s]')
ylabel('Lateral Acceleration [m/s^2]')
% xlim([-8 0])
% ylim([0 1.3])
lgd = legend;
grid on
hold on

figure(2)
plot(yawrate.time,yawrate.data,line,'DisplayName',car_type)
xlabel('Time [s]')
ylabel('Yaw Rate [deg/s]')
% xlim([-8 0])
% ylim([0 1.3])
lgd = legend;
grid on
hold on

end