clc
clear all

%% Choose Car & Load Parameters into Workspace
% answer = questdlg('Which car would you like to use?','Choose Car','Formula Car','Road Car','None');
% switch answer
%     case "Formula Car"
%         bicycle_coefficient_formula
        load('formulacar_params.mat')
%         answer = "Formula Car";
%     case "Road Car"
%         bicycle_coefficient_roadcar
%         load('roadcar_params.mat')
%         answer = "Road Car";
% end
%% Modify Parameters
% SA = 10;
V = 120;
% L = 3.5;
% a = 1.9;
% b = L-a;
% CF = 1.7631;
% BF = 0.3609;
% EF = -1.989;
%% Run Simulink Model
FzF = 4.463323015873017e+03;
FzR = 4.725871428571430e+03;
sim('pacejka_lateral_model.slx',10)
%% Post Simulation Results
FyFC = FyF.data./FzF.data;
FyRC = FyR.data./FzR.data;
FzF = unique(FzF.data);
FzR = unique(FzR.data);
%% Generate Plots
% figure(1)
% subplot(2,2,1)
% plot(SAF.data,FyF.data,'--')
% legend(strcat("FzF = ",num2str(FzF(1))),strcat("FzF = ",num2str(FzF(2))),strcat("FzF = ",num2str(FzF(3))),...
%     strcat("FzF = ",num2str(FzF(4))),strcat("FzF = ",num2str(FzF(5))),strcat("FzF = ",num2str(FzF(6))),...
%     strcat("FzF = ",num2str(FzF(7))),strcat("FzF = ",num2str(FzF(8))));
% xlabel('Slip Angle [deg]')
% ylabel('Lateral Force [N]')
% subplot(2,2,2)
% plot(SAR.data,FyR.data,'--')
% legend(strcat("FzR = ",num2str(FzR(1))),strcat("FzR = ",num2str(FzR(2))),strcat("FzR = ",num2str(FzR(3))),...
%     strcat("FzR = ",num2str(FzR(4))),strcat("FzR = ",num2str(FzR(5))),strcat("FzR = ",num2str(FzR(6))),...
%     strcat("FzR = ",num2str(FzR(7))),strcat("FzR = ",num2str(FzR(8))));
% xlabel('Slip Angle [deg]')
% ylabel('Lateral Force [N]')
% subplot(2,2,3)
% plot(SAF.data,FyFC,'--')
% legend(strcat("FzF = ",num2str(FzF(1))),strcat("FzF = ",num2str(FzF(2))),strcat("FzF = ",num2str(FzF(3))),...
%     strcat("FzF = ",num2str(FzF(4))),strcat("FzF = ",num2str(FzF(5))),strcat("FzF = ",num2str(FzF(6))),...
%     strcat("FzF = ",num2str(FzF(7))),strcat("FzF = ",num2str(FzF(8))));
% xlabel('Slip Angle [deg]')
% ylabel('Lateral Force Coefficient [N/N]')
% subplot(2,2,4)
% plot(SAR.data,FyRC,'--')
% legend(strcat("FzR = ",num2str(FzR(1))),strcat("FzR = ",num2str(FzR(2))),strcat("FzR = ",num2str(FzR(3))),...
%     strcat("FzR = ",num2str(FzR(4))),strcat("FzR = ",num2str(FzR(5))),strcat("FzR = ",num2str(FzR(6))),...
%     strcat("FzR = ",num2str(FzR(7))),strcat("FzR = ",num2str(FzR(8))));
% xlabel('Slip Angle [deg]')
% ylabel('Lateral Force Coefficient [N/N]')

figure(6)
subplot(2,2,1)
plot(SAF.data,FyF.data,line,'DisplayName',car_type)
legend(strcat("FzF = ",num2str(FzF(1))));
xlabel('Slip Angle Front [deg]')
ylabel('Lateral Force Front [N]')
grid on
hold on
subplot(2,2,2)
plot(SAF.data,FyFC,line,'DisplayName',car_type)
legend(strcat("FzF = ",num2str(FzF(1))));
xlabel('Slip Angle Front[deg]')
ylabel('Lateral Force Coefficient Front [N/N]')
grid on
hold on
subplot(2,2,3)
plot(SAR.data,FyR.data,line,'DisplayName',car_type)
legend(strcat("FzR = ",num2str(FzR(1))));
xlabel('Slip Angle Rear [deg]')
ylabel('Lateral Force Rear [N]')
grid on
hold on
subplot(2,2,4)
plot(SAR.data,FyRC,line,'DisplayName',car_type)
legend(strcat("FzR = ",num2str(FzR(1))));
xlabel('Slip Angle Rear [deg]')
ylabel('Lateral Force Coefficient Rear [N/N]')
grid on
hold on
%% Save Parameters
% if answer == "Formula Car"
%     save('formulacar_params.mat')
% else
%     save('roadcar_params.mat')
% end