clc
clear all
try
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

    try
    % Modify Parameters
    prompt = {'Simulation Time:', 'Slope:', 'Line:', 'SA:', 'V:', 'L:', 'a:', 'a1F:', 'a1R:', 'CF:', 'BF:', 'EF:'};
    dlgtitle = 'Would you like to modify any parameters?';
    definput = {'20','2', 'blue--', '11', '120', '3.5', '1.8', '-1e-5', '-1e-5', '1.8391', '0.2719', '-2.5276'};
    fieldsize = [1 45; 1 45; 1 45; 1 45; 1 45; 1 45; 1 45; 1 45; 1 45; 1 45; 1 45; 1 45];
    answer = inputdlg(prompt,dlgtitle,fieldsize,definput);
    simulation_time = str2double(answer{1});
    slope = str2double(answer{2}); %deg/s
    line = answer{3};
    SA = str2double(answer{4});
    V = str2double(answer{5});
    L = str2double(answer{6});
    a = str2double(answer{7});
    b = L-a;
    a1F = str2double(answer{8});
    a1R = str2double(answer{9});
    CF = str2double(answer{10});
    BF = str2double(answer{11});
    EF = str2double(answer{12});
    catch
    end

    %% Run Bicycle Model
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
    Kus = (SWA.data./(latacc.data*SR))-(180*L/(pi()*(V/3.6)^2));
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

    %% Run Pacejka Model
    sim('pacejka_lateral_model.slx',10)

    %% Post Simulation Results
    FyFC_Tyres = FyF_Tyres.data./FzF;
    FyRC_Tyres = FyR_Tyres.data./FzR;
    % FzF = unique(FzF.data);
    % FzR = unique(FzR.data);
    %% Plot Figures
    set(0,'DefaultFigureWindowStyle','docked');
    figure(1)
    subplot(2,3,1)
    plot(X.data,Y.data,line,'DisplayName',car_type)
    xlabel('X [m]')
    ylabel('Y [m]')
    xlim([-max(max(abs(X.data),abs(Y.data))) max(max(abs(X.data),abs(Y.data)))])
    ylim([-max(max(abs(X.data),abs(Y.data))) max(max(abs(X.data),abs(Y.data)))])
    lgd = legend;
    hold on
    subplot(2,3,2)
    plot(yawrate.time,yawrate.data,line,'DisplayName',car_type)
    xlabel('Time [s]')
    ylabel('Yaw Rate [deg/s]')
    lgd = legend;
    hold on
    subplot(2,3,3)
    plot(sideslip.time,sideslip.data,line,'DisplayName',car_type)
    xlabel('Time [s]')
    ylabel('Sideslip (Attitude) [deg]')
    lgd = legend;
    hold on
    subplot(2,3,4)
    plot(latacc.time,latacc.data,line,'DisplayName',car_type)
    xlabel('Time [s]')
    ylabel('Lateral Acceleration [m/s^2]')
    lgd = legend;
    hold on
    subplot(2,3,5)
    plot(SWA.time,SWA.data,line,'DisplayName',car_type)
    xlabel('Time [s]')
    ylabel('Steering Wheel Angle [deg]')
    lgd = legend;
    hold on

    figure(2)
    subplot(2,2,1)
    plot(X.data,Y.data,line,'DisplayName',car_type)
    xlabel('X [m]')
    ylabel('Y [m]')
    xlim([-max(max(abs(X.data),abs(Y.data))) max(max(abs(X.data),abs(Y.data)))])
    ylim([-max(max(abs(X.data),abs(Y.data))) max(max(abs(X.data),abs(Y.data)))])
    lgd = legend;
    grid on
    hold on
    subplot(2,2,2)
    plot(SWA.data,latacc.data,line,'DisplayName',car_type)
    xlabel('Steering Angle [deg]')
    ylabel('Lateral Acceleration [m/s^2]')
    lgd = legend;
    grid on
    hold on
    subplot(2,2,3)
    plot(sideslip.data,latacc.data,line,'DisplayName',car_type)
    xlabel('Side Slip (Attitude) [deg]')
    ylabel('Lateral Acceleration [m/s^2]')
    lgd = legend;
    grid on
    hold on
    subplot(2,2,4)
    plot(sideslip.time,Kus,line,'DisplayName',car_type)
    xlabel('Time [s]')
    ylabel('US Gradient [deg/(m/s^2)]')
    lgd = legend;
    grid on
    hold on

    figure(3)
    subplot(2,1,1)
    plot(SAF.time,SAF.data,line,'DisplayName',car_type)
    xlabel('Time [m]')
    ylabel('Slip Angle Front [deg]')
    % ylim([-11 0])
    lgd = legend;
    grid on
    hold on
    subplot(2,1,2)
    plot(SAR.time,SAR.data,line,'DisplayName',car_type)
    xlabel('Time [m]')
    ylabel('Slip Angle Rear [deg]')
    % ylim([-11 0])
    lgd = legend;
    grid on
    hold on


    figure(4)
    subplot(2,2,1)
    plot(SAF.data,FyF.data,line,'DisplayName',car_type)
    xlabel('Slip Angle Front [deg]')
    % xlim([-11 0])
    % ylim([0 8500])
    ylabel('Lateral Force Front [N]')
    lgd = legend;
    grid on
    hold on
    subplot(2,2,2)
    plot(SAF.data,FyFC,line,'DisplayName',car_type)
    xlabel('Slip Angle Front [deg]')
    ylabel('Lateral Force Coefficient Front [N/N]')
    % xlim([-11 0])
    % ylim([0 1.3])
    lgd = legend;
    grid on
    hold on
    subplot(2,2,3)
    plot(SAR.data,FyR.data,line,'DisplayName',car_type)
    xlabel('Slip Angle Rear [deg]')
    ylabel('Lateral Force Rear [N]')
    % xlim([-8 0])
    % ylim([0 8500])
    lgd = legend;
    grid on
    hold on
    subplot(2,2,4)
    plot(SAR.data,FyRC,line,'DisplayName',car_type)
    xlabel('Slip Angle Rear [deg]')
    ylabel('Lateral Force Coefficient Rear [N/N]')
    % xlim([-8 0])
    % ylim([0 1.3])
    lgd = legend;
    grid on
    hold on

    figure(5)
    subplot(2,2,1)
    plot(SAF_Tyres.data,FyF_Tyres.data,line,'DisplayName',strcat("FzF = ",num2str(FzF(1))))
    xlabel('Slip Angle Front [deg]')
    ylabel('Lateral Force Front [N]')
    lgd = legend;
    grid on
    hold on
    subplot(2,2,2)
    plot(SAF_Tyres.data,FyFC_Tyres,line,'DisplayName',strcat("FzF = ",num2str(FzF(1))))
    xlabel('Slip Angle Front[deg]')
    ylabel('Lateral Force Coefficient Front [N/N]')
    lgd = legend;
    grid on
    hold on
    subplot(2,2,3)
    plot(SAR_Tyres.data,FyR_Tyres.data,line,'DisplayName',strcat("FzR = ",num2str(FzR(1))))
    xlabel('Slip Angle Rear [deg]')
    ylabel('Lateral Force Rear [N]')
    lgd = legend;
    grid on
    hold on
    subplot(2,2,4)
    plot(SAR_Tyres.data,FyRC_Tyres,line,'DisplayName',strcat("FzR = ",num2str(FzR(1))))
    xlabel('Slip Angle Rear [deg]')
    ylabel('Lateral Force Coefficient Rear [N/N]')
    lgd = legend;
    grid on
    hold on

catch ME
    message = getReport(ME);
    errordlg(message)
end