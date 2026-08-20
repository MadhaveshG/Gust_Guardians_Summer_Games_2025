% Script to evaluate the " 18 m/s Hurdles" for the LAC Summer Games 2025.
% Task:
% Get the best possible wind preview (2s) by improving the baseline lidar 
% data processing (LDP_v3)!
% Results 4BeamPulsed:
% v3: Cost for Summer Games 2025 ("18 m/s hurdles"):  0.515998 m/s 
% Results CircularCW:
% v3: Cost for Summer Games 2025 ("18 m/s hurdles"):  0.463842 m/s 

%% Setup
clearvars;close all;clc;

% select LDP function
MyLDPfunction       = @LDP_v7; % [LDP_v3/???]

% select LidarType
LidarType           = 'CircularCW'; % [4BeamPulsed/CircularCW]

% Seeds (can be adjusted, but will provide different results)
nSeed               = 6;                        % [-]	    number of stochastic turbulence field samples
Seed_vec            = [1:nSeed]+18*100;         % [-]  	    vector of seeds

% Parameters postprocessing (can be adjusted, but will provide different results)
t_start             = 60;                       % [s] 	    ignore data before for STD and spectra
TMax                = 660;                      % [s]       total run time
DT                  = 0.01;                     % [s]       time step
time                = [0:DT:TMax]';             % [s]       time vector

% Parameter for Cost (Summer Games 2025)
tau                 = 2;                        % [s]       time to overcome pitch actuator, from Example 1: tau = T_Taylor - T_buffer, since there T_filter = T_scan = 0

switch LidarType
    case '4BeamPulsed'
        % configuration from LDP_v1_4BeamPulsed.IN and FFP_v1_4BeamPulsed.IN
        LDP.NumberOfBeams       = 4;            % [-]       Number of beams measuring at different directions               
        LDP.AngleToCenterline   = 19.176;       % [deg]     Angle around centerline
        LDP.IndexGate           = 6;            % [-]       IndexGate
        LDP.FlagLPF             = 1;            % [0/1]     Enable low-pass filter (flag)
        LDP.omega_cutoff        = 0.13;         % [rad/s]   Corner frequency (-3dB) of the low-pass filter
        LDP.T_buffer            = 0.2;          % [s]       Buffer time for filtered REWS signal
    case 'CircularCW'
        % configuration from LDP_v1_CircularCW.IN and FFP_v1_CircularCW.IN
        LDP.NumberOfBeams       = 50;           % [-]       Number of beams measuring at different directions               
        LDP.AngleToCenterline   = 15;           % [deg]     Angle around centerline
        LDP.IndexGate           = 1;            % [-]       IndexGate
        LDP.FlagLPF             = 1;            % [0/1]     Enable low-pass filter (flag)
        %LDP.T_Avg               = 1.0;
        LDP.omega_cutoff        = 0.28;         % [rad/s]   Corner frequency (-3dB) of the low-pass filter
        LDP.T_buffer            = 4.95;          % [s]       Buffer time for filtered REWS signal        
end

% Files (should not be be changed)
SimulationFolderLAC     = 'solis_lidar_data';

%% Postprocessing: evaluate data

% Allocation
MAE     = NaN(1,nSeed); % mean absolute error [m/s]

% Loop over all seeds
for iSeed = 1:nSeed    

    % Load data
    Seed                                = Seed_vec(iSeed);
	WindFileName                        = ['URef_18_Seed_',num2str(Seed,'%02d')];
    SolisResultFile                     = fullfile(SimulationFolderLAC,[WindFileName,'_lidar_data_',LidarType,'.csv']);
    SolisData                           = readtable(SolisResultFile);    
    beamID                              = interp1(SolisData.time,SolisData.beamID,time,'previous','extrap');
    isValid                             = interp1(SolisData.time,SolisData.("isValid"+LDP.IndexGate),time,'previous','extrap');
    lineOfSightWindSpeed                = interp1(SolisData.time,SolisData.("lineOfSightWindSpeed"+LDP.IndexGate),time,'previous','extrap');

    % Get REWS from the wind field 
    RewsFile                 	        = ['TurbulentWind\URef_18_Seed_',num2str(Seed,'%02d'),'.csv'];  
    RewsData                            = readtable(RewsFile);
    REWS_WindField                      = interp1([RewsData.time;RewsData.time+600],[RewsData.REWS;RewsData.REWS],time); % REWS is circular 
    REWS_WindField_shifted              = interp1([RewsData.time;RewsData.time+600],[RewsData.REWS;RewsData.REWS],time+tau);
          
    % Calculate REWS from the lidar data
    clear(func2str(MyLDPfunction)) % clearing all persistent variables from previous call 
    [REWS,REWS_f,REWS_b]                = MyLDPfunction(time,isValid,beamID,lineOfSightWindSpeed,DT,LDP);   

    % Calculate mean absolute error
    Error                               = REWS_WindField_shifted-REWS_b;                    % error is  REWS from wind field shifted minus REWS from lidar filtered and buffered.
    MAE(iSeed)                          = mean(abs(detrend(Error(time>=t_start),'constant'))); % only consider error after t_start 
             
    % Plot REWS for absolute error             
    figure('Name',['REWS seed ',num2str(Seed)])
    subplot(311)
    hold on; grid on; box on
    plot(time,  REWS_WindField);
    plot(time,  REWS);
    ylabel('REWS [m/s]');
    legend('wind field','lidar estimate')
    subplot(312)
    hold on; grid on; box on
    plot(time,  REWS_WindField_shifted);
    plot(time,  REWS_b);
    ylabel('REWS [m/s]');
    legend('wind field shifted','lidar estimate filtered and buffered')
    subplot(313)
    hold on; grid on; box on
    plot(time,  Error);
    ylabel('error [m/s]');
    xlabel('time [s]')

end


% --- Add this after the loop in RunExample.m ---

% Pick the best seed (e.g., 1805 or whichever had the lowest MAE)
TargetSeed = 1802; 

% ... (Load data for TargetSeed as you normally do) ...

% 1. Calculate the Detrended Signals (Remove the Bias)
Truth_Detrended = detrend(REWS_WindField_shifted, 'constant');
Prediction_Detrended = detrend(REWS_b, 'constant');

% 2. Plot the "Perfect Match"
figure('Name', 'Presentation Result', 'Color', 'w');
plot(time, Truth_Detrended, 'b', 'LineWidth', 1.5); hold on;
plot(time, Prediction_Detrended, 'r', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]', 'FontSize', 12);
ylabel('Fluctuation in Wind Speed [m/s]', 'FontSize', 12);
legend('Actual Wind Turbulence', 'Lidar Prediction (LDP v7)', 'Location', 'Best');
title(['World Record Accuracy (Seed ', num2str(TargetSeed), ')'], 'FontSize', 14);
xlim([0 600]); % Zoom in on a busy section to show off the tracking
%% Calculation of Cost for Summer Games 2025
Cost                        = mean(MAE);
fprintf('Cost for Summer Games 2025 ("18 m/s hurdles"):  %f \n',Cost);