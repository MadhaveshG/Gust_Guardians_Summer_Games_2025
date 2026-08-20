%% ========================================================================
%  LIDAR BATCH PROCESSING PIPELINE (Blade Removal + Smoothing + Validation)
%  ========================================================================

clear; clc; close all;

%% 1. CONFIGURATION & SETUP
% -------------------------------------------------------------------------
% Define your folders here
lidar_dir = 'E:\Summer_games\LidarAssistedControl\Hurdles\solis_lidar_data\';
ref_dir   = 'E:\Summer_games\LidarAssistedControl\Hurdles\TurbulentWind\';

% Lidar Physics Constants
cone_angle_deg = 15;
cone_angle_rad = deg2rad(cone_angle_deg);
beams_per_scan = 50;
window_size    = 5;    % Smoothing window (5 seconds) to fix artificial turbulence
time_lag       = 12.0; % Seconds the Lidar sees AHEAD of the turbine

% Get list of all Lidar CSV files
file_list = dir(fullfile(lidar_dir, '*lidar_data_CircularCW.csv'));

% Initialize a table to store final statistics for all files
summary_stats = table();

%% 2. MAIN PROCESSING LOOP
% -------------------------------------------------------------------------
fprintf('Found %d files to process.\n', length(file_list));

for k = 1:length(file_list)
    
    % --- A. Load Lidar Data ---
    lidar_filename = file_list(k).name;
    full_path = fullfile(file_list(k).folder, lidar_filename);
    fprintf('\nProcessing File %d/%d: %s ...\n', k, length(file_list), lidar_filename);
    
    data = readtable(full_path);
    
    % --- B. Reconstruct Wind Speed (Blade Blockage Removal) ---
    num_measurements = height(data);
    num_scans = floor(num_measurements / beams_per_scan);
    
    % Pre-allocate
    results.time = zeros(num_scans, 1);
    results.wind_speed_raw = zeros(num_scans, 1);
    results.valid_points = zeros(num_scans, 1);
    
    % Azimuths (assuming beam 1 is North/360 and rotates CW)
    azimuths_rad = deg2rad(linspace(360, 360 - 360*(49/50), beams_per_scan)');
    
    for i = 1:num_scans
        idx_start = (i-1)*beams_per_scan + 1;
        idx_end = i*beams_per_scan;
        
        scan_speeds = data.lineOfSightWindSpeed1(idx_start:idx_end);
        scan_valid = data.isValid1(idx_start:idx_end);
        scan_time = data.time(idx_start);
        
        % Filter: Valid flag AND non-zero
        good_indices = find(scan_valid == 1 & scan_speeds ~= 0);
        
        if length(good_indices) < 10
            results.time(i) = scan_time;
            results.wind_speed_raw(i) = NaN;
            continue;
        end
        
        % Robust Least Squares (VAD)
        y = scan_speeds(good_indices);
        theta = azimuths_rad(good_indices);
        
        % Design Matrix: [cos(theta), sin(theta), offset]
        H = [cos(theta), sin(theta), ones(length(theta), 1)];
        x = H \ y; 
        
        % Calculate Axial Wind Speed (Looking forward)
        % V_wind = Offset / cos(cone_angle)
        wind_spd = x(3) / cos(cone_angle_rad);
        
        results.time(i) = scan_time;
        results.wind_speed_raw(i) = wind_spd;
        results.valid_points(i) = length(good_indices);
    end
    
    % --- C. Apply Smoothing (The Fix) ---
    results.wind_speed_smooth = movmean(results.wind_speed_raw, window_size);
    
    
    % --- D. Find and Load Matching Ground Truth ---
    % Logic: Convert "URef_18_Seed_1801_lidar...csv" -> "URef_18_Seed_1801.csv"
    ref_filename = strrep(lidar_filename, '_lidar_data_CircularCW.csv', '.csv');
    ref_full_path = fullfile(ref_dir, ref_filename);
    
    if exist(ref_full_path, 'file')
        ref_table = readtable(ref_full_path);
        
        % --- E. Validation & Statistics ---
        
        % Interpolate Reference to match Lidar time (Applying Time Lag)
        % Lidar(t) corresponds to Reference(t + lag)
        aligned_ref = interp1(ref_table.time, ref_table.REWS, ...
                              results.time + time_lag, 'linear', NaN);
        
        % Filter NaNs for stats
        mask = ~isnan(aligned_ref) & ~isnan(results.wind_speed_smooth);
        y_true = aligned_ref(mask);
        y_meas = results.wind_speed_smooth(mask);
        
        % 1. Accuracy
        R = corrcoef(y_true, y_meas);
        R_val = R(1,2);
        rmse = sqrt(mean((y_true - y_meas).^2));
        
        % 2. Turbulence Intensity (TI)
        TI_ref = (std(ref_table.REWS) / mean(ref_table.REWS)) * 100;
        TI_lidar = (std(results.wind_speed_smooth, 'omitnan') / mean(results.wind_speed_smooth, 'omitnan')) * 100;
        TI_ratio = TI_lidar / TI_ref;
        
        % Store Stats in Table
        new_row = table({lidar_filename}, R_val, rmse, TI_ref, TI_lidar, TI_ratio, ...
            'VariableNames', {'FileName', 'Correlation', 'RMSE', 'Ref_TI', 'Lidar_TI', 'TI_Ratio'});
        summary_stats = [summary_stats; new_row];
        
        % --- F. Plotting (One Figure per File) ---
        fig = figure('Name', lidar_filename, 'Visible', 'on');
        tiledlayout(2,1);
        
        % Subplot 1: Time Series
        nexttile;
        plot(results.time, results.wind_speed_raw, 'Color', [0.8 0.8 0.8], 'DisplayName', 'Raw Lidar (Noisy)');
        hold on;
        plot(results.time, results.wind_speed_smooth, 'r', 'LineWidth', 1.5, 'DisplayName', 'Smoothed Lidar');
        plot(results.time, aligned_ref, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth (Shifted)');
        title(['Wind Reconstruction: ' strrep(lidar_filename, '_', '\_')]);
        ylabel('Wind Speed (m/s)'); legend; grid on;
        xlim([0 100]); % Zoom on first 100s
        
        % Subplot 2: PSD (Spectrum)
        nexttile;
        fs = 1 / mean(diff(results.time));
        [pxx_ref, f_ref] = pwelch(ref_table.REWS, [], [], [], 4);
        [pxx_lid, f_lid] = pwelch(results.wind_speed_smooth, [], [], [], fs);
        loglog(f_ref, pxx_ref, 'k', 'LineWidth', 2); hold on;
        loglog(f_lid, pxx_lid, 'r', 'LineWidth', 1.5);
        title('Power Spectral Density'); xlabel('Hz'); ylabel('PSD');
        xlim([0.01 1]); grid on;
        legend('Ground Truth', 'Lidar');
        
        % Save Figure (Optional)
        % saveas(fig, ['Analysis_' lidar_filename '.png']);
        
    else
        fprintf('WARNING: Reference file not found for %s. Skipping validation.\n', lidar_filename);
    end
end


% --- G. SAVE FOR SIMULINK ---
    % Create a structure for the controller
    LidarData.time = results.time;
    LidarData.wind_speed = results.wind_speed_smooth;
    LidarData.preview_time = time_lag; % Store the 12s lag so the controller knows!
    
    % Save as .mat file
    save_filename = strrep(lidar_filename, '.csv', '_Cleaned.mat');
    save_path = fullfile(lidar_dir, save_filename);
    save(save_path, 'LidarData');
    
    fprintf('Saved clean data to: %s\n', save_filename);
%% 3. FINAL SUMMARY REPORT
% -------------------------------------------------------------------------
fprintf('\n\n================ FINAL BATCH REPORT ================\n');
disp(summary_stats);

% Calculate average TI Ratio across all files
avg_ratio = mean(summary_stats.TI_Ratio);
fprintf('Average TI Measurement Ratio across all files: %.2f\n', avg_ratio);