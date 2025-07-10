function runAISFilters()
% RUNAISFILTERS - Apply and analyze multiple filter types on AIS data
% Uses MATLAB's Sensor Fusion and Tracking Toolbox for filter implementations

% Close any previous figures
close all;
addpath('functions');

% Display banner
fprintf('\n========= AIS State Estimation Analysis =========\n\n');

% Load the AIS data
fprintf('Loading AIS data...\n');
dataPath = fullfile('data', 'specific_ais_movement.csv');
if ~exist(dataPath, 'file')
    error('AIS data file not found. Run simulateSpecificAISTrack first.');
end

data = readtable(dataPath);
fprintf('Loaded %d AIS points with %d variables\n', height(data), width(data));
q_kf=0.25;
q_ekf_cv = 0.25;
q_ekf_ca = 0.25;
q_ukf = 0.1;
% Run all filters
% fprintf('\nRunning Kalman Filter (CV model)...\n');
% [kf_estimates, kf_stats] = runKalmanFilter(data, q_kf);
% 
% fprintf('\nRunning Extended Kalman Filter Constant Velocity...\n');
% [ekf_cv_estimates, ekf_cv_stats] = runExtendedKalmanFilterCV(data, q_ekf_cv);
% 
% fprintf('\nRunning Extended Kalman Filter Constant Acceleration...\n');
% [ekf_ca_estimates, ekf_ca_stats] = runExtendedKalmanFilterCA(data, q_ekf_ca);
% 
% fprintf('\nRunning Unscented Kalman Filter...\n');
% [ukf_estimates, ukf_stats] = runUnscentedKalmanFilter(data, q_ukf);

fprintf('\nRunning IMM Filter...\n');
[imm_estimates, imm_stats] = runIMMFilter(data);
% 
% % Compare filter performance
% fprintf('\nComparing filter performance...\n');
% compareFilterPerformance(data, kf_estimates, ekf_cv_estimates, ekf_ca_estimates, ukf_estimates, imm_estimates);
% 
% % Analyze performance by segment
% fprintf('\nAnalyzing performance by motion segment...\n');
% analyseSegmentPerformance(data, kf_estimates, ekf_cv_estimates, ekf_ca_estimates, ukf_estimates, imm_estimates);

fprintf('\n========= Analysis Complete =========\n');
end