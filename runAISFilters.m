function runAISFilters()
% RUNAISFILTERS - Apply and analyze multiple filter types on AIS data
% Uses MATLAB's Sensor Fusion and Tracking Toolbox for filter implementations

% Close any previous figures
addpath('functions');

% Display banner
fprintf('\n========= AIS State Estimation Analysis =========\n\n');

% Load the AIS data
fprintf('Loading AIS data...\n');
dataPath = fullfile('data', 'monte_carlo_sample.csv');
if ~exist(dataPath, 'file')
    error('AIS data file not found. Run simulateSpecificAISTrack first.');
end

data = readtable(dataPath);
fprintf('Loaded %d AIS points with %d variables\n', height(data), width(data));
% OPTIMAL CALIBRATED PARAMETERS
q_kf = 0.3;
q_ekf_cv = 0.014;    % ← Update this
q_ekf_ca = 0.010;
q_ekf_ctrv = 0.030;
q_ukf_cv = 0.060;
q_ukf_ctrv = 0.1;
q_imm_cv = 0.010;
q_imm_ctrv = 0.063;
q_imm_ca = 0.126;
q_imm_cv2 = 0.010;
q_imm_ctrv2 = 0.420;

pos_std = 4;         
vel_std = 0.6;       
acc_std = 0.1;

% % Run all filters
fprintf('\nRunning Kalman Filter (CV model)...\n');
[kf_est, kf_stats] = runKF(data, q_kf, pos_std, vel_std);

fprintf('\nRunning Extended Kalman Filter Constant Velocity...\n');
[ekf_est_cv, ekf_cv_stats] = runEKFCV(data, q_ekf_cv, pos_std, vel_std);

fprintf('\nRunning Extended Kalman Filter Constant Acceleraction...\n');
[ekf_est_ca, ekf_ca_stats] = runEKFCA(data, q_ekf_ca, pos_std, vel_std, acc_std);

fprintf('\nRunning Extended Kalman Filter Constant Turn Rate Velocity...\n');
[ekf_est_ctrv, ekf_ctrv_stats] = runEKFCTRV(data, q_ekf_ctrv, pos_std, vel_std);

fprintf('\nRunning Unscented Kalman Filter...\n');
[ukf_est_cv, ukf_cv_stats] = runUKFCV(data, q_ukf_cv, pos_std, vel_std);

fprintf('\nRunnin Unscented Kalman Filter...\n');
[ukf_est_ctrv, ukf_ctrv_stats] = runUKFCTRV(data, q_ukf_ctrv, pos_std, vel_std);

fprintf('\nRunning IMM Filter...\n');
[imm_est_2, imm_stats] = runIMMEKF2(data, q_imm_cv2, q_imm_ctrv2, pos_std, vel_std);
[imm_est_3, imm_stats] = runIMMFilterEKF(data, q_ekf_cv, q_ekf_ca, q_ekf_ctrv, pos_std, vel_std, acc_std);


% Create the filter estimates struct using the helper function
filter_estimates = createFilterEstimatesStruct('KF', kf_est, ...
                                               'EKF_CV', ekf_est_cv, ...
                                               'EKF_CA', ekf_est_ca, ...
                                               'EKF_CTRV', ekf_est_ctrv, ...
                                               'UKF_CV', ukf_est_cv, ...
                                               'UKF_CTRV', ukf_est_ctrv, ...
                                               'IMM_3', imm_est_3, ...
                                               'IMM_2', imm_est_2);

% Compare filter performance
fprintf('\nComparing filter performance...\n');
compareFilterPerformance(data, filter_estimates);

% Analyze performance by segment
fprintf('\nAnalyzing performance by motion segment...\n');
analyseSegmentPerformance(data, filter_estimates);

runEnhancedVisualization();

fprintf('\n========= Analysis Complete =========\n');
end

