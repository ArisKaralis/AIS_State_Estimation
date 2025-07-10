clc; clear; close all;

% === CONFIG ===
addpath('functions');
input_path = ('data');
data = fullfile(input_path, 'AIS_2022_03_31.csv');
output_path = 'output/';
targetMMSI = 368121820;
dt_target = 10;


% === PREPROCESS ===
%preprocessAISForKF(data, targetMMSI, dt_target);

% === VISUALIZE ===
% data_raw = readtable(fullfile(input_path, ['raw_ais_' num2str(targetMMSI) '.csv']));
% data_resampled = readtable(fullfile(input_path, ['resampled_ais_' num2str(targetMMSI) '.csv']));
% data_segmented = readtable(fullfile(input_path, ['resampled_moving_ais_', num2str(targetMMSI) '.csv']));
% 
% segmentAIS(data_segmented, num2str(targetMMSI), 15, 0.5, input_path);
data_segment15 = readtable(fullfile(input_path, 'segment15_ais_manualutm.csv'));
synthetic_data = readtable(fullfile(input_path, 'synthetic_ais_segment.csv'));
% plotInterMessageGaps(data_raw, data_resampled, data_segment15, targetMMSI);
% plotTrajectories(data_raw, data_resampled, data_segment15, targetMMSI);what
% compareSOGCOG(data_raw, data_resampled, data_segment15, targetMMSI);
% compareTimeGaps(data_raw, data_resampled, data_segment15, targetMMSI);
% 
% === KALMAN FILTER ===
[x_est, P_est, R, H] = kalmanCV_2D(synthetic_data);

% === EVALUATE ===
evaluateFilter(data_segment15, x_est, P_est, R, H);

