clc; clear; close all;
addpath('functions');

% === Load synthetic AIS data ===
data = readtable('data/synthetic_ais_segment.csv');

% === Run Kalman Filter ===
[x_est, P_est, R, H] = kalmanCV_2D(data);

% === Evaluate the result ===
evaluateSimulatedKF(data, x_est, P_est, R, H);