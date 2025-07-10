clc; clear; close all;

% === Parameters ===
T = 1500;
dt = 10;
sigma_pos = 8;
sigma_sog = 1;
sigma_cog = 2;

% === Simulate Truth ===
[x_true, time] = simulateTruth(T, dt);

% === Generate Noisy Position ===
z_meas = generateAISMeas(x_true, sigma_pos);

% === Generate Noisy SOG/COG ===
[sog_meas, cog_meas, sog_true, cog_true] = ...
    addNoiseToSOGCOG(x_true, sigma_sog, sigma_cog);

% === Derive vx, vy from noisy sog/cog ===
[vx, vy] = deriveVelocityFromSOGCOG(sog_meas, cog_meas);

% === Save to CSV ===
saveAISLog('synthetic_ais_segment.csv', time, z_meas, vx, vy, sog_meas, cog_meas, sog_true, cog_true);

% === Plot ===
plotSimulatedData(x_true, z_meas, sog_meas, cog_meas, sog_true, cog_true);
