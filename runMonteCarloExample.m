% RUNMONTECARLOEXAMPLE - Demonstration script for Monte Carlo AIS simulation
%
% This script runs a complete Monte Carlo simulation to generate realistic
% vessel trajectories and AIS sensor data for filter testing. It demonstrates
% how to configure parameters, execute simulations, and analyze results.
%
% SIMULATION CONFIGURATION:
%   - 50 Monte Carlo runs with different randomized parameters
%   - 1-hour trajectories with 10-second AIS reporting
%   - Low measurement noise for high-quality sensor data
%   - Reproducible results using fixed random seed
%
% OUTPUTS:
%   - Complete Monte Carlo results saved to output/ directory
%   - Statistical summary printed to console
%   - Sample AIS dataset exported as CSV
%   - Visual analysis report generated automatically

clear all;
addpath('functions');

% STEP 1: GET DEFAULT PARAMETERS
% Load standard simulation parameters based on real-world AIS systems
params = getDefaultSimulationParameters();

% STEP 2: CUSTOMIZE SIMULATION PARAMETERS
% Configure for specific testing scenario

% Temporal settings - 1-hour simulation with 10-second AIS reports
params.totalDuration = 3600;        % Total simulation time (seconds) - 1 hour
params.aisReportInterval = 10;      % AIS reporting frequency (seconds) - every 10s

% Measurement noise levels - Simulate high-quality AIS equipment
params.measurementNoise.position = 6;   % GPS position error std (meters) - good accuracy
params.measurementNoise.velocity = 1.0; % Speed measurement error std (m/s) - typical AIS

% Motion behavior - Moderate maneuvering frequency
params.modeTransition.probability = 0.03; % 3% chance per second of changing motion mode

% STEP 3: RUN MONTE CARLO SIMULATION
% Execute 50 independent runs with randomized parameters and reproducible seed
fprintf('Starting Monte Carlo simulation...\n');
results = runMonteCarloSimulation(100, params, 'Seed', 42);

% STEP 4: DISPLAY RESULTS SUMMARY
% Print comprehensive statistics across all simulation runs
fprintf('\n===== Monte Carlo Simulation Results =====\n');
fprintf('Configuration:\n');
fprintf('  Number of runs: %d\n', results.summary.numRuns);
fprintf('  Total observations: %d\n', results.summary.totalObservations);
fprintf('  Random seed: %d (for reproducibility)\n', results.metadata.seed);

% Position accuracy statistics
fprintf('\nPosition Error Analysis:\n');
fprintf('  RMSE: %.2f m (root mean square error)\n', results.summary.position.rmse);
fprintf('  Mean: %.2f m (average error magnitude)\n', results.summary.position.mean);
fprintf('  Std:  %.2f m (error variability)\n', results.summary.position.std);
fprintf('  Max:  %.2f m (max error)\n', results.summary.position.max);
fprintf('  Min:  %.2f m (min error)\n', results.summary.position.min);

% Velocity accuracy statistics  
fprintf('\nVelocity Error Analysis:\n');
fprintf('  RMSE: %.2f m/s (speed tracking accuracy)\n', results.summary.velocity.rmse);
fprintf('  Mean: %.2f m/s (average speed error)\n', results.summary.velocity.mean);
fprintf('  Std:  %.2f m/s (speed error variability)\n', results.summary.velocity.std);
fprintf('  Max:  %.2f m/s (max error)\n', results.summary.velocity.max);
fprintf('  Min:  %.2f m/s (min error)\n', results.summary.velocity.min);

% Course accuracy statistics
fprintf('\nCourse Error Analysis:\n');
fprintf('  RMSE: %.2f rad (heading tracking accuracy)\n', results.summary.course.rmse);
fprintf('  Mean: %.2f rad (average heading error)\n', results.summary.course.mean);
fprintf('  Std:  %.2f rad (heading error variability)\n', results.summary.course.std);
fprintf('  Max:  %.2f m (max error)\n', results.summary.course.max);
fprintf('  Min:  %.2f m (min error)\n', results.summary.course.min);

% STEP 5: GENERATE SAMPLE AIS DATASET
% Create CSV file with realistic AIS data from first simulation run
fprintf('\n===== Sample Dataset Generation =====\n');
run = 1;  % Use first Monte Carlo run for sample dataset

% Convert simulation results to standard AIS CSV format
aisData = generateAISDataset(results.runs{run}.groundTruth, ...
                           results.runs{run}.observations, ...
                           results.runs{run}.metadata);

% Save to CSV file for use with filtering algorithms
outputFile = 'data/monte_carlo_sample.csv';
if ~exist('data', 'dir')
    mkdir('data');  % Create data directory if needed
end
writetable(aisData, outputFile);
fprintf('Sample AIS dataset saved to: %s\n', outputFile);
fprintf('Dataset contains: %d AIS reports with ground truth\n', height(aisData));

% STEP 6: DEBUGGING INFORMATION
% Provide detailed information about data generation quality
fprintf('\n=== DEBUG INFO ===\n');
fprintf('First run data quality check:\n');
fprintf('  Available observations in run 1: %d\n', sum(results.runs{1}.observations.available));
fprintf('  Total observations in run 1: %d\n', length(results.runs{1}.observations.available));
fprintf('  Ground truth steps in run 1: %d\n', size(results.runs{1}.groundTruth.position, 2));

% Calculate and display dropout rate
dropoutRate = 1 - (sum(results.runs{1}.observations.available) / length(results.runs{1}.observations.available));
fprintf('  AIS dropout rate: %.1f%% (realistic communication losses)\n', dropoutRate * 100);

% USAGE NOTES:
% - Results are automatically saved to output/ directory with timestamp
% - Visual analysis report is generated as PNG file
% - Sample CSV can be loaded with: data = readtable('data/monte_carlo_sample.csv')
% - Modify parameters above to test different scenarios
% - Change seed value or remove 'Seed' parameter for different random trajectories

fprintf('\n===== Simulation Complete =====\n');
fprintf('Check output/ directory for saved results and analysis plots\n');