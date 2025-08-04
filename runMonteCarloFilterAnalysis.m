function results = runMonteCarloFilterAnalysis(varargin)
% RUNMONTECARLOFILTERANALYSIS - Comprehensive Monte Carlo analysis of best performing filters
%
% This function runs Monte Carlo simulations to statistically evaluate the performance
% of the three best performing filters: 2-model IMM, 3-model IMM, and EKF-CTRV.
% It generates multiple trajectory types and analyzes filter robustness and consistency.
%
% USAGE:
%   results = runMonteCarloFilterAnalysis()  % Use defaults
%   results = runMonteCarloFilterAnalysis('NumRuns', 50, 'FilterTypes', {'IMM_2', 'IMM_3', 'EKF_CTRV'})
%
% OPTIONAL PARAMETERS:
%   'NumRuns'       - Number of Monte Carlo runs (default: 50)
%   'FilterTypes'   - Cell array of filter names (default: {'IMM_2', 'IMM_3', 'EKF_CTRV'})
%   'SaveResults'   - Save results to files (default: true)
%   'Seed'          - Random seed for reproducibility (default: 42)
%   'Verbose'       - Display progress information (default: true)

% Parse input arguments
p = inputParser;
addParameter(p, 'NumRuns', 500, @isnumeric);
addParameter(p, 'FilterTypes', {'IMM_2', 'IMM_3', 'EKF_CTRV'}, @iscell);
addParameter(p, 'SaveResults', true, @islogical);
addParameter(p, 'Seed', 42, @isnumeric);
addParameter(p, 'Verbose', true, @islogical);
parse(p, varargin{:});

numRuns = p.Results.NumRuns;
filterTypes = p.Results.FilterTypes;
saveResults = p.Results.SaveResults;
seedValue = p.Results.Seed;
verbose = p.Results.Verbose;

% Add required paths (following runMonteCarloExample.m pattern)
addpath('simulateTrajectory');
addpath('functions');

if verbose
    fprintf('\n========= Monte Carlo Filter Analysis for Best Filters =========\n\n');
    fprintf('Number of runs: %d\n', numRuns);
    fprintf('Filter types: %s\n', strjoin(filterTypes, ', '));
    fprintf('Random seed: %d\n', seedValue);
end

% STEP 1: GET DEFAULT PARAMETERS (following runMonteCarloExample.m pattern)
% Load standard simulation parameters based on real-world AIS systems
params = getDefaultSimulationParameters();

% STEP 2: CUSTOMIZE SIMULATION PARAMETERS (following runMonteCarloExample.m pattern)
% Configure for specific testing scenario
% Temporal settings - 1-hour simulation with 10-second AIS reports
params.totalDuration = 3600;        % Total simulation time (seconds) - 1 hour
params.aisReportInterval = 10;      % AIS reporting frequency (seconds) - every 10s

% Measurement noise levels - Simulate high-quality AIS equipment
params.measurementNoise.position = 6;   % GPS position error std (meters) - good accuracy
params.measurementNoise.velocity = 1.0; % Speed measurement error std (m/s) - typical AIS

% Motion behavior - Moderate maneuvering frequency
params.modeTransition.probability = 0.03; % 3% chance per second of changing motion mode

% STEP 3: RUN MONTE CARLO SIMULATION (following runMonteCarloExample.m pattern)
% Execute Monte Carlo runs with randomized parameters and reproducible seed
fprintf('Starting Monte Carlo simulation...\n');
monteCarloResults = runMonteCarloSimulation(numRuns, params, 'Seed', seedValue);

% STEP 4: ANALYZE MONTE CARLO RESULTS WITH FILTERS
% Initialize filter analysis results structure
results = struct();
results.filterStats = struct();
results.rawResults = cell(numRuns, length(filterTypes));
results.comparison = struct();
results.metadata = struct();
results.metadata.timestamp = datetime('now');
results.metadata.numRuns = numRuns;
results.metadata.filterTypes = filterTypes;
results.metadata.monteCarloSeed = seedValue;

% Initialize measurement statistics tracking with separate metrics
measurementStats = struct();
measurementStats.position_rmse = [];
measurementStats.velocity_rmse = [];
measurementStats.sog_rmse = [];
measurementStats.cog_rmse = [];
measurementStats.valid_runs = [];
measurementStats.observation_counts = [];

% Initialize filter statistics with separate metrics for each type
for i = 1:length(filterTypes)
    filterName = filterTypes{i};
    results.filterStats.(filterName) = struct();
    
    % Position metrics
    results.filterStats.(filterName).position_rmse = [];
    results.filterStats.(filterName).position_mean = [];
    results.filterStats.(filterName).position_std = [];
    results.filterStats.(filterName).position_max = [];
    results.filterStats.(filterName).position_min = [];
    
    % Velocity metrics
    results.filterStats.(filterName).velocity_rmse = [];
    results.filterStats.(filterName).velocity_mean = [];
    results.filterStats.(filterName).velocity_std = [];
    results.filterStats.(filterName).velocity_max = [];
    results.filterStats.(filterName).velocity_min = [];
    
    % SOG metrics
    results.filterStats.(filterName).sog_rmse = [];
    results.filterStats.(filterName).sog_mean = [];
    results.filterStats.(filterName).sog_std = [];
    results.filterStats.(filterName).sog_max = [];
    results.filterStats.(filterName).sog_min = [];
    
    % COG metrics
    results.filterStats.(filterName).cog_rmse = [];
    results.filterStats.(filterName).cog_mean = [];
    results.filterStats.(filterName).cog_std = [];
    results.filterStats.(filterName).cog_max = [];
    results.filterStats.(filterName).cog_min = [];
    
    % Runtime metrics
    results.filterStats.(filterName).runtime = [];
end

if verbose
    fprintf('\n===== Testing Filters on Monte Carlo Data =====\n');
    fprintf('Processing %d Monte Carlo runs with %d filter types...\n', numRuns, length(filterTypes));
end

validRunCount = 0;

% Process each Monte Carlo run
for run = 1:numRuns
    if verbose
        fprintf('Processing run %d/%d\n', run, numRuns);
    end
    
    try
        % Extract Monte Carlo results for this run
        groundTruth = monteCarloResults.runs{run}.groundTruth;
        observations = monteCarloResults.runs{run}.observations;
        metadata = monteCarloResults.runs{run}.metadata;
        
        % Convert to AIS dataset format (following runMonteCarloExample.m pattern)
        aisData = generateAISDataset(groundTruth, observations, metadata);
        
        % Debug information
        if verbose
            fprintf('  AIS data points: %d\n', height(aisData));
        end
        
        % Skip runs with insufficient data
        if height(aisData) < 20
            if verbose
                fprintf('  Skipping run %d: Insufficient data (%d points)\n', run, height(aisData));
            end
            continue;
        end
        
        % Calculate measurement statistics for this run
        measurementMetrics = calculateMeasurementStatistics(aisData);
        measurementStats.position_rmse(end+1) = measurementMetrics.position_rmse;
        measurementStats.velocity_rmse(end+1) = measurementMetrics.velocity_rmse;
        measurementStats.sog_rmse(end+1) = measurementMetrics.sog_rmse;
        measurementStats.cog_rmse(end+1) = measurementMetrics.cog_rmse;
        measurementStats.observation_counts(end+1) = height(aisData);
        
        validRunCount = validRunCount + 1;
        measurementStats.valid_runs(end+1) = run;
        
        if verbose
            fprintf('  Measurement RMSE - Pos: %.2fm, Vel: %.2fm/s, SOG: %.2fm/s, COG: %.2f°\n', ...
                measurementMetrics.position_rmse, measurementMetrics.velocity_rmse, ...
                measurementMetrics.sog_rmse, measurementMetrics.cog_rmse);
        end
        
        % Test each filter on this dataset
        for f = 1:length(filterTypes)
            filterName = filterTypes{f};
            
            try
                if verbose
                    fprintf('  Testing %s filter...\n', filterName);
                end
                
                tic;
                [estimates, stats] = runFilterByName(filterName, aisData);
                runtime = toc;
                
                % Calculate performance metrics using the stats structure
                metrics = calculateFilterPerformance(aisData, estimates, stats);
                metrics.runtime = runtime;
                
                % Store results only if valid
                if ~isnan(metrics.position_rmse)
                    results.rawResults{run, f} = metrics;
                    
                    % Update position statistics
                    results.filterStats.(filterName).position_rmse(end+1) = metrics.position_rmse;
                    results.filterStats.(filterName).position_mean(end+1) = metrics.position_mean;
                    results.filterStats.(filterName).position_std(end+1) = metrics.position_std;
                    results.filterStats.(filterName).position_max(end+1) = metrics.position_max;
                    results.filterStats.(filterName).position_min(end+1) = metrics.position_min;

                    % Update velocity statistics
                    if isfield(metrics, 'velocity_rmse') && ~isnan(metrics.velocity_rmse)
                        results.filterStats.(filterName).velocity_rmse(end+1) = metrics.velocity_rmse;
                        results.filterStats.(filterName).velocity_mean(end+1) = metrics.velocity_mean;
                        results.filterStats.(filterName).velocity_std(end+1) = metrics.velocity_std;
                        results.filterStats.(filterName).velocity_max(end+1) = metrics.velocity_max;
                        results.filterStats.(filterName).velocity_min(end+1) = metrics.velocity_min;
                    end
                    
                    % Update SOG statistics
                    if isfield(metrics, 'sog_rmse') && ~isnan(metrics.sog_rmse)
                        results.filterStats.(filterName).sog_rmse(end+1) = metrics.sog_rmse;
                        results.filterStats.(filterName).sog_mean(end+1) = metrics.sog_mean;
                        results.filterStats.(filterName).sog_std(end+1) = metrics.sog_std;
                        results.filterStats.(filterName).sog_max(end+1) = metrics.sog_max;
                        results.filterStats.(filterName).sog_min(end+1) = metrics.sog_min;
                    end
                    
                    % Update COG statistics
                    if isfield(metrics, 'cog_rmse') && ~isnan(metrics.cog_rmse)
                        results.filterStats.(filterName).cog_rmse(end+1) = metrics.cog_rmse;
                        results.filterStats.(filterName).cog_mean(end+1) = metrics.cog_mean;
                        results.filterStats.(filterName).cog_std(end+1) = metrics.cog_std;
                        results.filterStats.(filterName).cog_max(end+1) = metrics.cog_max;
                        results.filterStats.(filterName).cog_min(end+1) = metrics.cog_min;
                    end

                    results.filterStats.(filterName).runtime(end+1) = runtime;
                    
                    if verbose
                        fprintf('    %s: Pos=%.2fm, Vel=%.2fm/s, SOG=%.2fm/s, COG=%.2f°, Runtime=%.3fs\n', ...
                            filterName, metrics.position_rmse, ...
                            metrics.velocity_rmse, metrics.sog_rmse, metrics.cog_rmse, runtime);
                    end
                else
                    if verbose
                        fprintf('    %s: Invalid results (NaN RMSE)\n', filterName);
                    end
                end
                
            catch ME
                if verbose
                    fprintf('    Error in %s filter: %s\n', filterName, ME.message);
                end
                continue;
            end
        end
        
    catch ME
        if verbose
            fprintf('  Error processing run %d: %s\n', run, ME.message);
        end
        continue;
    end
end

% Add measurement statistics to results
results.measurementStats = measurementStats;

% STEP 5: CALCULATE SUMMARY STATISTICS (following runMonteCarloExample.m pattern)
if verbose
    fprintf('\n===== Filter Analysis Results =====\n');
    fprintf('Valid runs processed: %d/%d\n', validRunCount, numRuns);
end

% Calculate summary statistics for all filters
for i = 1:length(filterTypes)
    filterName = filterTypes{i};
    
    results.filterStats.(filterName).summary = struct();
    
    % Position statistics
    posRMSE = results.filterStats.(filterName).position_rmse;
    if ~isempty(posRMSE)
        results.filterStats.(filterName).summary.position = struct();
        results.filterStats.(filterName).summary.position.rmse_mean = mean(posRMSE);
        results.filterStats.(filterName).summary.position.rmse_std = std(posRMSE);
        results.filterStats.(filterName).summary.position.rmse_min = min(posRMSE);
        results.filterStats.(filterName).summary.position.rmse_max = max(posRMSE);
    end
    
    % Velocity statistics
    velRMSE = results.filterStats.(filterName).velocity_rmse;
    if ~isempty(velRMSE)
        results.filterStats.(filterName).summary.velocity = struct();
        results.filterStats.(filterName).summary.velocity.rmse_mean = mean(velRMSE);
        results.filterStats.(filterName).summary.velocity.rmse_std = std(velRMSE);
        results.filterStats.(filterName).summary.velocity.rmse_min = min(velRMSE);
        results.filterStats.(filterName).summary.velocity.rmse_max = max(velRMSE);
    end
    
    % SOG statistics
    sogRMSE = results.filterStats.(filterName).sog_rmse;
    if ~isempty(sogRMSE)
        results.filterStats.(filterName).summary.sog = struct();
        results.filterStats.(filterName).summary.sog.rmse_mean = mean(sogRMSE);
        results.filterStats.(filterName).summary.sog.rmse_std = std(sogRMSE);
        results.filterStats.(filterName).summary.sog.rmse_min = min(sogRMSE);
        results.filterStats.(filterName).summary.sog.rmse_max = max(sogRMSE);
    end
    
    % COG statistics
    cogRMSE = results.filterStats.(filterName).cog_rmse;
    if ~isempty(cogRMSE)
        results.filterStats.(filterName).summary.cog = struct();
        results.filterStats.(filterName).summary.cog.rmse_mean = mean(cogRMSE);
        results.filterStats.(filterName).summary.cog.rmse_std = std(cogRMSE);
        results.filterStats.(filterName).summary.cog.rmse_min = min(cogRMSE);
        results.filterStats.(filterName).summary.cog.rmse_max = max(cogRMSE);
    end
    
    % Runtime statistics
    runtimes = results.filterStats.(filterName).runtime;
    if ~isempty(runtimes)
        results.filterStats.(filterName).summary.runtime = struct();
        results.filterStats.(filterName).summary.runtime.mean = mean(runtimes);
        results.filterStats.(filterName).summary.runtime.std = std(runtimes);
        results.filterStats.(filterName).summary.runtime.min = min(runtimes);
        results.filterStats.(filterName).summary.runtime.max = max(runtimes);
    end
    
    % Display summary
    if verbose && isfield(results.filterStats.(filterName), 'summary')
        fprintf('\n%s Summary:\n', filterName);
        if isfield(results.filterStats.(filterName).summary, 'position')
            fprintf('  Position RMSE: %.2f ± %.2f m (%.2f to %.2f m)\n', ...
                results.filterStats.(filterName).summary.position.rmse_mean, ...
                results.filterStats.(filterName).summary.position.rmse_std, ...
                results.filterStats.(filterName).summary.position.rmse_min, ...
                results.filterStats.(filterName).summary.position.rmse_max);
        end
        if isfield(results.filterStats.(filterName).summary, 'velocity')
            fprintf('  Velocity RMSE: %.2f ± %.2f m/s (%.2f to %.2f m/s)\n', ...
                results.filterStats.(filterName).summary.velocity.rmse_mean, ...
                results.filterStats.(filterName).summary.velocity.rmse_std, ...
                results.filterStats.(filterName).summary.velocity.rmse_min, ...
                results.filterStats.(filterName).summary.velocity.rmse_max);
        end
        if isfield(results.filterStats.(filterName).summary, 'sog')
            fprintf('  SOG RMSE: %.2f ± %.2f m/s (%.2f to %.2f m/s)\n', ...
                results.filterStats.(filterName).summary.sog.rmse_mean, ...
                results.filterStats.(filterName).summary.sog.rmse_std, ...
                results.filterStats.(filterName).summary.sog.rmse_min, ...
                results.filterStats.(filterName).summary.sog.rmse_max);
        end
        if isfield(results.filterStats.(filterName).summary, 'cog')
            fprintf('  COG RMSE: %.2f ± %.2f° (%.2f to %.2f°)\n', ...
                results.filterStats.(filterName).summary.cog.rmse_mean, ...
                results.filterStats.(filterName).summary.cog.rmse_std, ...
                results.filterStats.(filterName).summary.cog.rmse_min, ...
                results.filterStats.(filterName).summary.cog.rmse_max);
        end
        if isfield(results.filterStats.(filterName).summary, 'runtime')
            fprintf('  Runtime: %.3f ± %.3f s\n', ...
                results.filterStats.(filterName).summary.runtime.mean, ...
                results.filterStats.(filterName).summary.runtime.std);
        end
    end
end

% STEP 6: GENERATE COMPARISON ANALYSIS
results.comparison = generateFilterComparison(results.filterStats, filterTypes);

% STEP 7: DISPLAY MEASUREMENT STATISTICS WITH SEPARATE METRICS
if verbose && ~isempty(measurementStats.position_rmse)
    fprintf('\n===== Measurement Quality Statistics =====\n');
    fprintf('Position measurement RMSE: %.2f ± %.2f m (%.2f to %.2f m)\n', ...
        mean(measurementStats.position_rmse), std(measurementStats.position_rmse), ...
        min(measurementStats.position_rmse), max(measurementStats.position_rmse));
    fprintf('Velocity measurement RMSE: %.2f ± %.2f m/s (%.2f to %.2f m/s)\n', ...
        mean(measurementStats.velocity_rmse), std(measurementStats.velocity_rmse), ...
        min(measurementStats.velocity_rmse), max(measurementStats.velocity_rmse));
    fprintf('SOG measurement RMSE: %.2f ± %.2f m/s (%.2f to %.2f m/s)\n', ...
        mean(measurementStats.sog_rmse), std(measurementStats.sog_rmse), ...
        min(measurementStats.sog_rmse), max(measurementStats.sog_rmse));
    fprintf('COG measurement RMSE: %.2f ± %.2f° (%.2f to %.2f°)\n', ...
        mean(measurementStats.cog_rmse), std(measurementStats.cog_rmse), ...
        min(measurementStats.cog_rmse), max(measurementStats.cog_rmse));
    fprintf('Average observations per run: %.1f\n', mean(measurementStats.observation_counts));
end

% STEP 8: GENERATE COMPREHENSIVE REPORT
if verbose
    generateFilterAnalysisReport(results);
end

% STEP 9: SAVE RESULTS IF REQUESTED (following runMonteCarloExample.m pattern)
if saveResults
    saveFilterAnalysisResults(results, verbose);
end

% STEP 10: FINAL SUMMARY (following runMonteCarloExample.m pattern)
if verbose
    fprintf('\n===== Filter Analysis Complete =====\n');
    fprintf('Best performing filter: %s\n', results.comparison.best_filter);
    if ~isnan(results.comparison.best_rmse)
        fprintf('Best RMSE: %.2f meters\n', results.comparison.best_rmse);
        
        % Display improvement over measurements
        if ~isempty(measurementStats.position_rmse)
            avgMeasurementRMSE = mean(measurementStats.position_rmse);
            improvement = ((avgMeasurementRMSE - results.comparison.best_rmse) / avgMeasurementRMSE) * 100;
            fprintf('Improvement over raw measurements: %.1f%% (%.2fm → %.2fm)\n', ...
                improvement, avgMeasurementRMSE, results.comparison.best_rmse);
        end
    end
    
    fprintf('\nUsage:\n');
    fprintf('  Check output/ directory for saved results and analysis plots\n');
    fprintf('  Results saved with timestamp for tracking\n');
end

end

function measurementMetrics = calculateMeasurementStatistics(aisData)
% Calculate measurement error statistics (noise in the sensor data) with separate metrics
measurementMetrics = struct();

% Position measurement error (built into the AIS dataset)
if ismember('pos_error', aisData.Properties.VariableNames)
    measurementMetrics.position_rmse = sqrt(mean(aisData.pos_error.^2));
    measurementMetrics.position_mean = mean(aisData.pos_error);
    measurementMetrics.position_std = std(aisData.pos_error);
else
    % Calculate manually if not available
    if all(ismember({'x', 'y', 'x_true', 'y_true'}, aisData.Properties.VariableNames))
        pos_errors = sqrt((aisData.x - aisData.x_true).^2 + (aisData.y - aisData.y_true).^2);
        measurementMetrics.position_rmse = sqrt(mean(pos_errors.^2));
        measurementMetrics.position_mean = mean(pos_errors);
        measurementMetrics.position_std = std(pos_errors);
    else
        measurementMetrics.position_rmse = NaN;
        measurementMetrics.position_mean = NaN;
        measurementMetrics.position_std = NaN;
    end
end

% Velocity measurement error (separate from SOG)
if ismember('vel_error', aisData.Properties.VariableNames)
    measurementMetrics.velocity_rmse = sqrt(mean(aisData.vel_error.^2));
    measurementMetrics.velocity_mean = mean(aisData.vel_error);
else
    % Calculate manually using velocity components
    if all(ismember({'vx_true', 'vy_true'}, aisData.Properties.VariableNames))
        % Calculate velocity from position derivatives if available
        if all(ismember({'x', 'y', 'time'}, aisData.Properties.VariableNames))
            dt = diff(aisData.time);
            vx_meas = [0; diff(aisData.x) ./ dt];
            vy_meas = [0; diff(aisData.y) ./ dt];
            vel_errors = sqrt((vx_meas - aisData.vx_true).^2 + (vy_meas - aisData.vy_true).^2);
            measurementMetrics.velocity_rmse = sqrt(mean(vel_errors.^2));
            measurementMetrics.velocity_mean = mean(vel_errors);
        else
            measurementMetrics.velocity_rmse = NaN;
            measurementMetrics.velocity_mean = NaN;
        end
    else
        measurementMetrics.velocity_rmse = NaN;
        measurementMetrics.velocity_mean = NaN;
    end
end

% SOG measurement error (separate from velocity)
if ismember('sog_error', aisData.Properties.VariableNames)
    measurementMetrics.sog_rmse = sqrt(mean(aisData.sog_error.^2));
    measurementMetrics.sog_mean = mean(aisData.sog_error);
else
    % Calculate manually
    if all(ismember({'SOG', 'sog_true'}, aisData.Properties.VariableNames))
        sog_errors = abs(aisData.SOG - aisData.sog_true);
        measurementMetrics.sog_rmse = sqrt(mean(sog_errors.^2));
        measurementMetrics.sog_mean = mean(sog_errors);
    else
        measurementMetrics.sog_rmse = NaN;
        measurementMetrics.sog_mean = NaN;
    end
end

% COG measurement error
if ismember('cog_error', aisData.Properties.VariableNames)
    measurementMetrics.cog_rmse = sqrt(mean(aisData.cog_error.^2)) * 180/pi; % Convert to degrees
    measurementMetrics.cog_mean = mean(aisData.cog_error) * 180/pi;
else
    % Calculate manually
    if all(ismember({'COG', 'cog_true'}, aisData.Properties.VariableNames))
        course_errors = abs(angdiff(deg2rad(aisData.COG), deg2rad(aisData.cog_true)));
        measurementMetrics.cog_rmse = sqrt(mean(course_errors.^2)) * 180/pi;
        measurementMetrics.cog_mean = mean(course_errors) * 180/pi;
    else
        measurementMetrics.cog_rmse = NaN;
        measurementMetrics.cog_mean = NaN;
    end
end
end

function [estimates, stats] = runFilterByName(filterName, aisData)
% Run the specified filter on the given AIS data with optimal Q values
pos_std = 15;
vel_std = 1;
acc_std = 0.1;

switch filterName
    case 'IMM_2'
        % 2-model IMM (CV + CTRV) with optimized Q values
        q_cv = 0.1;      % Optimal Q for CV model
        q_ctrv = 0.1;    % Optimal Q for CTRV model
        [estimates, stats] = runIMMEKF2(aisData, q_cv, q_ctrv, pos_std, vel_std);
        
    case 'IMM_3'
        % 3-model IMM (CV + CA + CTRV) with optimized Q values
        q_cv = 0.1;      % Optimal Q for CV model
        q_ca = 0.5;      % Optimal Q for CA model
        q_ctrv = 0.1;    % Optimal Q for CTRV model
        [estimates, stats] = runIMMFilterEKF(aisData, q_cv, q_ca, q_ctrv, pos_std, vel_std, acc_std);
        
    case 'EKF_CTRV'
        % Extended Kalman Filter with CTRV model
        q = 0.1;         % Optimal Q for EKF-CTRV
        [estimates, stats] = runEKFCTRV(aisData, q, pos_std, vel_std);
        
    otherwise
        error('Unknown filter type: %s', filterName);
end
end

function metrics = calculateFilterPerformance(aisData, estimates, stats)
% Calculate comprehensive performance metrics from the stats structure with separate metrics
metrics = struct();

% Extract metrics from the stats structure (which contains the actual calculated values)
if isfield(stats, 'posRMSE')
    metrics.position_rmse = stats.posRMSE;
    metrics.position_mean = stats.posMAE;  % Mean absolute error
    metrics.position_max = stats.posMax;
    metrics.position_std = 0;  % Will calculate if position errors available
    metrics.position_min = 0;  % Will calculate if position errors available
else
    % Fallback calculation if stats doesn't have the expected fields
    if ismember('x_true', aisData.Properties.VariableNames) && ismember('y_true', aisData.Properties.VariableNames) && ...
       isfield(estimates, 'x_est') && isfield(estimates, 'y_est')
        pos_errors = sqrt((estimates.x_est - aisData.x_true).^2 + (estimates.y_est - aisData.y_true).^2);
        metrics.position_rmse = sqrt(mean(pos_errors.^2));
        metrics.position_mean = mean(pos_errors);
        metrics.position_std = std(pos_errors);
        metrics.position_max = max(pos_errors);
        metrics.position_min = min(pos_errors);
        metrics.position_errors = pos_errors;
    else
        metrics.position_rmse = NaN;
        metrics.position_mean = NaN;
        metrics.position_std = NaN;
        metrics.position_max = NaN;
        metrics.position_min = NaN;
    end
end

% Velocity metrics (separate from SOG)
if isfield(stats, 'velRMSE')
    metrics.velocity_rmse = stats.velRMSE;
    metrics.velocity_mean = stats.velMAE;
    metrics.velocity_max = stats.velMax;
    metrics.velocity_std = 0;
    metrics.velocity_min = 0;
else
    % Fallback calculation using velocity components
    if all(ismember({'vx_true', 'vy_true'}, aisData.Properties.VariableNames)) && ...
       isfield(estimates, 'vx_est') && isfield(estimates, 'vy_est')
        vel_errors = sqrt((estimates.vx_est - aisData.vx_true).^2 + (estimates.vy_est - aisData.vy_true).^2);
        metrics.velocity_rmse = sqrt(mean(vel_errors.^2));
        metrics.velocity_mean = mean(vel_errors);
        metrics.velocity_std = std(vel_errors);
        metrics.velocity_max = max(vel_errors);
        metrics.velocity_min = min(vel_errors);
        metrics.velocity_errors = vel_errors;
    else
        metrics.velocity_rmse = NaN;
        metrics.velocity_mean = NaN;
        metrics.velocity_std = NaN;
        metrics.velocity_max = NaN;
        metrics.velocity_min = NaN;
    end
end

% SOG metrics (separate from velocity)
if isfield(stats, 'sogRMSE')
    metrics.sog_rmse = stats.sogRMSE;
    metrics.sog_mean = stats.sogMAE;
    metrics.sog_std = 0;
    metrics.sog_max = 0;
    metrics.sog_min = 0;
else
    % Fallback calculation
    if ismember('sog_true', aisData.Properties.VariableNames) && isfield(estimates, 'sog_est')
        sog_errors = abs(estimates.sog_est - aisData.sog_true);
        metrics.sog_rmse = sqrt(mean(sog_errors.^2));
        metrics.sog_mean = mean(sog_errors);
        metrics.sog_std = std(sog_errors);
        metrics.sog_max = max(sog_errors);
        metrics.sog_min = min(sog_errors);
        metrics.sog_errors = sog_errors;
    else
        metrics.sog_rmse = NaN;
        metrics.sog_mean = NaN;
        metrics.sog_std = NaN;
        metrics.sog_max = NaN;
        metrics.sog_min = NaN;
    end
end

% COG metrics (separate from course)
if isfield(stats, 'cogRMSE')
    metrics.cog_rmse = stats.cogRMSE;
    metrics.cog_mean = stats.cogMAE;
    metrics.cog_std = 0;
    metrics.cog_max = 0;
    metrics.cog_min = 0;
else
    % Fallback calculation
    if ismember('cog_true', aisData.Properties.VariableNames) && isfield(estimates, 'cog_est')
        course_errors = abs(angdiff(deg2rad(estimates.cog_est), deg2rad(aisData.cog_true)));
        metrics.cog_rmse = sqrt(mean(course_errors.^2)) * 180/pi; % Convert to degrees
        metrics.cog_mean = mean(course_errors) * 180/pi;
        metrics.cog_std = std(course_errors) * 180/pi;
        metrics.cog_max = max(course_errors) * 180/pi;
        metrics.cog_min = min(course_errors) * 180/pi;
        metrics.cog_errors = course_errors;
    else
        metrics.cog_rmse = NaN;
        metrics.cog_mean = NaN;
        metrics.cog_std = NaN;
        metrics.cog_max = NaN;
        metrics.cog_min = NaN;
    end
end

% Calculate position errors for detailed analysis if not already done
if ~isfield(metrics, 'position_errors') && ismember('x_true', aisData.Properties.VariableNames) && ...
   ismember('y_true', aisData.Properties.VariableNames) && isfield(estimates, 'x_est') && isfield(estimates, 'y_est')
    pos_errors = sqrt((estimates.x_est - aisData.x_true).^2 + (estimates.y_est - aisData.y_true).^2);
    metrics.position_errors = pos_errors;
    
    % Update std and min if not calculated
    if metrics.position_std == 0
        metrics.position_std = std(pos_errors);
    end
    if metrics.position_min == 0
        metrics.position_min = min(pos_errors);
    end
end

% Convergence analysis
if isfield(metrics, 'position_errors') && length(metrics.position_errors) > 10
    finalAvg = mean(metrics.position_errors(end-min(10, floor(length(metrics.position_errors)/3)):end));
    convergenceThreshold = 2 * finalAvg;
    convergenceIdx = find(metrics.position_errors < convergenceThreshold, 1);
    if ~isempty(convergenceIdx)
        metrics.convergence_time = convergenceIdx;
    end
end
end

function comparison = generateFilterComparison(filterStats, filterTypes)
% Generate comprehensive filter comparison with separate metrics
comparison = struct();

% Initialize comparison arrays for all metrics
pos_rmse_means = [];
pos_rmse_stds = [];
vel_rmse_means = [];
vel_rmse_stds = [];
sog_rmse_means = [];
sog_rmse_stds = [];
cog_rmse_means = [];
cog_rmse_stds = [];

for i = 1:length(filterTypes)
    filterName = filterTypes{i};
    
    % Position RMSE
    if isfield(filterStats.(filterName), 'summary') && isfield(filterStats.(filterName).summary, 'position')
        pos_rmse_means(i) = filterStats.(filterName).summary.position.rmse_mean;
        pos_rmse_stds(i) = filterStats.(filterName).summary.position.rmse_std;
    else
        pos_rmse_means(i) = NaN;
        pos_rmse_stds(i) = NaN;
    end
    
    % Velocity RMSE
    if isfield(filterStats.(filterName), 'summary') && isfield(filterStats.(filterName).summary, 'velocity')
        vel_rmse_means(i) = filterStats.(filterName).summary.velocity.rmse_mean;
        vel_rmse_stds(i) = filterStats.(filterName).summary.velocity.rmse_std;
    else
        vel_rmse_means(i) = NaN;
        vel_rmse_stds(i) = NaN;
    end
    
    % SOG RMSE
    if isfield(filterStats.(filterName), 'summary') && isfield(filterStats.(filterName).summary, 'sog')
        sog_rmse_means(i) = filterStats.(filterName).summary.sog.rmse_mean;
        sog_rmse_stds(i) = filterStats.(filterName).summary.sog.rmse_std;
    else
        sog_rmse_means(i) = NaN;
        sog_rmse_stds(i) = NaN;
    end
    
    % COG RMSE
    if isfield(filterStats.(filterName), 'summary') && isfield(filterStats.(filterName).summary, 'cog')
        cog_rmse_means(i) = filterStats.(filterName).summary.cog.rmse_mean;
        cog_rmse_stds(i) = filterStats.(filterName).summary.cog.rmse_std;
    else
        cog_rmse_means(i) = NaN;
        cog_rmse_stds(i) = NaN;
    end
end

% Store all metrics
comparison.position_rmse.means = pos_rmse_means;
comparison.position_rmse.stds = pos_rmse_stds;
comparison.velocity_rmse.means = vel_rmse_means;
comparison.velocity_rmse.stds = vel_rmse_stds;
comparison.sog_rmse.means = sog_rmse_means;
comparison.sog_rmse.stds = sog_rmse_stds;
comparison.cog_rmse.means = cog_rmse_means;
comparison.cog_rmse.stds = cog_rmse_stds;
comparison.filterTypes = filterTypes;

% Find best filter based on position RMSE (primary metric)
validIdx = ~isnan(pos_rmse_means);
if any(validIdx)
    [~, bestIdx] = min(pos_rmse_means(validIdx));
    validTypes = filterTypes(validIdx);
    comparison.best_filter = validTypes{bestIdx};
    comparison.best_rmse = min(pos_rmse_means(validIdx));
    comparison.relative_performance = pos_rmse_means / min(pos_rmse_means(validIdx));
else
    comparison.best_filter = 'None';
    comparison.best_rmse = NaN;
    comparison.relative_performance = pos_rmse_means;
end

% Find best filters for each metric
comparison.best_filters = struct();
if any(~isnan(pos_rmse_means))
    [~, idx] = min(pos_rmse_means);
    comparison.best_filters.position = filterTypes{idx};
end
if any(~isnan(vel_rmse_means))
    [~, idx] = min(vel_rmse_means);
    comparison.best_filters.velocity = filterTypes{idx};
end
if any(~isnan(sog_rmse_means))
    [~, idx] = min(sog_rmse_means);
    comparison.best_filters.sog = filterTypes{idx};
end
if any(~isnan(cog_rmse_means))
    [~, idx] = min(cog_rmse_means);
    comparison.best_filters.cog = filterTypes{idx};
end
end

function generateFilterAnalysisReport(results)
% Generate comprehensive visual analysis report including separate metrics for all measurements
figure;

filterNames = results.comparison.filterTypes;

% Plot 1: Position RMSE comparison
subplot(3, 5, 1);
pos_means = results.comparison.position_rmse.means;
pos_stds = results.comparison.position_rmse.stds;
validIdx = ~isnan(pos_means);
if any(validIdx)
    errorbar(find(validIdx), pos_means(validIdx), pos_stds(validIdx), 'o-', 'LineWidth', 2, 'MarkerSize', 8);
    set(gca, 'XTick', find(validIdx), 'XTickLabel', filterNames(validIdx));
end
ylabel('Position RMSE (m)');
title('Position RMSE Comparison');
grid on;

% Plot 2: Velocity RMSE comparison
subplot(3, 5, 2);
vel_means = results.comparison.velocity_rmse.means;
vel_stds = results.comparison.velocity_rmse.stds;
validIdx = ~isnan(vel_means);
if any(validIdx)
    errorbar(find(validIdx), vel_means(validIdx), vel_stds(validIdx), 's-', 'LineWidth', 2, 'MarkerSize', 8);
    set(gca, 'XTick', find(validIdx), 'XTickLabel', filterNames(validIdx));
end
ylabel('Velocity RMSE (m/s)');
title('Velocity RMSE Comparison');
grid on;

% Plot 3: SOG RMSE comparison
subplot(3, 5, 3);
sog_means = results.comparison.sog_rmse.means;
sog_stds = results.comparison.sog_rmse.stds;
validIdx = ~isnan(sog_means);
if any(validIdx)
    errorbar(find(validIdx), sog_means(validIdx), sog_stds(validIdx), '^-', 'LineWidth', 2, 'MarkerSize', 8);
    set(gca, 'XTick', find(validIdx), 'XTickLabel', filterNames(validIdx));
end
ylabel('SOG RMSE (m/s)');
title('SOG RMSE Comparison');
grid on;

% Plot 4: COG RMSE comparison
subplot(3, 5, 4);
cog_means = results.comparison.cog_rmse.means;
cog_stds = results.comparison.cog_rmse.stds;
validIdx = ~isnan(cog_means);
if any(validIdx)
    errorbar(find(validIdx), cog_means(validIdx), cog_stds(validIdx), 'd-', 'LineWidth', 2, 'MarkerSize', 8);
    set(gca, 'XTick', find(validIdx), 'XTickLabel', filterNames(validIdx));
end
ylabel('COG RMSE (°)');
title('COG RMSE Comparison');
grid on;

% Plot 5: All metrics bar chart
subplot(3, 5, 5);
if any(~isnan(pos_means))
    % Normalize metrics for comparison (position as reference)
    pos_norm = pos_means / min(pos_means(~isnan(pos_means)));
    vel_norm = vel_means / min(vel_means(~isnan(vel_means)));
    sog_norm = sog_means / min(sog_means(~isnan(sog_means)));
    cog_norm = cog_means / min(cog_means(~isnan(cog_means)));
    
    validFilters = ~isnan(pos_means);
    if any(validFilters)
        metrics_data = [pos_norm(validFilters)', vel_norm(validFilters)', ...
                       sog_norm(validFilters)', cog_norm(validFilters)'];
        bar(metrics_data);
        set(gca, 'XTickLabel', filterNames(validFilters));
        ylabel('Normalized RMSE');
        title('All Metrics Comparison');
        legend({'Position', 'Velocity', 'SOG', 'COG'}, 'Location', 'best');
    end
end
grid on;

% Plot 6: Measurement vs Filter Performance (Position)
subplot(3, 5, 6);
if isfield(results, 'measurementStats') && ~isempty(results.measurementStats.position_rmse)
    measurementRMSE = mean(results.measurementStats.position_rmse);
    data_to_plot = [measurementRMSE];
    labels = {'Measurements'};
    
    for i = 1:length(filterNames)
        if ~isnan(pos_means(i))
            data_to_plot(end+1) = pos_means(i);
            labels{end+1} = filterNames{i};
        end
    end
    
    bar(data_to_plot);
    set(gca, 'XTickLabel', labels, 'XTickLabelRotation', 45);
    ylabel('Position RMSE (m)');
    title('Position: Measurement vs Filter');
    grid on;
end

% Plot 7: Measurement vs Filter Performance (SOG)
subplot(3, 5, 7);
if isfield(results, 'measurementStats') && ~isempty(results.measurementStats.sog_rmse)
    measurementSOGRMSE = mean(results.measurementStats.sog_rmse);
    data_to_plot = [measurementSOGRMSE];
    labels = {'Measurements'};
    
    for i = 1:length(filterNames)
        if ~isnan(sog_means(i))
            data_to_plot(end+1) = sog_means(i);
            labels{end+1} = filterNames{i};
        end
    end
    
    bar(data_to_plot);
    set(gca, 'XTickLabel', labels, 'XTickLabelRotation', 45);
    ylabel('SOG RMSE (m/s)');
    title('SOG: Measurement vs Filter');
    grid on;
end

% Plot 8: Measurement vs Filter Performance (COG)
subplot(3, 5, 8);
if isfield(results, 'measurementStats') && ~isempty(results.measurementStats.cog_rmse)
    measurementCOGRMSE = mean(results.measurementStats.cog_rmse);
    data_to_plot = [measurementCOGRMSE];
    labels = {'Measurements'};
    
    for i = 1:length(filterNames)
        if ~isnan(cog_means(i))
            data_to_plot(end+1) = cog_means(i);
            labels{end+1} = filterNames{i};
        end
    end
    
    bar(data_to_plot);
    set(gca, 'XTickLabel', labels, 'XTickLabelRotation', 45);
    ylabel('COG RMSE (°)');
    title('COG: Measurement vs Filter');
    grid on;
end

% Plot 9: Runtime comparison
subplot(3, 5, 9);
runtime_means = [];
runtime_stds = [];
for i = 1:length(filterNames)
    filterName = filterNames{i};
    if isfield(results.filterStats.(filterName), 'summary') && isfield(results.filterStats.(filterName).summary, 'runtime')
        runtime_means(i) = results.filterStats.(filterName).summary.runtime.mean;
        runtime_stds(i) = results.filterStats.(filterName).summary.runtime.std;
    else
        runtime_means(i) = NaN;
        runtime_stds(i) = NaN;
    end
end

validIdx = ~isnan(runtime_means);
if any(validIdx)
    errorbar(find(validIdx), runtime_means(validIdx), runtime_stds(validIdx), 'p-', 'LineWidth', 2, 'MarkerSize', 8);
    set(gca, 'XTick', find(validIdx), 'XTickLabel', filterNames(validIdx));
end
ylabel('Runtime (s)');
title('Computational Performance');
grid on;

% Plot 10: Measurement quality statistics histograms
subplot(3, 5, 10);
if isfield(results, 'measurementStats') && ~isempty(results.measurementStats.position_rmse)
    histogram(results.measurementStats.position_rmse, min(20, length(results.measurementStats.position_rmse)));
    xlabel('Position Measurement RMSE (m)');
    ylabel('Frequency');
    title('Measurement Quality Distribution');
    grid on;
end

% Plot 11: Best filter error evolution
subplot(3, 5, 11);
if ~strcmp(results.comparison.best_filter, 'None')
    bestFilter = results.comparison.best_filter;
    runsToShow = min(3, size(results.rawResults, 1));
    plotCount = 0;
    for run = 1:size(results.rawResults, 1)
        for f = 1:length(filterNames)
            if strcmp(filterNames{f}, bestFilter) && ~isempty(results.rawResults{run, f})
                if isfield(results.rawResults{run, f}, 'position_errors')
                    plot(results.rawResults{run, f}.position_errors, 'DisplayName', sprintf('Run %d', run));
                    hold on;
                    plotCount = plotCount + 1;
                    if plotCount >= runsToShow
                        break;
                    end
                end
                break;
            end
        end
        if plotCount >= runsToShow
            break;
        end
    end
    if plotCount > 0
        xlabel('Time Step');
        ylabel('Position Error (m)');
        title(sprintf('Error Evolution - %s Filter', bestFilter));
        legend('Location', 'best');
    end
end
grid on;

% Plot 12: Summary text
subplot(3, 5, 12);
axis off;

summaryText = {'Filter Performance Summary:', ''};

% Add best performers for each metric
if isfield(results.comparison, 'best_filters')
    summaryText{end+1} = 'Best Performers:';
    if isfield(results.comparison.best_filters, 'position')
        summaryText{end+1} = sprintf('  Position: %s', results.comparison.best_filters.position);
    end
    if isfield(results.comparison.best_filters, 'velocity')
        summaryText{end+1} = sprintf('  Velocity: %s', results.comparison.best_filters.velocity);
    end
    if isfield(results.comparison.best_filters, 'sog')
        summaryText{end+1} = sprintf('  SOG: %s', results.comparison.best_filters.sog);
    end
    if isfield(results.comparison.best_filters, 'cog')
        summaryText{end+1} = sprintf('  COG: %s', results.comparison.best_filters.cog);
    end
    summaryText{end+1} = '';
end

% Add detailed performance for each filter
for i = 1:length(filterNames)
    filterName = filterNames{i};
    if ~isnan(pos_means(i))
        summaryText{end+1} = sprintf('%s:', filterName);
        summaryText{end+1} = sprintf('  Pos: %.2f±%.2f m', pos_means(i), pos_stds(i));
        if ~isnan(vel_means(i))
            summaryText{end+1} = sprintf('  Vel: %.2f±%.2f m/s', vel_means(i), vel_stds(i));
        end
        if ~isnan(sog_means(i))
            summaryText{end+1} = sprintf('  SOG: %.2f±%.2f m/s', sog_means(i), sog_stds(i));
        end
        if ~isnan(cog_means(i))
            summaryText{end+1} = sprintf('  COG: %.2f±%.2f°', cog_means(i), cog_stds(i));
        end
        summaryText{end+1} = '';
    end
end

% Add measurement statistics
if isfield(results, 'measurementStats') && ~isempty(results.measurementStats.position_rmse)
    summaryText{end+1} = 'Measurement Quality:';
    summaryText{end+1} = sprintf('  Pos: %.2f±%.2f m', ...
        mean(results.measurementStats.position_rmse), std(results.measurementStats.position_rmse));
    summaryText{end+1} = sprintf('  Vel: %.2f±%.2f m/s', ...
        mean(results.measurementStats.velocity_rmse), std(results.measurementStats.velocity_rmse));
    summaryText{end+1} = sprintf('  SOG: %.2f±%.2f m/s', ...
        mean(results.measurementStats.sog_rmse), std(results.measurementStats.sog_rmse));
    summaryText{end+1} = sprintf('  COG: %.2f±%.2f°', ...
        mean(results.measurementStats.cog_rmse), std(results.measurementStats.cog_rmse));
end

text(0.1, 0.9, summaryText, 'Units', 'normalized', 'VerticalAlignment', 'top', ...
     'FontName', 'FixedWidth', 'FontSize', 8);

sgtitle(sprintf('Monte Carlo Filter Analysis - Separate Metrics (%d runs)', results.metadata.numRuns));

% Save figure
if ~exist('output', 'dir')
    mkdir('output');
end
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
filename = sprintf('output/best_filters_separate_metrics_%s.png', timestamp);
saveas(gcf, filename);
fprintf('Filter analysis report with separate metrics saved to: %s\n', filename);
end

function saveFilterAnalysisResults(results, verbose)
% Save comprehensive results to files including separate measurement statistics
if ~exist('output', 'dir')
    mkdir('output');
end

timestamp = datestr(now, 'yyyymmdd_HHMMSS');

% Save MATLAB results
filename = sprintf('output/best_filters_results_%s.mat', timestamp);
save(filename, 'results', '-v7.3');
if verbose
    fprintf('Filter analysis results saved to: %s\n', filename);
end

% Save CSV summary with separate metrics
csvFilename = sprintf('output/best_filters_separate_metrics_%s.csv', timestamp);

filterNames = results.comparison.filterTypes;

summaryTable = table();
summaryTable.Filter = filterNames';

% Position metrics
summaryTable.Position_RMSE_Mean = results.comparison.position_rmse.means';
summaryTable.Position_RMSE_Std = results.comparison.position_rmse.stds';

% Velocity metrics
summaryTable.Velocity_RMSE_Mean = results.comparison.velocity_rmse.means';
summaryTable.Velocity_RMSE_Std = results.comparison.velocity_rmse.stds';

% SOG metrics
summaryTable.SOG_RMSE_Mean = results.comparison.sog_rmse.means';
summaryTable.SOG_RMSE_Std = results.comparison.sog_rmse.stds';

% COG metrics
summaryTable.COG_RMSE_Mean = results.comparison.cog_rmse.means';
summaryTable.COG_RMSE_Std = results.comparison.cog_rmse.stds';

% Runtime metrics
runtime_means = [];
runtime_stds = [];
for i = 1:length(filterNames)
    filterName = filterNames{i};
    if isfield(results.filterStats.(filterName), 'summary') && isfield(results.filterStats.(filterName).summary, 'runtime')
        runtime_means(i) = results.filterStats.(filterName).summary.runtime.mean;
        runtime_stds(i) = results.filterStats.(filterName).summary.runtime.std;
    else
        runtime_means(i) = NaN;
        runtime_stds(i) = NaN;
    end
end

summaryTable.Runtime_Mean = runtime_means';
summaryTable.Runtime_Std = runtime_stds';
summaryTable.Relative_Performance = results.comparison.relative_performance';

% Add measurement statistics if available
if isfield(results, 'measurementStats') && ~isempty(results.measurementStats.position_rmse)
    summaryTable.Measurement_Pos_RMSE = repmat(mean(results.measurementStats.position_rmse), length(filterNames), 1);
    summaryTable.Measurement_Vel_RMSE = repmat(mean(results.measurementStats.velocity_rmse), length(filterNames), 1);
    summaryTable.Measurement_SOG_RMSE = repmat(mean(results.measurementStats.sog_rmse), length(filterNames), 1);
    summaryTable.Measurement_COG_RMSE = repmat(mean(results.measurementStats.cog_rmse), length(filterNames), 1);
end

writetable(summaryTable, csvFilename);
if verbose
    fprintf('Summary statistics with separate metrics saved to: %s\n', csvFilename);
end

% Save detailed measurement statistics
measurementCsvFilename = sprintf('output/measurement_statistics_%s.csv', timestamp);
if isfield(results, 'measurementStats') && ~isempty(results.measurementStats.position_rmse)
    measurementTable = table();
    measurementTable.Run = results.measurementStats.valid_runs';
    measurementTable.Position_RMSE = results.measurementStats.position_rmse';
    measurementTable.Velocity_RMSE = results.measurementStats.velocity_rmse';
    measurementTable.SOG_RMSE = results.measurementStats.sog_rmse';
    measurementTable.COG_RMSE = results.measurementStats.cog_rmse';
    measurementTable.Observation_Count = results.measurementStats.observation_counts';
    
    writetable(measurementTable, measurementCsvFilename);
    if verbose
        fprintf('Detailed measurement statistics saved to: %s\n', measurementCsvFilename);
    end
end
end