function [aisData, isValidRun] = extractAndConvertRunData(monteCarloResults, run, config)
% EXTRACTANDCONVERTRUNDATA - Extract and convert Monte Carlo run data to AIS format
%
% Inputs:
%   monteCarloResults - Monte Carlo simulation results
%   run - Run number
%   config - Configuration structure
%
% Outputs:
%   aisData - Converted AIS dataset
%   isValidRun - Boolean indicating if run is valid

try
    % Extract Monte Carlo results for this run
    groundTruth = monteCarloResults.runs{run}.groundTruth;
    observations = monteCarloResults.runs{run}.observations;
    metadata = monteCarloResults.runs{run}.metadata;
    
    % Convert to AIS dataset format
    aisData = generateAISDataset(groundTruth, observations, metadata);
    

    
    % Skip runs with insufficient data
    if height(aisData) < 20
        if config.verbose
            fprintf('  Skipping run %d: Insufficient data (%d points)\n', run, height(aisData));
        end
        isValidRun = false;
        return;
    end
    
    isValidRun = true;
    
catch ME
    if config.verbose
        fprintf('  Error extracting run %d: %s\n', run, ME.message);
    end
    aisData = [];
    isValidRun = false;
end
end

function measurementStats = updateMeasurementStatistics(measurementStats, aisData, run, config)
% UPDATEMEASUREMENTSTATISTICS - Update measurement statistics for a run
%
% Inputs:
%   measurementStats - Current measurement statistics
%   aisData - AIS data for this run
%   run - Run number
%   config - Configuration structure
%
% Outputs:
%   measurementStats - Updated measurement statistics

% Calculate measurement statistics for this run
measurementMetrics = calculateMeasurementStatistics(aisData);

measurementStats.position_rmse(end+1) = measurementMetrics.position_rmse;
measurementStats.velocity_rmse(end+1) = measurementMetrics.velocity_rmse;
measurementStats.sog_rmse(end+1) = measurementMetrics.sog_rmse;
measurementStats.cog_rmse(end+1) = measurementMetrics.cog_rmse;
measurementStats.observation_counts(end+1) = height(aisData);
measurementStats.valid_runs(end+1) = run;
end

function measurementMetrics = calculateMeasurementStatistics(aisData)
% CALCULATEMEASUREMENTSTATISTICS - Calculate measurement error statistics
%
% Inputs:
%   aisData - AIS dataset
%
% Outputs:
%   measurementMetrics - Structure with measurement error metrics

measurementMetrics = struct();

% Position measurement error
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

% Velocity measurement error
if ismember('vel_error', aisData.Properties.VariableNames)
    measurementMetrics.velocity_rmse = sqrt(mean(aisData.vel_error.^2));
    measurementMetrics.velocity_mean = mean(aisData.vel_error);
else
    % Calculate manually using velocity components
    if all(ismember({'vx_true', 'vy_true'}, aisData.Properties.VariableNames))
        % Calculate velocity from position derivatives if available
        if all(ismember({'vx', 'vy'}, aisData.Properties.VariableNames))
            vel_errors = sqrt((aisData.vx - aisData.vx_true).^2 + (aisData.vy - aisData.vy_true).^2);
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

% SOG measurement error
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
% RUNFILTERBYNAME - Run the specified filter on the given AIS data
%
% Inputs:
%   filterName - Name of the filter to run
%   aisData - AIS data
%
% Outputs:
%   estimates - Filter estimates
%   stats - Filter statistics

% Standard noise parameters
pos_std = 15;
vel_std = 1;
acc_std = 0.1;

switch filterName
    case 'IMM_2'
        q_cv = 0.01;      % Optimal Q for CV model
        q_ctrv = 0.051;    % Optimal Q for CTRV model
        [estimates, stats] = runIMMEKF2(aisData, q_cv, q_ctrv, pos_std, vel_std);
    case 'IMM_3'
        q_cv = 0.014;      % Optimal Q for CV model
        q_ca = 0.010;      % Optimal Q for CA model
        q_ctrv = 0.030;    % Optimal Q for CTRV model
        [estimates, stats] = runIMMFilterEKF(aisData, q_cv, q_ca, q_ctrv, pos_std, vel_std, acc_std);
        
    case 'EKF_CTRV'
        q = 0.030;         % Optimal Q for EKF-CTRV
        [estimates, stats] = runEKFCTRV(aisData, q, pos_std, vel_std);

    case 'EKF_CV'
        q = 0.014;         % Optimal Q for EKF-CV
        [estimates, stats] = runEKFCV(aisData, q, pos_std, vel_std);
        
     case 'EKF_CA'
        q = 0.010;         % Optimal Q for EKF-CA
        [estimates, stats] = runEKFCA(aisData, q, pos_std, vel_std);
        
     case 'UKF_CTRV'
        q = 0.1;         % Optimal Q for UKF-CTRV
        [estimates, stats] = runUKFCTRV(aisData, q, pos_std, vel_std);
        
     case 'UKF_CV'
        q = 0.060;         % Optimal Q for UKF-CV
        [estimates, stats] = runUKFCV(aisData, q, pos_std, vel_std);
        
     case 'KF'
        q = 0.020;         % Optimal Q for KF
        [estimates, stats] = runKF(aisData, q, pos_std, vel_std);
       
    otherwise
        error('Unknown filter type: %s', filterName);
end
end

function metrics = calculateFilterPerformance(aisData, estimates, stats)
% CALCULATEFILTERPERFORMANCE - Calculate comprehensive performance metrics
%
% Inputs:
%   aisData - AIS data
%   estimates - Filter estimates  
%   stats - Filter statistics
%
% Outputs:
%   metrics - Performance metrics structure

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

% Velocity metrics
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

% SOG metrics
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

% COG metrics
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