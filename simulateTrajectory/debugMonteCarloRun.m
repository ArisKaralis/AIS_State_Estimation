function debugMonteCarloRun()
% DEBUGMONTECARLORUN - Diagnostic tool for troubleshooting Monte Carlo simulation
%
% Runs a single Monte Carlo iteration with detailed logging to diagnose
% issues with trajectory generation, observation creation, or data processing.
% Useful for understanding why Monte Carlo statistics might be zero or
% identifying problems with parameter settings.
%
% DIAGNOSTIC CHECKS:
%   - Parameter validation and expected observation counts
%   - Ground truth trajectory generation verification
%   - AIS observation creation and dropout analysis
%   - Data structure integrity and bounds checking
%   - AIS dataset conversion validation
%
% OUTPUTS:
%   - Detailed console output with diagnostic information
%   - No files saved (this is purely for debugging)

clear all;

% STEP 1: PARAMETER SETUP
% Use same base parameters as main simulation for consistency
params = getDefaultSimulationParameters();

% Configure to match main simulation parameters
% NOTE: Update these to match your runMonteCarloExample settings
params.totalDuration = 3600;               % 1 hour simulation
params.aisReportInterval = 10;             % AIS reports every 10 seconds  
params.measurementNoise.position = 25;     % Position measurement noise (meters)
params.measurementNoise.velocity = 1.5;    % Velocity measurement noise (m/s)
params.modeTransition.probability = 0.03;  % Mode change probability per time step

% Set reproducible seed for consistent debugging
rng(42);

% STEP 2: PARAMETER ANALYSIS
fprintf('=== Debugging Single Monte Carlo Run ===\n');
fprintf('Input Parameters:\n');
fprintf('  Total Duration: %d seconds (%.1f minutes)\n', params.totalDuration, params.totalDuration/60);
fprintf('  AIS Report Interval: %d seconds\n', params.aisReportInterval);
fprintf('  Expected observations: %d\n', params.totalDuration / params.aisReportInterval);
fprintf('  Dropout probability: %.3f (%.1f%% expected losses)\n', ...
    params.dropoutProbability, params.dropoutProbability*100);
fprintf('  Position noise std: %.1f meters\n', params.measurementNoise.position);
fprintf('  Velocity noise std: %.1f m/s\n', params.measurementNoise.velocity);

% STEP 3: PARAMETER RANDOMIZATION TEST
% Apply same randomization process as Monte Carlo simulation
fprintf('\n=== Parameter Randomization ===\n');
runParams = randomizeSimulationParameters(params);

fprintf('Randomized parameters:\n');
fprintf('  Max speed: %.1f m/s (was %.1f)\n', runParams.vesselDynamics.maxSpeed, params.vesselDynamics.maxSpeed);
fprintf('  Max acceleration: %.3f m/s² (was %.3f)\n', runParams.vesselDynamics.maxAcceleration, params.vesselDynamics.maxAcceleration);
fprintf('  Position noise: %.1f m (was %.1f)\n', runParams.measurementNoise.position, params.measurementNoise.position);
fprintf('  Dropout probability: %.3f (was %.3f)\n', runParams.dropoutProbability, params.dropoutProbability);

% STEP 4: TRAJECTORY GENERATION TEST
fprintf('\n=== Trajectory Generation ===\n');
[groundTruth, observations, metadata] = generateSingleRun(runParams);

% Validate ground truth generation
fprintf('Ground Truth Results:\n');
fprintf('  Ground truth steps: %d\n', size(groundTruth.position, 2));
fprintf('  Time span: %.1f seconds\n', size(groundTruth.position, 2) * runParams.timeStep);

if size(groundTruth.position, 2) > 0
    fprintf('  Position range: X[%.1f, %.1f], Y[%.1f, %.1f] meters\n', ...
        min(groundTruth.position(1,:)), max(groundTruth.position(1,:)), ...
        min(groundTruth.position(2,:)), max(groundTruth.position(2,:)));
    fprintf('  Speed range: %.1f to %.1f m/s\n', min(groundTruth.speed), max(groundTruth.speed));
    fprintf('  Total distance: %.1f km\n', sum(sqrt(sum(diff(groundTruth.position,1,2).^2)))/1000);
end

% STEP 5: OBSERVATION ANALYSIS
fprintf('\nObservation Results:\n');
fprintf('  Total observation slots: %d\n', length(observations.time));
fprintf('  Available observations: %d\n', sum(observations.available));
fprintf('  Dropped observations: %d\n', sum(~observations.available));
fprintf('  Dropout rate: %.1f%% (expected ~%.1f%%)\n', ...
    100*(1-sum(observations.available)/length(observations.available)), ...
    runParams.dropoutProbability*100);

% STEP 6: OBSERVATION QUALITY CHECK
if length(observations.time) > 0
    fprintf('\nObservation Quality:\n');
    fprintf('  Time range: %.1f to %.1f seconds\n', min(observations.time), max(observations.time));
    
    % Check position data
    validIdx = observations.available;
    if sum(validIdx) > 0
        validPositions = observations.position(:, validIdx);
        fprintf('  Position range X: %.1f to %.1f meters\n', ...
            min(validPositions(1,:)), max(validPositions(1,:)));
        fprintf('  Position range Y: %.1f to %.1f meters\n', ...
            min(validPositions(2,:)), max(validPositions(2,:)));
        
        validVelocities = observations.velocity(validIdx);
        fprintf('  Velocity range: %.1f to %.1f m/s\n', ...
            min(validVelocities), max(validVelocities));
        
        validCourses = observations.course(validIdx);
        fprintf('  Course range: %.1f to %.1f degrees\n', ...
            rad2deg(min(validCourses)), rad2deg(max(validCourses)));
    end
else
    fprintf('  ERROR: No observations generated!\n');
    fprintf('  Check: totalDuration, aisReportInterval, and parameter settings\n');
    return;
end

% STEP 7: AIS DATASET CONVERSION TEST
fprintf('\n=== AIS Dataset Conversion ===\n');
if sum(observations.available) > 0
    try
        aisData = generateAISDataset(groundTruth, observations, metadata);
        fprintf('AIS Dataset Results:\n');
        fprintf('  Rows generated: %d\n', height(aisData));
        
        if height(aisData) > 0
            fprintf('  Position errors: %.2f to %.2f m (mean: %.2f m)\n', ...
                min(aisData.pos_error), max(aisData.pos_error), mean(aisData.pos_error));
            fprintf('  Speed errors: %.2f to %.2f m/s (mean: %.2f m/s)\n', ...
                min(aisData.sog_error), max(aisData.sog_error), mean(aisData.sog_error));
            fprintf('  Course errors: %.3f to %.3f rad (mean: %.3f rad)\n', ...
                min(aisData.cog_error), max(aisData.cog_error), mean(aisData.cog_error));
            
            % Show motion mode distribution
            if ismember('mode', aisData.Properties.VariableNames)
                modeStats = groupcounts(aisData, 'mode');
                fprintf('  Motion modes: ');
                for i = 1:height(modeStats)
                    fprintf('%s(%d) ', char(modeStats.mode(i)), modeStats.GroupCount(i));
                end
                fprintf('\n');
            end
        end
        
    catch ME
        fprintf('ERROR in AIS dataset generation: %s\n', ME.message);
        fprintf('This suggests an issue with data alignment or indexing\n');
    end
else
    fprintf('ERROR: No available observations to create AIS dataset!\n');
    fprintf('All observations were dropped - check dropout probability settings\n');
end

% STEP 8: DIAGNOSTIC SUMMARY
fprintf('\n=== Diagnostic Summary ===\n');
if sum(observations.available) > 0 && exist('aisData', 'var') && height(aisData) > 0
    fprintf('✓ Simulation pipeline working correctly\n');
    fprintf('✓ Ground truth generation: OK\n');
    fprintf('✓ Observation generation: OK\n');
    fprintf('✓ AIS dataset conversion: OK\n');
else
    fprintf('⚠ Issues detected in simulation pipeline\n');
    fprintf('Check the error messages above and verify:\n');
    fprintf('  - Parameter values are reasonable\n');
    fprintf('  - Dropout probability is not too high\n');
    fprintf('  - Ground truth generation is working\n');
    fprintf('  - Data structure alignment is correct\n');
end

fprintf('\n=== Debug Complete ===\n');

% TROUBLESHOOTING GUIDE:
% - If no observations: Check aisReportInterval and totalDuration
% - If all dropped: Reduce dropoutProbability parameter  
% - If AIS conversion fails: Check data structure alignment
% - If positions are NaN: Check noise parameters and initial conditions
% - If speeds are invalid: Check vessel dynamics limits

end