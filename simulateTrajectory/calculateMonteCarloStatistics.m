function summary = calculateMonteCarloStatistics(results)
% CALCULATEMONTECARLSTATISTICS - Compute error statistics across all Monte Carlo runs
%
% Analyzes results from multiple Monte Carlo runs to calculate comprehensive
% error statistics. Compares sensor observations against ground truth to
% quantify measurement accuracy and noise characteristics.
%
% ERROR METRICS CALCULATED:
%   - Position errors: Euclidean distance between true and measured positions
%   - Velocity errors: Absolute difference between true and measured speeds
%   - Course errors: Angular difference between true and measured headings
%
% STATISTICS PROVIDED:
%   - Mean: Average error magnitude
%   - Standard deviation: Error variability/consistency
%   - RMSE: Root mean square error (penalizes large errors more)
%
% INPUTS:
%   results - Monte Carlo results structure containing all simulation runs
%
% OUTPUTS:
%   summary - Statistics structure with position, velocity, and course metrics

numRuns = length(results.runs);

% DEBUG OUTPUT - Track processing progress
fprintf('=== CALCULATE STATISTICS DEBUG ===\n');
fprintf('Processing %d Monte Carlo runs...\n', numRuns);

% Initialize error collection arrays
posErrors = [];    % Position error magnitudes (meters)
velErrors = [];    % Velocity error magnitudes (m/s)
courseErrors = []; % Course error magnitudes (radians)

% MAIN PROCESSING LOOP - Extract errors from each Monte Carlo run
for run = 1:numRuns
    try
        % Extract ground truth and observations for this run
        gt = results.runs{run}.groundTruth;   % Perfect trajectory data
        obs = results.runs{run}.observations; % Noisy sensor measurements
        
        % VALIDATION - Ensure essential data exists
        if isempty(gt) || isempty(obs)
            fprintf('Run %d: Skipping - missing ground truth or observations\n', run);
            continue;
        end
        
        % FILTER VALID OBSERVATIONS - Exclude dropped/missing AIS reports
        validObs = obs.available;          % Boolean array of successful AIS transmissions
        numValidObs = sum(validObs);       % Count of successful observations
        
        fprintf('Run %d: %d valid observations out of %d total\n', run, numValidObs, length(validObs));
        
        if numValidObs == 0
            fprintf('Run %d: Skipping - no valid observations\n', run);
            continue;
        end
        
        % GET OBSERVATION INDICES - Find which time steps have valid AIS reports
        validIndices = find(validObs);     % Indices of successful observations
        
        % MAP TO GROUND TRUTH - Find corresponding ground truth time steps
        gtIndices = obs.groundTruthIndices(validIndices); % Ground truth indices for valid observations
        
        % BOUNDS CHECKING - Ensure ground truth indices are valid
        maxGtIndex = size(gt.position, 2); % Total ground truth time steps
        validGtMask = gtIndices <= maxGtIndex & gtIndices >= 1; % Valid index mask
        
        if sum(validGtMask) == 0
            fprintf('Run %d: Skipping - no valid ground truth indices\n', run);
            continue;
        end
        
        % FINAL INDEX FILTERING - Keep only valid observation-ground truth pairs
        finalValidIndices = validIndices(validGtMask);  % Final valid observation indices
        finalGtIndices = gtIndices(validGtMask);        % Final valid ground truth indices
        
        fprintf('Run %d: Processing %d valid observation pairs\n', run, length(finalValidIndices));
        
        % ERROR CALCULATION LOOP - Compute errors for each valid observation
        runPosErrors = 0;  % Counter for this run
        
        for i = 1:length(finalValidIndices)
            obsIdx = finalValidIndices(i);  % Index in observation arrays
            gtIdx = finalGtIndices(i);      % Index in ground truth arrays
            
            % POSITION ERROR - Euclidean distance between true and observed positions
            obsPos = obs.position(:, obsIdx);    % Observed position [x,y] (meters)
            gtPos = gt.position(:, gtIdx);       % True position [x,y] (meters)
            posError = norm(obsPos - gtPos);     % Distance error (meters)
            posErrors = [posErrors, posError];
            
            % VELOCITY ERROR - Absolute difference between true and observed speeds
            obsVel = obs.velocity(obsIdx);       % Observed speed (m/s)
            gtSpeed = gt.speed(gtIdx);           % True speed (m/s)
            velError = abs(obsVel - gtSpeed);    % Speed error magnitude (m/s)
            velErrors = [velErrors, velError];
            
            % COURSE ERROR - Angular difference between true and observed headings
            obsCourse = obs.course(obsIdx);      % Observed heading (radians)
            gtCourse = gt.heading(gtIdx);        % True heading (radians)
            courseError = abs(angdiff(obsCourse, gtCourse)); % Angular error (radians)
            courseErrors = [courseErrors, courseError];
            
            runPosErrors = runPosErrors + 1;
        end
        
        fprintf('Run %d: Added %d position errors\n', run, runPosErrors);
        
    catch ME
        fprintf('Run %d: Error - %s\n', run, ME.message);
        continue;
    end
end

fprintf('Total errors collected: %d position, %d velocity, %d course\n', ...
    length(posErrors), length(velErrors), length(courseErrors));

% COMPILE SUMMARY STATISTICS
summary = struct();

% POSITION ERROR STATISTICS
if ~isempty(posErrors)
    summary.position = struct();
    summary.position.mean = mean(posErrors);           % Average position error (m)
    summary.position.std = std(posErrors);             % Position error variability (m)
    summary.position.rmse = sqrt(mean(posErrors.^2));  % Root mean square error (m)
    summary.position.max = max(posErrors);             % Max error (m)
    summary.position.min = min(posErrors);             % Min error (m)
else
    summary.position = struct('mean', 0, 'std', 0, 'rmse', 0);
end

% VELOCITY ERROR STATISTICS  
if ~isempty(velErrors)
    summary.velocity = struct();
    summary.velocity.mean = mean(velErrors);           % Average speed error (m/s)
    summary.velocity.std = std(velErrors);             % Speed error variability (m/s)
    summary.velocity.rmse = sqrt(mean(velErrors.^2));  % Root mean square error (m/s)
    summary.velocity.max = max(posErrors);             % Max error (m/s)
    summary.velocity.min = min(posErrors);             % Min error (m/s)
else
    summary.velocity = struct('mean', 0, 'std', 0, 'rmse', 0);
end

% COURSE ERROR STATISTICS
if ~isempty(courseErrors)
    summary.course = struct();
    summary.course.mean = mean(courseErrors);           % Average heading error (rad)
    summary.course.std = std(courseErrors);             % Heading error variability (rad)
    summary.course.rmse = sqrt(mean(courseErrors.^2));  % Root mean square error (rad)
    summary.course.max = max(posErrors);                % Max error (m)
    summary.course.min = min(posErrors);                % Min error (m)
else
    summary.course = struct('mean', 0, 'std', 0, 'rmse', 0);
end

% METADATA
summary.numRuns = numRuns;                    % Total Monte Carlo runs processed
summary.totalObservations = length(posErrors); % Total valid observations across all runs

fprintf('Final summary: %d total observations processed\n', summary.totalObservations);
fprintf('=== END CALCULATE STATISTICS DEBUG ===\n');

% INTERPRETATION NOTES:
% - Lower RMSE values indicate better sensor/measurement accuracy
% - Position RMSE should be close to measurement noise parameter
% - Velocity and course RMSE depend on motion complexity and noise levels
% - Standard deviation shows measurement consistency across different scenarios

end