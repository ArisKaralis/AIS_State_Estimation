function simulateSpecificAISTrack()
% SIMULATESPECIFICAISTRACK - Create AIS track with specific motion patterns
% Generates realistic vessel movement with defined maneuver sequence
%
% This function:
% 1. Defines simulation parameters
% 2. Generates ground truth trajectory with specific motion patterns
% 3. Adds realistic measurement noise
% 4. Saves data and creates visualization plots
%
% Motion sequence:
% 1. Constant velocity
% 2. Constant acceleration
% 3. 45-degree turn
% 4. Constant velocity
% 5. 90-degree turn
% 6. Constant deceleration
% 7. Final constant velocity

% Get simulation parameters
params = defineSimulationParameters();

% Generate ground truth trajectory
[x_true, y_true, vx_true, vy_true, sog_true, cog_true, t, segmentInfo, segStarts] = ...
    generateGroundTruthTrajectory(params);

% Generate noisy AIS measurements
[x_meas, y_meas, sog_meas, cog_meas] = ...
    generateNoisyMeasurements(x_true, y_true, sog_true, cog_true, params);

% Calculate RMSE between measurements and ground truth
[rmse_stats, segment_rmse] = calculateMeasurementRMSE(x_true, y_true, sog_true, cog_true, ...
                                                    x_meas, y_meas, sog_meas, cog_meas, segmentInfo);

% Create data table with ground truth and measurements
aisData = createDataTable(t, x_true, y_true, vx_true, vy_true, sog_true, cog_true, ...
                         x_meas, y_meas, sog_meas, cog_meas, segmentInfo, rmse_stats);

% Save data to CSV
outputPath = saveDataToCSV(aisData);

% Create and save visualization plots
createVisualizationPlots(aisData, x_true, y_true, x_meas, y_meas, ...
                        sog_true, sog_meas, cog_true, cog_meas, t, segStarts, params, rmse_stats, segment_rmse);

% Report completion
fprintf('Generated %d AIS reports with specific motion patterns\n', params.numSamples);
fprintf('Data saved to: %s\n', outputPath);
fprintf('Total trajectory duration: %.1f minutes\n', params.totalDuration/60);

% Display RMSE statistics
fprintf('\n===== Measurement Error Statistics =====\n');
fprintf('Position RMSE: %.2f meters\n', rmse_stats.position);
fprintf('Speed RMSE: %.2f m/s\n', rmse_stats.speed);
fprintf('Course RMSE: %.2f degrees\n', rmse_stats.course);
end

function [rmse_stats, segment_rmse] = calculateMeasurementRMSE(x_true, y_true, sog_true, cog_true, ...
                                                          x_meas, y_meas, sog_meas, cog_meas, segmentInfo)
% Calculate RMSE between measurements and ground truth, both overall and per segment

% Overall RMSE
pos_error = sqrt((x_meas - x_true).^2 + (y_meas - y_true).^2);
sog_error = abs(sog_meas - sog_true);

% Handle course angle difference correctly (considering circular nature)
cog_true_rad = deg2rad(cog_true);
cog_meas_rad = deg2rad(cog_meas);
cog_error = abs(rad2deg(angdiff(cog_meas_rad, cog_true_rad)));

% Overall RMSE statistics
rmse_stats = struct();
rmse_stats.position = sqrt(mean(pos_error.^2));
rmse_stats.speed = sqrt(mean(sog_error.^2));
rmse_stats.course = sqrt(mean(cog_error.^2));

% Per-segment RMSE
segments = unique(segmentInfo);
num_segments = length(segments);
segment_rmse = struct();
segment_rmse.position = zeros(num_segments, 1);
segment_rmse.speed = zeros(num_segments, 1);
segment_rmse.course = zeros(num_segments, 1);
segment_rmse.names = cell(num_segments, 1);

for i = 1:num_segments
    seg_idx = segmentInfo == segments(i);
    
    % Set segment name based on index
    if segments(i) == 1
        segment_rmse.names{i} = 'Const Vel 1';
    elseif segments(i) == 2
        segment_rmse.names{i} = 'Accel';
    elseif segments(i) == 3
        segment_rmse.names{i} = '45° Turn';
    elseif segments(i) == 4
        segment_rmse.names{i} = 'Const Vel 2';
    elseif segments(i) == 5
        segment_rmse.names{i} = '90° Turn';
    elseif segments(i) == 6
        segment_rmse.names{i} = 'Decel';
    elseif segments(i) == 7
        segment_rmse.names{i} = 'Const Vel 3';
    end
    
    % Calculate segment RMSE
    segment_rmse.position(i) = sqrt(mean(pos_error(seg_idx).^2));
    segment_rmse.speed(i) = sqrt(mean(sog_error(seg_idx).^2));
    segment_rmse.course(i) = sqrt(mean(cog_error(seg_idx).^2));
end
end

function aisData = createDataTable(t, x_true, y_true, vx_true, vy_true, sog_true, cog_true, ...
                                 x_meas, y_meas, sog_meas, cog_meas, segmentInfo, rmse_stats)
% Create a data table with both measurements and ground truth

% Create timestamp vector
startTime = datetime('now');
timestamps = startTime + seconds(t);

% Create table with AIS measurements
aisData = table(timestamps, x_meas, y_meas, sog_meas, cog_meas, ...
                'VariableNames', {'timestamp', 'x', 'y', 'SOG', 'COG'});

% Add ground truth to the table for evaluation
aisData.x_true = x_true;
aisData.y_true = y_true;
aisData.vx_true = vx_true;
aisData.vy_true = vy_true;
aisData.sog_true = sog_true;
aisData.cog_true = cog_true;

% Add measurement errors
aisData.pos_error = sqrt((x_meas - x_true).^2 + (y_meas - y_true).^2);
aisData.sog_error = abs(sog_meas - sog_true);
cog_true_rad = deg2rad(cog_true);
cog_meas_rad = deg2rad(cog_meas);
aisData.cog_error = abs(rad2deg(angdiff(cog_meas_rad, cog_true_rad)));

% Add overall RMSE statistics as metadata
aisData.Properties.UserData.rmse_stats = rmse_stats;

% Add segment information
aisData.segment = segmentInfo;

% Create segment names as categorical variable
segment_names = {'Const Vel 1', 'Accel', '45° Turn', 'Const Vel 2', ...
                '90° Turn', 'Decel', 'Const Vel 3'};
aisData.segment_name = categorical(segmentInfo, 1:7, segment_names);

% Create random MMSI (Maritime Mobile Service Identity)
mmsi = randi([100000000, 999999999]);
aisData.MMSI = repmat(mmsi, height(aisData), 1);
end

function createVisualizationPlots(aisData, x_true, y_true, x_meas, y_meas, ...
                                sog_true, sog_meas, cog_true, cog_meas, t, segStarts, params, ...
                                rmse_stats, segment_rmse)
% Create visualization plots of the simulated track

% Create figure with 3x2 layout for additional error plots
figure;

% Get MMSI for the title
mmsi = aisData.MMSI(1);

% 1. Trajectory plot with segments colored
subplot(3, 2, 1);
plotTrajectory(x_true, y_true, x_meas, y_meas, aisData, params);

% 2. Speed over time plot
subplot(3, 2, 2);
plotSpeedProfile(t, sog_true, sog_meas, segStarts, params);

% 3. Course over time plot
subplot(3, 2, 3);
plotCourseProfile(t, cog_true, cog_meas, segStarts, params);

% 4. Segment duration bar chart
subplot(3, 2, 4);
plotSegmentDurations(aisData, params);

% 5. Error plots (position, speed, and course error by segment)
subplot(3, 2, 5);
plotErrorBySegment(segment_rmse);

% 6. Position error over time
subplot(3, 2, 6);
plotErrorOverTime(aisData, t, segStarts, params);

% Add overall title with RMSE info
sgtitle(sprintf('Simulated AIS Track (MMSI: %d) - Pos RMSE: %.2fm, Speed RMSE: %.2fm/s, Course RMSE: %.2f°', ...
               mmsi, rmse_stats.position, rmse_stats.speed, rmse_stats.course));

% Create output directory if it doesn't exist
if ~exist('../figures', 'dir')
    mkdir('../figures');
end

% Save figures
saveas(gcf, fullfile('../figures', 'specific_ais_movement.png'));
saveas(gcf, fullfile('../figures', 'specific_ais_movement.fig'));
end

function plotErrorBySegment(segment_rmse)
% Plot RMSE by segment as bar chart

% Number of segments
num_segments = length(segment_rmse.position);

% Create bar chart
bar([segment_rmse.position, segment_rmse.speed, segment_rmse.course]);

% Add labels and formatting
xlabel('Segment');
ylabel('RMSE');
title('Error by Segment');
xticklabels(segment_rmse.names);
xtickangle(45);
legend('Position (m)', 'Speed (m/s)', 'Course (deg)', 'Location', 'best');
grid on;
end

function plotErrorOverTime(aisData, t, segStarts, params)
% Plot position error over time

% Convert time to minutes for better readability
t_min = t/60;

% Position error
plot(t_min, aisData.pos_error, 'b-', 'LineWidth', 1.5);
hold on;

% Add segment boundaries as vertical lines
for s = 2:length(segStarts)
    if segStarts(s) <= params.numSamples
        xline(t_min(segStarts(s)), 'k--');
    end
end

% Add segment labels at midpoints
uniqueSegments = unique(aisData.segment);
for s = 1:length(uniqueSegments)
    segIdx = find(aisData.segment == s);
    if ~isempty(segIdx)
        midIdx = segIdx(ceil(length(segIdx)/2));
        text(t_min(midIdx), max(aisData.pos_error)/2, char(aisData.segment_name(midIdx)), ...
            'FontWeight', 'bold', 'FontSize', 8, 'HorizontalAlignment', 'center');
    end
end

% Add labels and formatting
xlabel('Time (minutes)');
ylabel('Error (m)');
title('Position Error Over Time');
grid on;

% Add horizontal line for overall RMSE
yline(sqrt(mean(aisData.pos_error.^2)), 'r--', sprintf('Overall RMSE: %.2fm', sqrt(mean(aisData.pos_error.^2))));
end
function params = defineSimulationParameters()
% Define all parameters for the simulation

params = struct();

% Time parameters
params.sampleTime = 10;                 % 10 seconds between samples
params.numSamples = 250;                % Target number of samples
params.totalDuration = params.sampleTime * params.numSamples;  % Total duration in seconds

% Vessel characteristics
params.initialSpeed = 5;                % Initial speed in m/s (about 10 knots)
params.maxSpeed = 8;                    % Maximum speed in m/s (about 16 knots)
params.acceleration = 0.05;             % Acceleration in m/s^2
params.deceleration = -0.05;            % Deceleration in m/s^2
params.initialHeading = 30 * pi/180;    % Initial heading in radians (45 degrees)

% Turn parameters
params.turnRate45 = 1.5;                % Degrees per sample for 45° turn
params.turnRate90 = 2.0;                % Degrees per sample for 90° turn

% Segment durations (in number of samples)
params.constVel1Duration = 40;          % Initial constant velocity
params.accelDuration = 30;              % Constant acceleration
params.turn45Duration = 30;             % 45° turn (smooth)
params.constVel2Duration = 40;          % Second constant velocity
params.turn90Duration = 45;             % 90° turn (smooth)
params.decelDuration = 30;              % Constant deceleration
params.constVel3Duration = 35;          % Final constant velocity

% Measurement noise parameters
params.positionNoise = 10;               % Position noise std dev (meters)
params.speedNoise = 1;                % Speed noise std dev (m/s)
params.courseNoise = 5;                 % Course noise std dev (degrees)

% Segment names
params.segmentNames = {'Const Vel 1', 'Accel', '45° Turn', 'Const Vel 2', ...
                      '90° Turn', 'Decel', 'Const Vel 3'};
end

function [x_true, y_true, vx_true, vy_true, sog_true, cog_true, t, segmentInfo, segStarts] = ...
    generateGroundTruthTrajectory(params)
% Generate ground truth trajectory with specific motion patterns

% Create time vector
t = (0:params.numSamples-1)' * params.sampleTime;

% Initialize arrays for storing trajectory data
x_true = zeros(params.numSamples, 1);
y_true = zeros(params.numSamples, 1);
vx_true = zeros(params.numSamples, 1);
vy_true = zeros(params.numSamples, 1);
sog_true = zeros(params.numSamples, 1);
cog_true = zeros(params.numSamples, 1);

% Calculate segment start indices
segStarts = cumsum([1; 
              params.constVel1Duration; 
              params.accelDuration; 
              params.turn45Duration; 
              params.constVel2Duration; 
              params.turn90Duration; 
              params.decelDuration]);
          
% Ensure we don't exceed array bounds              
if segStarts(end) > params.numSamples
    error('Total segment duration exceeds requested sample count');
end

% Initialize position, heading and speed
pos = [0; 0];                      % Starting at origin
heading = params.initialHeading;   % Initial heading
speed = params.initialSpeed;       % Initial speed
vel = speed * [cos(heading); sin(heading)];  % Initial velocity vector

% Generate trajectory sample by sample
for i = 1:params.numSamples
    % Apply appropriate motion model based on current segment
    if i < segStarts(2)
        % Segment 1: Constant Velocity
        % No changes to speed or heading
        
    elseif i < segStarts(3)
        % Segment 2: Constant Acceleration
        speed = min(params.maxSpeed, speed + params.acceleration * params.sampleTime);
        
    elseif i < segStarts(4)
        % Segment 3: 45-degree Turn (smooth)
        % Calculate turn rate to complete 45° in turn45Duration samples
        turnIncrement = 45 / params.turn45Duration * pi/180;  % radians per sample
        heading = heading + turnIncrement;
        
    elseif i < segStarts(5)
        % Segment 4: Constant Velocity
        % No changes to speed or heading
        
    elseif i < segStarts(6)
        % Segment 5: 90-degree Turn (smooth)
        % Calculate turn rate to complete 90° in turn90Duration samples
        turnIncrement = 90 / params.turn90Duration * pi/180;  % radians per sample
        heading = heading + turnIncrement;
        
    elseif i < segStarts(7)
        % Segment 6: Constant Deceleration
        speed = max(params.initialSpeed/2, speed + params.deceleration * params.sampleTime);
        
    else
        % Segment 7: Final Constant Velocity
        % No changes to speed or heading
    end
    
    % Update velocity vector based on current speed and heading
    vel = speed * [cos(heading); sin(heading)];
    
    % Update position using current velocity
    pos = pos + vel * params.sampleTime;
    
    % Store ground truth values
    x_true(i) = pos(1);
    y_true(i) = pos(2);
    vx_true(i) = vel(1);
    vy_true(i) = vel(2);
    sog_true(i) = speed;
    cog_true(i) = mod(heading * 180/pi, 360);  % Convert to degrees in [0,360)
end

% Create segment information array
segmentInfo = createSegmentInfo(params.numSamples, segStarts);
end

function segmentInfo = createSegmentInfo(numSamples, segStarts)
% Create an array identifying which segment each sample belongs to

segmentInfo = ones(numSamples, 1);

% Assign segment numbers to each sample
for s = 1:length(segStarts)-1
    startIdx = segStarts(s);
    endIdx = min(segStarts(s+1)-1, numSamples);
    segmentInfo(startIdx:endIdx) = s;
end

% Handle the last segment if needed
if segStarts(end) <= numSamples
    segmentInfo(segStarts(end):end) = length(segStarts);
end
end

function [x_meas, y_meas, sog_meas, cog_meas] = ...
    generateNoisyMeasurements(x_true, y_true, sog_true, cog_true, params)
% Add realistic measurement noise to the ground truth

% Add Gaussian noise to position
x_meas = x_true + randn(size(x_true)) * params.positionNoise;
y_meas = y_true + randn(size(y_true)) * params.positionNoise;

% Add Gaussian noise to speed
sog_meas = sog_true + randn(size(sog_true)) * params.speedNoise;
sog_meas = max(0, sog_meas);  % Speed can't be negative

% Add Gaussian noise to course
cog_meas = cog_true + randn(size(cog_true)) * params.courseNoise;
cog_meas = mod(cog_meas, 360);  % Keep course in [0,360) range
end


function outputPath = saveDataToCSV(aisData)
% Save the data table to a CSV file


% Define output path
outputPath = fullfile('data', 'specific_ais_movement.csv');

% Write table to CSV
writetable(aisData, outputPath);
end



function plotTrajectory(x_true, y_true, x_meas, y_meas, aisData, params)
% Plot the trajectory with segments in different colors

% Get distinct colors for segments
segmentColors = lines(7);
hold on;

% Plot each segment with its own color
for s = 1:7
    segIdx = aisData.segment == s;
    if any(segIdx)
        plot(x_true(segIdx), y_true(segIdx), 'Color', segmentColors(s,:), 'LineWidth', 2);
    end
end

% Plot measurements as scattered points
scatter(x_meas, y_meas, 15, 'k', 'filled', 'MarkerFaceAlpha', 0.3);

% Add segment labels at midpoints
for s = 1:7
    segIdx = find(aisData.segment == s);
    if ~isempty(segIdx)
        midIdx = segIdx(ceil(length(segIdx)/2));
        text(x_true(midIdx), y_true(midIdx), char(aisData.segment_name(midIdx)), ...
            'FontWeight', 'bold', 'FontSize', 10, 'HorizontalAlignment', 'center');
    end
end

% Add labels and formatting
xlabel('X (meters)');
ylabel('Y (meters)');
title('Simulated AIS Trajectory');
grid on;
axis equal;
legend([params.segmentNames, {'AIS Reports'}], 'Location', 'best');
end

function plotSpeedProfile(t, sog_true, sog_meas, segStarts, params)
% Plot the speed profile over time

% Convert time to minutes for better readability
t_min = t/60;

% Plot ground truth and measurements
plot(t_min, sog_true, 'b-', 'LineWidth', 1.5);
hold on;
plot(t_min, sog_meas, 'r.', 'MarkerSize', 6);

% Add segment boundaries as vertical lines
for s = 2:length(segStarts)
    if segStarts(s) <= params.numSamples
        xline(t_min(segStarts(s)), 'k--');
    end
end

% Add labels and formatting
xlabel('Time (minutes)');
ylabel('Speed (m/s)');
title('Speed Over Ground');
legend('Ground Truth', 'AIS Reports', 'Location', 'best');
grid on;
end

function plotCourseProfile(t, cog_true, cog_meas, segStarts, params)
% Plot the course profile over time

% Convert time to minutes for better readability
t_min = t/60;

% Plot ground truth and measurements
plot(t_min, cog_true, 'b-', 'LineWidth', 1.5);
hold on;
plot(t_min, cog_meas, 'r.', 'MarkerSize', 6);

% Add segment boundaries as vertical lines
for s = 2:length(segStarts)
    if segStarts(s) <= params.numSamples
        xline(t_min(segStarts(s)), 'k--');
    end
end

% Add labels and formatting
xlabel('Time (minutes)');
ylabel('Course (degrees)');
title('Course Over Ground');
legend('Ground Truth', 'AIS Reports', 'Location', 'best');
grid on;
end

function plotSegmentDurations(aisData, params)
% Plot the duration of each segment as a bar chart

% Calculate the number of samples in each segment
segmentCounts = histcounts(aisData.segment, 1:8);

% Create bar chart
bar(1:7, segmentCounts, 'FaceColor', 'flat');
colormap(lines(7));

% Add labels and formatting
xlabel('Segment');
ylabel('Number of samples');
title('Segment Durations');
xticks(1:7);
xticklabels(params.segmentNames);
xtickangle(45);
grid on;
end