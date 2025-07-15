function [estimates, stats] = runIMMFilter(data)
% RUNIMMFILTER - Apply IMM filter to AIS data following research paper specifications
% Uses trackingIMM combining:
% - CV model: Linear Kalman Filter (trackingKF) 
% - CTRV model: Unscented Kalman Filter (trackingUKF)
% Paper specifications:
% - Process noise: velocity σ_v = 0.07 m/s, heading σ_ψ = 0.5°, yaw rate σ_ω = 0.1°/s
% - Measurement noise: position σ = 5 meters  
% - Transition matrix: [0.8 0.2; 0.2 0.8]

% Extract measurements
n = height(data);
dt = zeros(n, 1);
for i = 2:n
    dt(i) = seconds(data.timestamp(i) - data.timestamp(i-1));
end

% Research paper noise specifications
pos_noise_std = 5;      % Position measurement noise: σ = 5 meters
vel_noise_std = 0.07;   % Velocity process noise: σ_v = 0.07 m/s
heading_noise_std = deg2rad(0.5);  % Heading process noise: σ_ψ = 0.5°
yaw_rate_noise_std = deg2rad(0.1); % Yaw rate process noise: σ_ω = 0.1°/s

% Get initial state estimates
initial_vx = 0;
initial_vy = 0;
initial_speed = 0;
initial_heading = 0;
initial_yaw_rate = 0;

if ~isnan(data.SOG(1)) && ~isnan(data.COG(1))
    initial_vx = data.SOG(1) * cosd(data.COG(1));
    initial_vy = data.SOG(1) * sind(data.COG(1));
    initial_speed = data.SOG(1);
    initial_heading = deg2rad(data.COG(1));
    
    % Estimate initial yaw rate from first few measurements
    if n > 2 && ~isnan(data.COG(2))
        dt_init = seconds(data.timestamp(2) - data.timestamp(1));
        if dt_init > 0
            initial_yaw_rate = angdiff(deg2rad(data.COG(2)), deg2rad(data.COG(1))) / dt_init;
        end
    end
end

% Create Filter 1: Linear Kalman Filter for CV model using built-in motion model
% This avoids the fixed transition matrix issue
cvFilter = trackingKF('MotionModel', '2D Constant Velocity', ...
                     'State', [data.x(1); data.y(1); initial_vx; initial_vy], ...
                     'StateCovariance', diag([pos_noise_std^2, pos_noise_std^2, vel_noise_std^2, vel_noise_std^2]), ...
                     'ProcessNoise', diag([vel_noise_std^2, vel_noise_std^2]), ... % 2x2 for velocity components
                     'MeasurementModel', [1 0 0 0; 0 1 0 0], ...
                     'MeasurementNoise', diag([pos_noise_std^2, pos_noise_std^2]));

% Create Filter 2: Unscented Kalman Filter for CTRV model (5 states: x, y, v, ψ, ψ̇)
ctrvFilter = trackingUKF(@ctrvStateTransitionFcn, @ctrvMeasurementFcn, ...
                        [data.x(1); data.y(1); initial_speed; initial_heading; initial_yaw_rate], ...
                        'StateCovariance', diag([pos_noise_std^2, pos_noise_std^2, vel_noise_std^2, ...
                                               heading_noise_std^2, yaw_rate_noise_std^2]), ...
                        'ProcessNoise', diag([0.1, 0.1, vel_noise_std^2, heading_noise_std^2, yaw_rate_noise_std^2]), ...
                        'MeasurementNoise', diag([pos_noise_std^2, pos_noise_std^2]));

% Configure UKF parameters
ctrvFilter.Alpha = 1e-3;
ctrvFilter.Beta = 2;
ctrvFilter.Kappa = 0;

% Research paper transition probability matrix
transitionMatrix = [0.8 0.2; 0.2 0.8];

% Create IMM filter with research paper specifications
imm = trackingIMM({cvFilter, ctrvFilter}, ...
                  'ModelNames', {'cv', 'ctrv'}, ...
                  'ModelConversionFcn', @CVCTRVModelConversionFcn, ...
                  'TransitionProbabilities', transitionMatrix);

% Initialize IMM filter with CTRV state format (5 states)
initialize(imm, [data.x(1); data.y(1); initial_speed; initial_heading; initial_yaw_rate], ...
           diag([pos_noise_std^2, pos_noise_std^2, vel_noise_std^2, heading_noise_std^2, yaw_rate_noise_std^2]));

% Get the actual state size from the initialized IMM filter
state_size = length(imm.State);

% Initialize state and covariance arrays with correct size
x_est = zeros(state_size, n);
P_est = zeros(state_size, state_size, n);
model_probs = zeros(2, n);  % Store model probabilities [CV; CTRV]

% Store initial estimates
x_est(:,1) = imm.State;
P_est(:,:,1) = imm.StateCovariance;
model_probs(:,1) = imm.ModelProbabilities;

fprintf('IMM: Initial model probabilities - CV: %.3f, CTRV: %.3f\n', ...
        model_probs(1,1), model_probs(2,1));
fprintf('IMM: State size is %d\n', state_size);

% Main filtering loop
for k = 2:n
    % Get time step for this iteration
    current_dt = dt(k);
    if isnan(current_dt) || current_dt <= 0
        current_dt = 1.0; % Default 1 second
    end
    
    % Predict with time step
    predict(imm, current_dt);
    
    % Create measurement vector - position only [x; y]
    z = [data.x(k); data.y(k)];
    
    % Correct
    correct(imm, z);
    
    % Store estimates
    x_est(:,k) = imm.State;
    P_est(:,:,k) = imm.StateCovariance;
    model_probs(:,k) = imm.ModelProbabilities;
    
    % Print model probabilities every 50 steps
    if mod(k, 50) == 0
        fprintf('Step %d - CV: %.3f, CTRV: %.3f\n', ...
                k, model_probs(1,k), model_probs(2,k));
    end
end

% Create output estimates based on the actual state size
estimates = table();
estimates.timestamp = data.timestamp;

% Handle different state sizes
if state_size == 5
    % CTRV format: [x, y, v, ψ, ψ̇]
    estimates.x_est = x_est(1,:)';
    estimates.y_est = x_est(2,:)';
    estimates.vx_est = x_est(3,:)' .* cos(x_est(4,:)');
    estimates.vy_est = x_est(3,:)' .* sin(x_est(4,:)');
    estimates.sog_est = x_est(3,:)';
    estimates.cog_est = mod(x_est(4,:)' * 180/pi, 360);
    estimates.yaw_rate_est = x_est(5,:)';
elseif state_size == 4
    % CV format: [x, y, vx, vy] - note the 2D Constant Velocity model uses this order
    estimates.x_est = x_est(1,:)';
    estimates.y_est = x_est(2,:)';
    estimates.vx_est = x_est(3,:)';
    estimates.vy_est = x_est(4,:)';
    estimates.sog_est = sqrt(x_est(3,:)'.^2 + x_est(4,:)'.^2);
    estimates.cog_est = mod(atan2(x_est(4,:)', x_est(3,:)') * 180/pi, 360);
    estimates.yaw_rate_est = zeros(size(x_est(1,:)'));
else
    error('Unexpected state size: %d', state_size);
end

% Model probabilities
estimates.cv_prob = model_probs(1,:)';
estimates.ctrv_prob = model_probs(2,:)';

% Calculate comprehensive error statistics
stats = calculateEnhancedErrorStatistics(data, estimates, 'IMM (CV+CTRV)');

% Plot results
plotFilterResults(data, estimates, 'IMM Filter (CV + CTRV)');
plotModelProbabilities(data, estimates);

end

%% Model Conversion Function for CV <-> CTRV
function x2 = CVCTRVModelConversionFcn(modelName1, x1, modelName2, x2)
% Convert between CV (4-state) and CTRV (5-state) models
if strcmp(modelName1, modelName2)
    x2 = x1;
    return;
end

if strcmp(modelName1, 'cv') && strcmp(modelName2, 'ctrv')
    % CV to CTRV conversion: [x, y, vx, vy] -> [x, y, v, ψ, ψ̇]
    if isvector(x1) && isvector(x2)
        x2(1) = x1(1);  % x
        x2(2) = x1(2);  % y
        x2(3) = sqrt(x1(3)^2 + x1(4)^2);  % v = sqrt(vx^2 + vy^2)
        x2(4) = atan2(x1(4), x1(3));      % ψ = atan2(vy, vx)
        x2(5) = 0;      % ψ̇ = 0 (assume no turn rate)
    else
        % Covariance conversion (approximate)
        x2 = zeros(5, 5);
        x2(1,1) = x1(1,1);  % x variance
        x2(2,2) = x1(2,2);  % y variance
        x2(3,3) = x1(3,3) + x1(4,4);  % v variance (approximate)
        x2(4,4) = deg2rad(5)^2;  % ψ variance (default)
        x2(5,5) = deg2rad(0.1)^2;  % ψ̇ variance (default)
    end
    
elseif strcmp(modelName1, 'ctrv') && strcmp(modelName2, 'cv')
    % CTRV to CV conversion: [x, y, v, ψ, ψ̇] -> [x, y, vx, vy]
    if isvector(x1) && isvector(x2)
        x2(1) = x1(1);  % x
        x2(2) = x1(2);  % y
        x2(3) = x1(3) * cos(x1(4));  % vx = v * cos(ψ)
        x2(4) = x1(3) * sin(x1(4));  % vy = v * sin(ψ)
    else
        % Covariance conversion (approximate)
        x2 = zeros(4, 4);
        x2(1,1) = x1(1,1);  % x variance
        x2(2,2) = x1(2,2);  % y variance
        x2(3,3) = x1(3,3);  % vx variance (approximate)
        x2(4,4) = x1(3,3);  % vy variance (approximate)
    end
else
    error('Unknown model conversion: %s to %s', modelName1, modelName2);
end
end

%% CTRV State Transition Function
function x = ctrvStateTransitionFcn(x, dt)
% CTRV state transition: [x, y, v, ψ, ψ̇] -> [x_{k+1}, y_{k+1}, v_{k+1}, ψ_{k+1}, ψ̇_{k+1}]
if nargin < 2
    dt = 0.1;
end

px = x(1); py = x(2); v = x(3); psi = x(4); psi_dot = x(5);

x_next = zeros(5, 1);

if abs(psi_dot) > 1e-6
    % Coordinated turn model
    x_next(1) = px + (v/psi_dot) * (sin(psi + psi_dot*dt) - sin(psi));
    x_next(2) = py + (v/psi_dot) * (-cos(psi + psi_dot*dt) + cos(psi));
    x_next(3) = v;  % Constant velocity
    x_next(4) = psi + psi_dot*dt;
    x_next(5) = psi_dot;  % Constant turn rate
else
    % Straight line motion (near-zero turn rate)
    x_next(1) = px + v * cos(psi) * dt;
    x_next(2) = py + v * sin(psi) * dt;
    x_next(3) = v;
    x_next(4) = psi;
    x_next(5) = psi_dot;
end

x = x_next;
end

%% CTRV Measurement Function
function z = ctrvMeasurementFcn(x)
% CTRV measurement function: [x, y] position only
z = [x(1); x(2)];
end


%% Enhanced Error Statistics Function
function stats = calculateEnhancedErrorStatistics(data, estimates, filterName)
% Calculate comprehensive error statistics following research paper metrics

% Initialize stats structure
stats = struct();
stats.filterName = filterName;

% Check if ground truth is available
if ~all(ismember({'x_true', 'y_true'}, data.Properties.VariableNames))
    warning('Ground truth not available for comprehensive statistics.');
    return;
end

% Calculate position error
posError = sqrt((estimates.x_est - data.x_true).^2 + ...
                (estimates.y_est - data.y_true).^2);

% Calculate velocity error if available
if all(ismember({'vx_true', 'vy_true'}, data.Properties.VariableNames))
    velError = sqrt((estimates.vx_est - data.vx_true).^2 + ...
                    (estimates.vy_est - data.vy_true).^2);
else
    velError = [];
end

% Calculate SOG and COG errors if available
if all(ismember({'sog_true', 'cog_true'}, data.Properties.VariableNames))
    sogError = abs(estimates.sog_est - data.sog_true);
    cogError = abs(angdiff(deg2rad(estimates.cog_est), deg2rad(data.cog_true))) * 180/pi;
else
    sogError = [];
    cogError = [];
end

% Position Statistics (Research Paper Metric 1)
stats.position.RMSE = sqrt(mean(posError.^2));
stats.position.MAE = mean(posError);
stats.position.maxDeviation = max(posError);
stats.position.percentile95 = prctile(posError, 95);
stats.position.percentile99 = prctile(posError, 99);

% Velocity Statistics
if ~isempty(velError)
    stats.velocity.RMSE = sqrt(mean(velError.^2));
    stats.velocity.MAE = mean(velError);
    stats.velocity.maxDeviation = max(velError);
    stats.velocity.percentile95 = prctile(velError, 95);
end

% SOG Statistics (Research Paper Metric 3)
if ~isempty(sogError)
    stats.SOG.RMSE = sqrt(mean(sogError.^2));
    stats.SOG.MAE = mean(sogError);
    stats.SOG.maxDeviation = max(sogError);
    stats.SOG.percentile95 = prctile(sogError, 95);
end

% COG Statistics (Research Paper Metric 2)
if ~isempty(cogError)
    stats.COG.RMSE = sqrt(mean(cogError.^2));
    stats.COG.MAE = mean(cogError);
    stats.COG.maxDeviation = max(cogError);
    stats.COG.percentile95 = prctile(cogError, 95);
end

% Model Selection Statistics
stats.modelSelection.avgCVProbability = mean(estimates.cv_prob);
stats.modelSelection.avgCTRVProbability = mean(estimates.ctrv_prob);
stats.modelSelection.numSwitches = sum(abs(diff(estimates.cv_prob > 0.5)));

% Fixed syntax for cell array indexing
model_names = {'CV', 'CTRV'};
if stats.modelSelection.avgCVProbability >= 0.5
    stats.modelSelection.dominantModel = model_names{1};
else
    stats.modelSelection.dominantModel = model_names{2};
end

% Segment-wise Statistics
if all(ismember({'segment', 'segment_name'}, data.Properties.VariableNames))
    segments = unique(data.segment);
    stats.segmentStats = struct();
    
    for i = 1:length(segments)
        seg = segments(i);
        segIdx = data.segment == seg;
        segName = char(data.segment_name(find(segIdx, 1)));
        
        % Position statistics per segment
        segPosError = posError(segIdx);
        segField = ['segment', num2str(seg)];
        stats.segmentStats.(segField).name = segName;
        stats.segmentStats.(segField).position.RMSE = sqrt(mean(segPosError.^2));
        stats.segmentStats.(segField).position.maxDeviation = max(segPosError);
        
        % Velocity statistics per segment
        if ~isempty(velError)
            segVelError = velError(segIdx);
            stats.segmentStats.(segField).velocity.RMSE = sqrt(mean(segVelError.^2));
        end
        
        % COG statistics per segment  
        if ~isempty(cogError)
            segCogError = cogError(segIdx);
            stats.segmentStats.(segField).COG.RMSE = sqrt(mean(segCogError.^2));
        end
        
        % Model selection per segment
        segCVProb = mean(estimates.cv_prob(segIdx));
        stats.segmentStats.(segField).modelSelection.avgCVProbability = segCVProb;
        
        % Fixed syntax for cell array indexing
        if segCVProb >= 0.5
            stats.segmentStats.(segField).modelSelection.dominantModel = model_names{1};
        else
            stats.segmentStats.(segField).modelSelection.dominantModel = model_names{2};
        end
    end
end

% Display comprehensive statistics
fprintf('\n===== %s Enhanced Performance Statistics =====\n', filterName);
fprintf('Position RMSE: %.2f m (Paper target: ~5m)\n', stats.position.RMSE);
fprintf('Position Max Deviation: %.2f m\n', stats.position.maxDeviation);
fprintf('Position 95th Percentile: %.2f m\n', stats.position.percentile95);

if ~isempty(cogError)
    fprintf('COG RMSE: %.2f degrees\n', stats.COG.RMSE);
    fprintf('COG Max Deviation: %.2f degrees\n', stats.COG.maxDeviation);
end

if ~isempty(sogError)
    fprintf('SOG RMSE: %.2f m/s\n', stats.SOG.RMSE);
    fprintf('SOG Max Deviation: %.2f m/s\n', stats.SOG.maxDeviation);
end

fprintf('Model Selection: %s (%.1f%% dominant)\n', stats.modelSelection.dominantModel, ...
        max(stats.modelSelection.avgCVProbability, stats.modelSelection.avgCTRVProbability) * 100);
fprintf('Number of Model Switches: %d\n', stats.modelSelection.numSwitches);

% Display segment statistics
if isfield(stats, 'segmentStats')
    fprintf('\n--- Performance by Motion Segment ---\n');
    fprintf('%-15s | %-12s | %-12s | %-12s | %-12s\n', ...
            'Segment', 'Pos RMSE', 'COG RMSE', 'Dominant', 'CV Prob');
    fprintf('------------------------------------------------------------------------\n');
    
    segmentFields = fieldnames(stats.segmentStats);
    for i = 1:length(segmentFields)
        segData = stats.segmentStats.(segmentFields{i});
        
        % Fixed: Replace ternary operator with if-else statement
        if isfield(segData, 'COG')
            cogRMSE = segData.COG.RMSE;
        else
            cogRMSE = 0;
        end
        
        fprintf('%-15s | %-12.2f | %-12.2f | %-12s | %-12.1f%%\n', ...
                segData.name, segData.position.RMSE, cogRMSE, ...
                segData.modelSelection.dominantModel, ...
                segData.modelSelection.avgCVProbability * 100);
    end
end

end

%% Enhanced Model Probabilities Plotting
function plotModelProbabilities(data, estimates)
figure;

% Main probability plot
subplot(2,1,1);
t = (0:height(data)-1)';
plot(t, estimates.cv_prob, 'b-', 'LineWidth', 2, 'DisplayName', 'CV Model');
hold on;
plot(t, estimates.ctrv_prob, 'r-', 'LineWidth', 2, 'DisplayName', 'CTRV Model');

% Add segment boundaries if available
if isfield(data, 'segment')
    segmentChanges = find(diff(data.segment) ~= 0);
    for i = 1:length(segmentChanges)
        xline(segmentChanges(i)+1, 'k--', 'Alpha', 0.7);
    end
end

xlabel('Sample Number');
ylabel('Model Probability');
title('IMM Filter - Model Probabilities (Research Paper Configuration)');
ylim([0, 1]);
grid on;
legend('Location', 'best');

% Model switching analysis
subplot(2,1,2);
model_switches = abs(diff(estimates.cv_prob > 0.5));
cumulative_switches = cumsum(model_switches);
plot(t(2:end), cumulative_switches, 'g-', 'LineWidth', 2);
xlabel('Sample Number');
ylabel('Cumulative Model Switches');
title('Model Switching Behavior');
grid on;

% Save figures
if ~exist('figures', 'dir')
    mkdir('figures');
end
saveas(gcf, 'figures/imm_enhanced_analysis.png');
saveas(gcf, 'figures/imm_enhanced_analysis.fig');

end