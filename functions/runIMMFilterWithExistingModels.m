function [estimates, stats] = runIMMFilterWithExistingModels(data, q_cv, q_ca, q_ctrv)
% RUNIMMFILTERWITHEXISTINGMODELS - Create IMM using existing filter implementations
% Uses: runKalmanFilter (CV), runExtendedKalmanFilterCA (CA), runUnscentedKalmanFilter (CTRV)
% 
% This creates filter objects that mimic your existing implementations

if nargin < 2, q_cv = 0.01; end
if nargin < 3, q_ca = 0.01; end  
if nargin < 4, q_ctrv = 0.01; end

% Extract measurements
n = height(data);
dt = zeros(n, 1);
for i = 2:n
    dt(i) = seconds(data.timestamp(i) - data.timestamp(i-1));
end

% Noise specifications (matching your existing filters)
pos_noise_std = 15;
vel_noise_std = 1;
accel_noise_std = 0.5;

% Simple initial conditions
initial_vx = 0;
initial_vy = 0;
initial_ax = 0;
initial_ay = 0;

% Create CV Filter (matching runKalmanFilter.m)
% State format: [x, vx, y, vy] (4-state)
cvFilter = trackingKF('MotionModel', '2D Constant Velocity', ...
                     'State', [data.x(1); initial_vx; data.y(1); initial_vy], ...
                     'StateCovariance', diag([pos_noise_std^2, vel_noise_std^2, pos_noise_std^2, vel_noise_std^2]), ...
                     'ProcessNoise', q_cv * diag([vel_noise_std^2, vel_noise_std^2]), ...
                     'MeasurementModel', [1 0 0 0; 0 0 1 0], ...
                     'MeasurementNoise', diag([pos_noise_std^2, pos_noise_std^2]));

% Create CA Filter (matching runExtendedKalmanFilterCA.m) 
% State format: [x, vx, ax, y, vy, ay] (6-state)
caFilter = trackingEKF(@caStateTransition6State, @caMeasurement6State, ...
                      [data.x(1); initial_vx; initial_ax; data.y(1); initial_vy; initial_ay], ...
                      'StateCovariance', diag([pos_noise_std^2, vel_noise_std^2, accel_noise_std^2, ...
                                             pos_noise_std^2, vel_noise_std^2, accel_noise_std^2]), ...
                      'ProcessNoise', q_ca * diag([0.1, vel_noise_std^2, accel_noise_std^2, ...
                                                  0.1, vel_noise_std^2, accel_noise_std^2]), ...
                      'MeasurementNoise', diag([pos_noise_std^2, pos_noise_std^2]));

% Create CTRV Filter (matching runUnscentedKalmanFilter.m)
% State format: [x, vx, y, vy] (4-state) - your UKF uses CV model
ctrvFilter = trackingKF('MotionModel', '2D Constant Velocity', ...
                       'State', [data.x(1); initial_vx; data.y(1); initial_vy], ...
                       'StateCovariance', diag([pos_noise_std^2, vel_noise_std^2, pos_noise_std^2, vel_noise_std^2]), ...
                       'ProcessNoise', q_ctrv * diag([vel_noise_std^2, vel_noise_std^2]), ...
                       'MeasurementModel', [1 0 0 0; 0 0 1 0], ...
                       'MeasurementNoise', diag([pos_noise_std^2, pos_noise_std^2]));

% 3-model transition probability matrix
transitionMatrix = [0.7 0.2 0.1;   % CV -> CV, CA, CTRV
                    0.2 0.6 0.2;   % CA -> CV, CA, CTRV  
                    0.1 0.2 0.7];  % CTRV -> CV, CA, CTRV

% Create IMM filter with model conversion
imm = trackingIMM({cvFilter, caFilter, ctrvFilter}, ...
                  'ModelNames', {'cv', 'ca', 'ctrv'}, ...
                  'ModelConversionFcn', @existingModelsConversionFcn, ...
                  'TransitionProbabilities', transitionMatrix);

% Initialize with CV state format [x, vx, y, vy]
initialize(imm, [data.x(1); initial_vx; data.y(1); initial_vy], ...
           diag([pos_noise_std^2, vel_noise_std^2, pos_noise_std^2, vel_noise_std^2]));

% Initialize arrays  
x_est = zeros(4, n);
P_est = zeros(4, 4, n);
model_probs = zeros(3, n);

% Initial state
x_est(:,1) = imm.State;
P_est(:,:,1) = imm.StateCovariance;
model_probs(:,1) = imm.ModelProbabilities;

% Main filtering loop
for k = 2:n
    current_dt = dt(k);
    if isnan(current_dt) || current_dt <= 0
        current_dt = 1.0;
    end
    
    % Predict
    predict(imm, current_dt);
    
    % Measurement
    z = [data.x(k); data.y(k)];
    
    % Correct
    correct(imm, z);
    
    % Store estimates
    x_est(:,k) = imm.State;
    P_est(:,:,k) = imm.StateCovariance;
    model_probs(:,k) = imm.ModelProbabilities;
end

% Create output table (matching your existing filter outputs)
estimates = table();
estimates.timestamp = data.timestamp;
estimates.x_est = x_est(1,:)';  % x position
estimates.y_est = x_est(3,:)';  % y position  
estimates.vx_est = x_est(2,:)'; % x velocity
estimates.vy_est = x_est(4,:)'; % y velocity
estimates.sog_est = sqrt(x_est(2,:).^2 + x_est(4,:).^2)';
estimates.cog_est = mod(atan2(x_est(4,:), x_est(2,:)) * 180/pi, 360)';

% Add model probabilities
estimates.cv_prob = model_probs(1,:)';
estimates.ca_prob = model_probs(2,:)';
estimates.ctrv_prob = model_probs(3,:)';

% Calculate error statistics
stats = calculateErrorStatistics(data, estimates, ...
    sprintf('IMM-Existing (q_cv=%.3f, q_ca=%.3f, q_ctrv=%.3f)', q_cv, q_ca, q_ctrv));

% Plot results
plotFilterResults(data, estimates, 'IMM with Existing Models');
plot3ModelProbabilities(data, estimates);

% Print model switching summary
fprintf('\n--- Model Selection Summary ---\n');
fprintf('CV Model:   %.1f%% dominant\n', mean(estimates.cv_prob) * 100);
fprintf('CA Model:   %.1f%% dominant\n', mean(estimates.ca_prob) * 100); 
fprintf('CTRV Model: %.1f%% dominant\n', mean(estimates.ctrv_prob) * 100);

end

%% Model Conversion Function for Existing Filter States
function x2 = existingModelsConversionFcn(modelName1, x1, modelName2, x2)
% Convert between CV (4-state), CA (6-state), and CTRV (4-state) models
% State formats:
% CV:   [x, vx, y, vy]
% CA:   [x, vx, ax, y, vy, ay] 
% CTRV: [x, vx, y, vy] (same as CV in your implementation)

if strcmp(modelName1, modelName2)
    x2 = x1;
    return;
end

% CV to CA conversion: [x, vx, y, vy] -> [x, vx, ax, y, vy, ay]
if strcmp(modelName1, 'cv') && strcmp(modelName2, 'ca')
    if isvector(x1) && isvector(x2)
        x2(1) = x1(1);  % x
        x2(2) = x1(2);  % vx
        x2(3) = 0;      % ax = 0
        x2(4) = x1(3);  % y
        x2(5) = x1(4);  % vy
        x2(6) = 0;      % ay = 0
    else
        x2 = zeros(6, 6);
        x2(1,1) = x1(1,1);  % x variance
        x2(2,2) = x1(2,2);  % vx variance
        x2(3,3) = 0.1;      % ax variance (small)
        x2(4,4) = x1(3,3);  % y variance
        x2(5,5) = x1(4,4);  % vy variance
        x2(6,6) = 0.1;      % ay variance (small)
    end

% CA to CV conversion: [x, vx, ax, y, vy, ay] -> [x, vx, y, vy]
elseif strcmp(modelName1, 'ca') && strcmp(modelName2, 'cv')
    if isvector(x1) && isvector(x2)
        x2(1) = x1(1);  % x
        x2(2) = x1(2);  % vx
        x2(3) = x1(4);  % y
        x2(4) = x1(5);  % vy
    else
        x2 = zeros(4, 4);
        x2(1,1) = x1(1,1);  % x variance
        x2(2,2) = x1(2,2);  % vx variance
        x2(3,3) = x1(4,4);  % y variance
        x2(4,4) = x1(5,5);  % vy variance
    end

% CV to CTRV conversion: [x, vx, y, vy] -> [x, vx, y, vy] (same format)
elseif strcmp(modelName1, 'cv') && strcmp(modelName2, 'ctrv')
    x2 = x1;  % Same state format

% CTRV to CV conversion: [x, vx, y, vy] -> [x, vx, y, vy] (same format)
elseif strcmp(modelName1, 'ctrv') && strcmp(modelName2, 'cv')
    x2 = x1;  % Same state format

% CA to CTRV conversion: [x, vx, ax, y, vy, ay] -> [x, vx, y, vy]
elseif strcmp(modelName1, 'ca') && strcmp(modelName2, 'ctrv')
    if isvector(x1) && isvector(x2)
        x2(1) = x1(1);  % x
        x2(2) = x1(2);  % vx
        x2(3) = x1(4);  % y
        x2(4) = x1(5);  % vy
    else
        x2 = zeros(4, 4);
        x2(1,1) = x1(1,1);  % x variance
        x2(2,2) = x1(2,2);  % vx variance
        x2(3,3) = x1(4,4);  % y variance
        x2(4,4) = x1(5,5);  % vy variance
    end

% CTRV to CA conversion: [x, vx, y, vy] -> [x, vx, ax, y, vy, ay]
elseif strcmp(modelName1, 'ctrv') && strcmp(modelName2, 'ca')
    if isvector(x1) && isvector(x2)
        x2(1) = x1(1);  % x
        x2(2) = x1(2);  % vx
        x2(3) = 0;      % ax = 0
        x2(4) = x1(3);  % y
        x2(5) = x1(4);  % vy
        x2(6) = 0;      % ay = 0
    else
        x2 = zeros(6, 6);
        x2(1,1) = x1(1,1);  % x variance
        x2(2,2) = x1(2,2);  % vx variance
        x2(3,3) = 0.1;      % ax variance (small)
        x2(4,4) = x1(3,3);  % y variance
        x2(5,5) = x1(4,4);  % vy variance
        x2(6,6) = 0.1;      % ay variance (small)
    end

else
    error('Unknown model conversion: %s to %s', modelName1, modelName2);
end
end

%% State Transition Functions
function x = caStateTransition6State(x, dt)
    % CA state transition for 6-state [x, vx, ax, y, vy, ay]
    F = [1 dt dt^2/2  0  0    0;
         0  1    dt   0  0    0;
         0  0     1   0  0    0;
         0  0     0   1 dt dt^2/2;
         0  0     0   0  1   dt;
         0  0     0   0  0    1];
    x = F * x;
end

function z = caMeasurement6State(x)
    % Extract position from 6-state [x, vx, ax, y, vy, ay]
    z = [x(1); x(4)];
end

%% 3-Model Probability Plotting
function plot3ModelProbabilities(data, estimates)
    figure;
    
    % Model probabilities
    subplot(1,2,1);
    t = (0:height(data)-1)';
    builtin('plot', t, estimates.cv_prob, 'b-', 'LineWidth', 2, 'DisplayName', 'CV (KF)');
    hold on;
    builtin('plot', t, estimates.ca_prob, 'g-', 'LineWidth', 2, 'DisplayName', 'CA (EKF)');
    builtin('plot', t, estimates.ctrv_prob, 'r-', 'LineWidth', 2, 'DisplayName', 'CTRV (UKF)');
    
    % Add segment boundaries if available
    if all(ismember({'segment'}, data.Properties.VariableNames))
        segmentChanges = find(diff(data.segment) ~= 0);
        for i = 1:length(segmentChanges)
            xline(segmentChanges(i)+1, 'k--', 'Alpha', 0.5);
        end
    end
    
    xlabel('Sample Number');
    ylabel('Model Probability');
    title('IMM with Existing Models - Probabilities');
    ylim([0, 1]);
    grid on;
    legend('Location', 'best');
    
    % Model dominance over time
    subplot(1,2,2);
    [~, dominant_model] = max([estimates.cv_prob, estimates.ca_prob, estimates.ctrv_prob], [], 2);
    builtin('plot', t, dominant_model, 'ko-', 'LineWidth', 1.5, 'MarkerSize', 3);
    xlabel('Sample Number');
    ylabel('Dominant Model');
    title('Dominant Model Selection');
    yticks([1, 2, 3]);
    yticklabels({'CV (KF)', 'CA (EKF)', 'CTRV (UKF)'});
    ylim([0.5, 3.5]);
    grid on;
    
    % Save figure
    if ~exist('figures', 'dir')
        mkdir('figures');
    end
    saveas(gcf, 'figures/imm_existing_models_analysis.png');
    saveas(gcf, 'figures/imm_existing_models_analysis.fig');
end