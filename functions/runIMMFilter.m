function [estimates, stats] = runIMMFilter(data, q_cv, q_ca, q_ctrv)
% RUNIMMFILTER - Apply IMM filter to AIS data
% Fully causal implementation without future information
% 3-model IMM: CV (KF) + CA (EKF) + CTRV (UKF)

if nargin < 2, q_cv = 0.01; end
if nargin < 3, q_ca = 0.01; end
if nargin < 4, q_ctrv = 0.01; end

% Extract measurements
n = height(data);
dt = zeros(n, 1);
for i = 2:n
    dt(i) = seconds(data.timestamp(i) - data.timestamp(i-1));
end

% Noise specifications
pos_noise_std = 30;
vel_noise_std = 1;
accel_noise_std = 0.5;
heading_noise_std = deg2rad(2);
yaw_rate_noise_std = deg2rad(5);

% Simple initial conditions
initial_vx = 0;
initial_vy = 0;
initial_ax = 0;
initial_ay = 0;
initial_speed = 0;
initial_heading = 0;
initial_yaw_rate = 0;

% Create CV Filter (keep existing)
cvFilter = trackingKF('MotionModel', '2D Constant Velocity', ...
                     'State', [data.x(1); data.y(1); initial_vx; initial_vy], ...
                     'StateCovariance', diag([pos_noise_std^2, pos_noise_std^2, vel_noise_std^2, vel_noise_std^2]), ...
                     'ProcessNoise', q_cv * diag([vel_noise_std^2, vel_noise_std^2]), ...
                     'MeasurementModel', [1 0 0 0; 0 1 0 0], ...
                     'MeasurementNoise', diag([pos_noise_std^2, pos_noise_std^2]));

% Create CA Filter (NEW - 6-state constant acceleration)
caFilter = trackingEKF(@caStateTransitionFcn, @caMeasurementFcn, ...
                      [data.x(1); data.y(1); initial_vx; initial_vy; initial_ax; initial_ay], ...
                      'StateCovariance', diag([pos_noise_std^2, pos_noise_std^2, vel_noise_std^2, ...
                                             vel_noise_std^2, accel_noise_std^2, accel_noise_std^2]), ...
                      'ProcessNoise', q_ca * diag([0.1, 0.1, vel_noise_std^2, vel_noise_std^2, ...
                                                  accel_noise_std^2, accel_noise_std^2]), ...
                      'MeasurementNoise', diag([pos_noise_std^2, pos_noise_std^2]));

% Create CTRV Filter (keep existing)
ctrvFilter = trackingUKF(@ctrvStateTransitionFcn, @ctrvMeasurementFcn, ...
                        [data.x(1); data.y(1); initial_speed; initial_heading; initial_yaw_rate], ...
                        'StateCovariance', diag([pos_noise_std^2, pos_noise_std^2, vel_noise_std^2, ...
                                               heading_noise_std^2, yaw_rate_noise_std^2]), ...
                        'ProcessNoise', q_ctrv * diag([0.1, 0.1, vel_noise_std^2, heading_noise_std^2, yaw_rate_noise_std^2]), ...
                        'MeasurementNoise', diag([pos_noise_std^2, pos_noise_std^2]));

ctrvFilter.Alpha = 0.6;
ctrvFilter.Beta = 2;
ctrvFilter.Kappa = 0;

% 3-model transition probability matrix
transitionMatrix = [0.7 0.2 0.1;   % CV -> CV, CA, CTRV
                    0.2 0.6 0.2;   % CA -> CV, CA, CTRV
                    0.1 0.2 0.7];  % CTRV -> CV, CA, CTRV

% Create 3-model IMM filter
imm = trackingIMM({cvFilter, caFilter, ctrvFilter}, ...
                  'ModelNames', {'cv', 'ca', 'ctrv'}, ...
                  'ModelConversionFcn', @threeModelConversionFcn, ...
                  'TransitionProbabilities', transitionMatrix);

% Initialize IMM with CV state format
initialize(imm, [data.x(1); data.y(1); initial_vx; initial_vy], ...
           diag([pos_noise_std^2, pos_noise_std^2, vel_noise_std^2, vel_noise_std^2]));

% Initialize arrays
x_est = zeros(4, n);
P_est = zeros(4, 4, n);
model_probs = zeros(3, n);  % Now 3 models

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

% Create output table
estimates = table();
estimates.timestamp = data.timestamp;
estimates.x_est = x_est(1,:)';
estimates.y_est = x_est(2,:)';
estimates.vx_est = x_est(3,:)';
estimates.vy_est = x_est(4,:)';
estimates.sog_est = sqrt(x_est(3,:).^2 + x_est(4,:).^2)';
estimates.cog_est = mod(atan2(x_est(4,:), x_est(3,:)) * 180/pi, 360)';

% Add model probabilities for all 3 models
estimates.cv_prob = model_probs(1,:)';
estimates.ca_prob = model_probs(2,:)';
estimates.ctrv_prob = model_probs(3,:)';

% Calculate error statistics
stats = calculateErrorStatistics(data, estimates, sprintf('3-Model IMM (q_cv=%.3f, q_ca=%.3f, q_ctrv=%.3f)', q_cv, q_ca, q_ctrv));

% Plot results
plotFilterResults(data, estimates, '3-Model IMM Filter');
plot3ModelProbabilities(data, estimates);

end

%% 3-Model Conversion Function
function x2 = threeModelConversionFcn(modelName1, x1, modelName2, x2)
% Convert between CV (4-state), CA (6-state), and CTRV (5-state) models

if strcmp(modelName1, modelName2)
    x2 = x1;
    return;
end

% CV to CA conversion
if strcmp(modelName1, 'cv') && strcmp(modelName2, 'ca')
    if isvector(x1) && isvector(x2)
        x2(1) = x1(1);  % x
        x2(2) = x1(2);  % y
        x2(3) = x1(3);  % vx
        x2(4) = x1(4);  % vy
        x2(5) = 0;      % ax = 0
        x2(6) = 0;      % ay = 0
    else
        x2 = zeros(6, 6);
        x2(1:4, 1:4) = x1;
        x2(5,5) = 0.1;  % Small acceleration variance
        x2(6,6) = 0.1;
    end

% CA to CV conversion
elseif strcmp(modelName1, 'ca') && strcmp(modelName2, 'cv')
    if isvector(x1) && isvector(x2)
        x2(1) = x1(1);  % x
        x2(2) = x1(2);  % y
        x2(3) = x1(3);  % vx
        x2(4) = x1(4);  % vy
    else
        x2 = x1(1:4, 1:4);
    end

% CV to CTRV conversion
elseif strcmp(modelName1, 'cv') && strcmp(modelName2, 'ctrv')
    if isvector(x1) && isvector(x2)
        x2(1) = x1(1);  % x
        x2(2) = x1(2);  % y
        x2(3) = sqrt(x1(3)^2 + x1(4)^2);  % v
        x2(4) = atan2(x1(4), x1(3));      % ψ
        x2(5) = 0;      % ψ̇ = 0
    else
        x2 = zeros(5, 5);
        x2(1,1) = x1(1,1);
        x2(2,2) = x1(2,2);
        x2(3,3) = x1(3,3) + x1(4,4);
        x2(4,4) = deg2rad(5)^2;
        x2(5,5) = deg2rad(0.1)^2;
    end

% CTRV to CV conversion
elseif strcmp(modelName1, 'ctrv') && strcmp(modelName2, 'cv')
    if isvector(x1) && isvector(x2)
        x2(1) = x1(1);  % x
        x2(2) = x1(2);  % y
        x2(3) = x1(3) * cos(x1(4));  % vx
        x2(4) = x1(3) * sin(x1(4));  % vy
    else
        x2 = zeros(4, 4);
        x2(1,1) = x1(1,1);
        x2(2,2) = x1(2,2);
        x2(3,3) = x1(3,3);
        x2(4,4) = x1(3,3);
    end

% CA to CTRV conversion
elseif strcmp(modelName1, 'ca') && strcmp(modelName2, 'ctrv')
    if isvector(x1) && isvector(x2)
        x2(1) = x1(1);  % x
        x2(2) = x1(2);  % y
        x2(3) = sqrt(x1(3)^2 + x1(4)^2);  % v
        x2(4) = atan2(x1(4), x1(3));      % ψ
        x2(5) = 0;      % ψ̇ = 0
    else
        x2 = zeros(5, 5);
        x2(1,1) = x1(1,1);
        x2(2,2) = x1(2,2);
        x2(3,3) = x1(3,3) + x1(4,4);
        x2(4,4) = deg2rad(5)^2;
        x2(5,5) = deg2rad(0.1)^2;
    end

% CTRV to CA conversion
elseif strcmp(modelName1, 'ctrv') && strcmp(modelName2, 'ca')
    if isvector(x1) && isvector(x2)
        x2(1) = x1(1);  % x
        x2(2) = x1(2);  % y
        x2(3) = x1(3) * cos(x1(4));  % vx
        x2(4) = x1(3) * sin(x1(4));  % vy
        x2(5) = 0;      % ax = 0
        x2(6) = 0;      % ay = 0
    else
        x2 = zeros(6, 6);
        x2(1,1) = x1(1,1);
        x2(2,2) = x1(2,2);
        x2(3,3) = x1(3,3);
        x2(4,4) = x1(3,3);
        x2(5,5) = 0.1;
        x2(6,6) = 0.1;
    end

else
    error('Unknown model conversion: %s to %s', modelName1, modelName2);
end
end

%% CA State Transition Function (6-state: [x, y, vx, vy, ax, ay])
function x = caStateTransitionFcn(x, dt)
    F = [1, 0, dt, 0, 0.5*dt^2, 0;
         0, 1, 0, dt, 0, 0.5*dt^2;
         0, 0, 1, 0, dt, 0;
         0, 0, 0, 1, 0, dt;
         0, 0, 0, 0, 1, 0;
         0, 0, 0, 0, 0, 1];
    x = F * x;
end

%% CA Measurement Function
function z = caMeasurementFcn(x)
    z = [x(1); x(2)];  % Position only
end

%% 3-Model Probability Plotting
function plot3ModelProbabilities(data, estimates)
    figure;
    
    % Model probabilities
    subplot(1,2,1);
    t = (0:height(data)-1)';
    plot(t, estimates.cv_prob, 'b-', 'LineWidth', 2, 'DisplayName', 'CV Model');
    hold on;
    plot(t, estimates.ca_prob, 'g-', 'LineWidth', 2, 'DisplayName', 'CA Model');
    plot(t, estimates.ctrv_prob, 'r-', 'LineWidth', 2, 'DisplayName', 'CTRV Model');
    
    % Add segment boundaries if available
    if all(ismember({'segment'}, data.Properties.VariableNames))
        segmentChanges = find(diff(data.segment) ~= 0);
        for i = 1:length(segmentChanges)
            xline(segmentChanges(i)+1, 'k--', 'Alpha', 0.5);
        end
    end
    
    xlabel('Sample Number');
    ylabel('Model Probability');
    title('3-Model IMM Probabilities');
    ylim([0, 1]);
    grid on;
    legend('Location', 'best');
    
    % Model dominance over time
    subplot(1,2,2);
    [~, dominant_model] = max([estimates.cv_prob, estimates.ca_prob, estimates.ctrv_prob], [], 2);
    plot(t, dominant_model, 'ko-', 'LineWidth', 1.5, 'MarkerSize', 3);
    xlabel('Sample Number');
    ylabel('Dominant Model');
    title('Dominant Model Selection');
    yticks([1, 2, 3]);
    yticklabels({'CV', 'CA', 'CTRV'});
    ylim([0.5, 3.5]);
    grid on;
    
    % Save figure
    if ~exist('figures', 'dir')
        mkdir('figures');
    end
    saveas(gcf, 'figures/3model_imm_analysis.png');
    saveas(gcf, 'figures/3model_imm_analysis.fig');
end

%% Keep existing CTRV functions unchanged
function x = ctrvStateTransitionFcn(x, dt)
    px = x(1);
    py = x(2);
    v = x(3);
    psi = x(4);
    psidot = x(5);
    
    if abs(psidot) < 1e-6
        % Straight line motion
        x_new = [px + v * cos(psi) * dt;
                 py + v * sin(psi) * dt;
                 v;
                 psi;
                 psidot];
    else
        % Curved motion
        x_new = [px + v/psidot * (sin(psi + psidot*dt) - sin(psi));
                 py + v/psidot * (-cos(psi + psidot*dt) + cos(psi));
                 v;
                 psi + psidot*dt;
                 psidot];
    end
    
    x = x_new;
end

function z = ctrvMeasurementFcn(x)
    z = [x(1); x(2)];
end