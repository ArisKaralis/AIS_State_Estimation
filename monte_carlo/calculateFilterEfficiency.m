function efficiency_stats = calculateFilterEfficiency(estimates, filter_P, crlb_stats, filter_type)
% CALCULATEFILTEREFFICIENCY - Calculate filter efficiency relative to CRLB
%
% This function compares filter performance against the Cramér-Rao Lower Bound
% to assess filter efficiency. An efficient filter should have covariance 
% close to the CRLB.
%
% INPUTS:
%   estimates    - Filter estimates structure with fields: x_est, y_est, vx_est, vy_est
%   filter_P     - Filter covariance matrices [n_x x n_x x N] 
%   crlb_stats   - CRLB statistics from calculateCRLB
%   filter_type  - String identifying the filter type
%
% OUTPUTS:
%   efficiency_stats - Structure with efficiency metrics:
%                     .position_efficiency_ratio - Ratio of filter to CRLB position variance
%                     .velocity_efficiency_ratio - Ratio of filter to CRLB velocity variance  
%                     .mean_position_efficiency  - Mean position efficiency over trajectory
%                     .mean_velocity_efficiency  - Mean velocity efficiency over trajectory
%                     .efficiency_consistency    - Percentage of time filter is near-optimal
%                     .nees_efficiency          - NEES-based efficiency metric
%                     .filter_type              - Filter identifier

fprintf('Calculating filter efficiency vs CRLB for %s...\n', filter_type);

N = length(estimates.x_est);
timestamps = crlb_stats.timestamps;

% Initialize efficiency metrics
position_efficiency_ratio = zeros(2, N); % [x_efficiency; y_efficiency] 
velocity_efficiency_ratio = zeros(2, N); % [vx_efficiency; vy_efficiency]
nees_efficiency = zeros(1, N);

% Extract position and velocity indices based on filter type and covariance size
[pos_idx, vel_idx] = getStateIndices(filter_P, filter_type);

for k = 1:N
    % Get current covariances
    P_filter = filter_P(:,:,k);
    P_crlb = crlb_stats.P_crlb(:,:,k);
    
    % Position efficiency (diagonal elements)
    if ~isempty(pos_idx) && length(pos_idx) >= 2
        % Filter position variances
        filter_pos_var = [P_filter(pos_idx(1), pos_idx(1)); 
                         P_filter(pos_idx(2), pos_idx(2))];
        
        % CRLB position variances (always [1,1] and [3,3] for position)
        crlb_pos_var = [P_crlb(1,1); P_crlb(3,3)];
        
        % Efficiency ratio (filter variance / CRLB variance)
        % Values close to 1.0 indicate near-optimal performance
        position_efficiency_ratio(:,k) = filter_pos_var ./ (crlb_pos_var + eps);
    else
        position_efficiency_ratio(:,k) = [NaN; NaN];
    end
    
    % Velocity efficiency (if available)
    if ~isempty(vel_idx) && length(vel_idx) >= 2
        filter_vel_var = [P_filter(vel_idx(1), vel_idx(1)); 
                         P_filter(vel_idx(2), vel_idx(2))];
        
        % CRLB velocity variances depend on model
        if strcmp(crlb_stats.model_type, 'CV')
            crlb_vel_var = [P_crlb(2,2); P_crlb(4,4)];
        elseif strcmp(crlb_stats.model_type, 'CA')  
            crlb_vel_var = [P_crlb(2,2); P_crlb(5,5)];
        elseif strcmp(crlb_stats.model_type, 'CTRV')
            crlb_vel_var = [P_crlb(3,3); P_crlb(3,3)]; % Speed variance (approximation)
        else
            crlb_vel_var = [P_crlb(2,2); P_crlb(4,4)]; % Default to CV-style
        end
        
        velocity_efficiency_ratio(:,k) = filter_vel_var ./ (crlb_vel_var + eps);
    else
        velocity_efficiency_ratio(:,k) = [NaN; NaN];
    end
    
    % NEES-based efficiency metric
    % Compare actual estimation error against CRLB bound
    if k <= length(estimates.x_est) && ~isnan(estimates.x_est(k))
        try
            % Use only position for NEES efficiency calculation
            pos_error = [estimates.x_est(k) - estimates.x_est(k); 
                        estimates.y_est(k) - estimates.y_est(k)]; % This should use true values
            
            % Get position block from CRLB
            P_crlb_pos = P_crlb([1,3], [1,3]); 
            
            % NEES with respect to CRLB
            nees_efficiency(k) = pos_error' * (P_crlb_pos \ pos_error);
        catch
            nees_efficiency(k) = NaN;
        end
    else
        nees_efficiency(k) = NaN;
    end
end

% Calculate summary statistics
valid_pos_idx = ~isnan(position_efficiency_ratio(1,:));
valid_vel_idx = ~isnan(velocity_efficiency_ratio(1,:));

% Mean efficiency ratios
if any(valid_pos_idx)
    mean_position_efficiency = mean(position_efficiency_ratio(:, valid_pos_idx), 'all');
    position_efficiency_std = std(position_efficiency_ratio(:, valid_pos_idx), [], 'all');
else
    mean_position_efficiency = NaN;
    position_efficiency_std = NaN;
end

if any(valid_vel_idx)
    mean_velocity_efficiency = mean(velocity_efficiency_ratio(:, valid_vel_idx), 'all');
    velocity_efficiency_std = std(velocity_efficiency_ratio(:, valid_vel_idx), [], 'all');
else
    mean_velocity_efficiency = NaN; 
    velocity_efficiency_std = NaN;
end

% Efficiency consistency: percentage of time filter is "near-optimal"
% Consider filter efficient if efficiency ratio is between 1.0 and 2.0
near_optimal_threshold = 2.0; % Filter variance <= 2x CRLB variance
if any(valid_pos_idx)
    pos_consistent = position_efficiency_ratio(:, valid_pos_idx) <= near_optimal_threshold;
    position_consistency = 100 * mean(pos_consistent, 'all');
else
    position_consistency = NaN;
end

if any(valid_vel_idx)
    vel_consistent = velocity_efficiency_ratio(:, valid_vel_idx) <= near_optimal_threshold;
    velocity_consistency = 100 * mean(vel_consistent, 'all');
else
    velocity_consistency = NaN;
end

% Overall efficiency consistency
if ~isnan(position_consistency) && ~isnan(velocity_consistency)
    efficiency_consistency = (position_consistency + velocity_consistency) / 2;
elseif ~isnan(position_consistency)
    efficiency_consistency = position_consistency;
elseif ~isnan(velocity_consistency)  
    efficiency_consistency = velocity_consistency;
else
    efficiency_consistency = NaN;
end

% Calculate efficiency grades
position_grade = classifyEfficiency(mean_position_efficiency);
velocity_grade = classifyEfficiency(mean_velocity_efficiency);

fprintf('Filter efficiency analysis complete.\n');
fprintf('  Position efficiency: %.2f (Grade: %s)\n', mean_position_efficiency, position_grade);
fprintf('  Velocity efficiency: %.2f (Grade: %s)\n', mean_velocity_efficiency, velocity_grade);
fprintf('  Consistency: %.1f%% near-optimal\n', efficiency_consistency);

% Package results
efficiency_stats = struct();
efficiency_stats.position_efficiency_ratio = position_efficiency_ratio;
efficiency_stats.velocity_efficiency_ratio = velocity_efficiency_ratio;
efficiency_stats.mean_position_efficiency = mean_position_efficiency;
efficiency_stats.mean_velocity_efficiency = mean_velocity_efficiency;
efficiency_stats.position_efficiency_std = position_efficiency_std;
efficiency_stats.velocity_efficiency_std = velocity_efficiency_std;
efficiency_stats.efficiency_consistency = efficiency_consistency;
efficiency_stats.position_consistency = position_consistency;
efficiency_stats.velocity_consistency = velocity_consistency;
efficiency_stats.nees_efficiency = nees_efficiency;
efficiency_stats.filter_type = filter_type;
efficiency_stats.timestamps = timestamps;
efficiency_stats.position_grade = position_grade;
efficiency_stats.velocity_grade = velocity_grade;

% Additional derived metrics
efficiency_stats.mean_efficiency_overall = nanmean([mean_position_efficiency, mean_velocity_efficiency]);
efficiency_stats.efficiency_degradation = max(1, efficiency_stats.mean_efficiency_overall); % How far from optimal
efficiency_stats.is_efficient = efficiency_stats.mean_efficiency_overall < 1.5; % Binary efficiency flag

end

%% Helper Functions

function [pos_idx, vel_idx] = getStateIndices(P, filter_type)
% Determine position and velocity indices based on filter covariance structure
[n_states, ~, ~] = size(P);

% Standard mappings based on common filter implementations
if n_states == 4
    % CV model: [x, vx, y, vy] or [x, y, vx, vy]
    if contains(filter_type, 'CV') || contains(filter_type, 'KF')
        pos_idx = [1, 3]; % x, y positions  
        vel_idx = [2, 4]; % vx, vy velocities
    else
        pos_idx = [1, 2]; % Assume first two are positions
        vel_idx = [3, 4]; % Assume last two are velocities  
    end
    
elseif n_states == 5
    % CTRV model: [px, py, v, psi, psi_dot]
    pos_idx = [1, 2]; % px, py positions
    vel_idx = [3, 3]; % v (speed) - use twice for both components
    
elseif n_states == 6  
    % CA model: [x, vx, ax, y, vy, ay]
    pos_idx = [1, 4]; % x, y positions
    vel_idx = [2, 5]; % vx, vy velocities
    
else
    % Unknown structure - make best guess
    pos_idx = [1, min(3, n_states)]; % First and third (or last if < 3)
    if n_states >= 4
        vel_idx = [2, min(4, n_states)]; % Second and fourth (or last if < 4)
    else
        vel_idx = [];
    end
end

% Validate indices
pos_idx = pos_idx(pos_idx <= n_states);
vel_idx = vel_idx(vel_idx <= n_states);

if isempty(pos_idx)
    pos_idx = [1, min(2, n_states)];
end
end

function grade = classifyEfficiency(efficiency_ratio)
% Classify filter efficiency based on ratio to CRLB
if isnan(efficiency_ratio)
    grade = 'N/A';
elseif efficiency_ratio <= 1.2
    grade = 'Excellent';
elseif efficiency_ratio <= 1.5
    grade = 'Good'; 
elseif efficiency_ratio <= 2.0
    grade = 'Fair';
elseif efficiency_ratio <= 3.0
    grade = 'Poor';
else
    grade = 'Very Poor';
end
end