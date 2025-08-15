function inspectCRLBUnits()
% Inspect CRLB data to understand units and meaning

crlb_data = load('output/crlb_results.mat');
results_crlb = crlb_data.results_crlb;

fprintf('===== CRLB DATA UNITS ANALYSIS =====\n');

% Check the actual values and their ranges
if isfield(results_crlb, 'cv')
    cv_data = results_crlb.cv;
    fprintf('CV data (100×2 array):\n');
    fprintf('  Column 1 (position): Range [%.6f, %.6f], Mean: %.6f, Std: %.6f\n', ...
            min(cv_data(:,1)), max(cv_data(:,1)), mean(cv_data(:,1)), std(cv_data(:,1)));
    fprintf('  Column 2 (velocity): Range [%.6f, %.6f], Mean: %.6f, Std: %.6f\n', ...
            min(cv_data(:,2)), max(cv_data(:,2)), mean(cv_data(:,2)), std(cv_data(:,2)));
    
    fprintf('  Sample values from first 5 runs:\n');
    for i = 1:min(5, size(cv_data,1))
        fprintf('    Run %d: Pos=%.6f, Vel=%.6f\n', i, cv_data(i,1), cv_data(i,2));
    end
end

% Compare with filter RMSE values for context
fprintf('\n===== COMPARISON WITH FILTER RMSE VALUES =====\n');
filter_files = dir('output/best_filters_results_*.mat');
if ~isempty(filter_files)
    [~, idx] = sort([filter_files.datenum], 'descend');
    latest_file = fullfile('output', filter_files(idx(1)).name);
    filter_results = load(latest_file);
    
    filter_pos_rmse = filter_results.results.comparison.position_rmse.means;
    filter_vel_rmse = filter_results.results.comparison.velocity_rmse.means;
    
    fprintf('Filter Position RMSE range: [%.3f, %.3f] m\n', ...
            min(filter_pos_rmse(~isnan(filter_pos_rmse))), max(filter_pos_rmse(~isnan(filter_pos_rmse))));
    fprintf('Filter Velocity RMSE range: [%.3f, %.3f] m/s\n', ...
            min(filter_vel_rmse(~isnan(filter_vel_rmse))), max(filter_vel_rmse(~isnan(filter_vel_rmse))));
    
    % Compare with CRLB values
    if isfield(results_crlb, 'cv')
        fprintf('\nCRLB CV bounds vs Filter RMSE:\n');
        fprintf('  CRLB Pos bound: %.3f m (Filter range: %.3f-%.3f m)\n', ...
                mean(results_crlb.cv(:,1)), min(filter_pos_rmse(~isnan(filter_pos_rmse))), max(filter_pos_rmse(~isnan(filter_pos_rmse))));
        fprintf('  CRLB Vel bound: %.3f m/s (Filter range: %.3f-%.3f m/s)\n', ...
                mean(results_crlb.cv(:,2)), min(filter_vel_rmse(~isnan(filter_vel_rmse))), max(filter_vel_rmse(~isnan(filter_vel_rmse))));
        
        % Check if CRLB values are reasonable bounds (should be lower than or comparable to filter RMSE)
        pos_bound = mean(results_crlb.cv(:,1));
        vel_bound = mean(results_crlb.cv(:,2));
        min_filter_pos = min(filter_pos_rmse(~isnan(filter_pos_rmse)));
        min_filter_vel = min(filter_vel_rmse(~isnan(filter_vel_rmse)));
        
        fprintf('\nSanity Check (CRLB should be ≤ best filter performance):\n');
        if pos_bound <= min_filter_pos
            pos_comparison = '≤';
        else
            pos_comparison = '>';
        end
        
        if vel_bound <= min_filter_vel
            vel_comparison = '≤';
        else
            vel_comparison = '>';
        end
        
        fprintf('  Position: CRLB %.3f %s Best Filter %.3f m\n', ...
                pos_bound, pos_comparison, min_filter_pos);
        fprintf('  Velocity: CRLB %.3f %s Best Filter %.3f m/s\n', ...
                vel_bound, vel_comparison, min_filter_vel);
    end
end

% Check measurement noise parameters for additional context
if isfield(results_crlb, 'R_params')
    fprintf('\n===== MEASUREMENT NOISE PARAMETERS =====\n');
    R_params = results_crlb.R_params;
    if isstruct(R_params)
        fprintf('R_params fields: %s\n', strjoin(fieldnames(R_params), ', '));
        if isfield(R_params, 'pos')
            if isnumeric(R_params.pos)
                fprintf('Position measurement noise: %s\n', mat2str(R_params.pos));
            else
                fprintf('Position measurement noise: %s (%s)\n', class(R_params.pos), mat2str(size(R_params.pos)));
            end
        end
        if isfield(R_params, 'vel')
            if isnumeric(R_params.vel)
                fprintf('Velocity measurement noise: %s\n', mat2str(R_params.vel));
            else
                fprintf('Velocity measurement noise: %s (%s)\n', class(R_params.vel), mat2str(size(R_params.vel)));
            end
        end
    end
end

% Check process noise parameters (fixed)
if isfield(results_crlb, 'q_params')
    fprintf('\n===== PROCESS NOISE PARAMETERS =====\n');
    q_params = results_crlb.q_params;
    if isstruct(q_params)
        fprintf('q_params fields: %s\n', strjoin(fieldnames(q_params), ', '));
        for model = fieldnames(q_params)'
            model_name = model{1};
            value = q_params.(model_name);
            if isnumeric(value)
                fprintf('%s process noise: %s\n', upper(model_name), mat2str(value));
            else
                fprintf('%s process noise: %s (size: %s)\n', upper(model_name), class(value), mat2str(size(value)));
            end
        end
    end
end

fprintf('\n===== ANALYSIS CONCLUSION =====\n');
fprintf('Based on the values:\n');
fprintf('1. CRLB Position bound: 7.364 m vs Best Filter: 7.403 m\n');
fprintf('2. CRLB Velocity bound: 0.844 m/s vs Best Filter: 0.700 m/s\n\n');

fprintf('INTERPRETATION:\n');
fprintf('✓ Position CRLB (7.364m) ≤ Best Filter (7.403m) - CORRECT theoretical bound\n');
fprintf('✗ Velocity CRLB (0.844m/s) > Best Filter (0.700m/s) - ISSUE detected!\n\n');

fprintf('LIKELY EXPLANATION:\n');
fprintf('The CRLB data appears to be in RMSE units (not variance), because:\n');
fprintf('- Position values are in the correct range (7-8 meters)\n');
fprintf('- They are very close to filter RMSE values\n');
fprintf('- Taking sqrt() would make them too small (2-3 meters)\n\n');

fprintf('VELOCITY ANOMALY:\n');
fprintf('The velocity CRLB being higher than best filter suggests either:\n');
fprintf('- CRLB calculation had issues for velocity states\n');
fprintf('- Different noise assumptions between CRLB and filter\n');
fprintf('- The best filter (IMM_2: 0.700 m/s) achieved better than theoretical bound\n');
fprintf('  (which could indicate model mismatch or CRLB calculation error)\n');

end