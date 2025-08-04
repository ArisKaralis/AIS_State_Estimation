function testFilterWithMultipleQ()
% TESTFILTERWITHMULTIPLEQ - Test a single filter with multiple q values
% This script runs one filter with different process noise values to find optimal tuning

close all;
addpath('functions');

% Display banner
fprintf('\n========= Filter Q-Value Optimization =========\n\n');

% Load the AIS data
fprintf('Loading AIS data...\n');
dataPath = fullfile('data', 'monte_carlo_sample.csv');
if ~exist(dataPath, 'file')
    error('AIS data file not found. Run simulateSpecificAISTrack first.');
end

data = readtable(dataPath);
fprintf('Loaded %d AIS points\n', height(data));

% Define q values to test
q_min = 0.001;
q_max = 1;
num_q_values = 50;
q_values = linspace(q_min, q_max, num_q_values);

% Choose which filter to test (change this to test different filters)
filter_to_test = 'EKF-CTRV';  % Options: 'KF', 'EKF-CV', 'EKF-CA', 'EKF-CTRV', 'UKF-CV', 'UKF-CTRV' 

fprintf('Selected filter: %s\n', filter_to_test);

% Pre-allocate results
results = struct();
results.q_values = q_values;
results.pos_rmse = zeros(1, num_q_values);
results.vel_rmse = zeros(1, num_q_values);
results.sog_rmse = zeros(1, num_q_values);
results.cog_rmse = zeros(1, num_q_values);

fprintf('\nTesting %s filter with %d different q values...\n', filter_to_test, num_q_values);
fprintf('Q values range from %.3f to %.3f\n\n', q_min, q_max);

% Test each q value
for i = 1:num_q_values
    q = q_values(i);
    
    fprintf('Testing q = %.3f (%d/%d)... ', q, i, num_q_values);
    
    try
        % Run the selected filter
        switch filter_to_test
            case 'KF'
                [estimates, stats] = runKF(data, q);
            case 'EKF-CV'
                [estimates, stats] = runEKFCV(data, q);
            case 'EKF-CA'
                [estimates, stats] = runEKFCA(data, q);
            case 'EKF-CTRV'
                [estimates, stats] = runEKFCTRV(data, q);
            case 'UKF-CV'
                [estimates, stats] = runUKFCV(data, q);
            case 'UKF-CTRV'
                [estimates, stats] = runUKFCTRV(data, q);
            case 'IMM'
                [estimates, stats] = runIMMFilter(data, q, q, q);
            otherwise
                error('Unknown filter type: %s', filter_to_test);
        end
        
        % Store results - CORRECTED FIELD NAMES
        results.pos_rmse(i) = stats.position_rmse;    % Fixed field name
        results.vel_rmse(i) = stats.velocity_rmse;    % Fixed field name
        results.sog_rmse(i) = stats.sog_rmse;         % Fixed field name
        results.cog_rmse(i) = stats.cog_rmse;         % Fixed field name
        
        fprintf('RMSE: Pos=%.2f, Vel=%.2f, SOG=%.2f, COG=%.2f\n', ...
                results.pos_rmse(i), results.vel_rmse(i), ...
                results.sog_rmse(i), results.cog_rmse(i));
        
    catch ME
        fprintf('FAILED: %s\n', ME.message);
        results.pos_rmse(i) = NaN;
        results.vel_rmse(i) = NaN;
        results.sog_rmse(i) = NaN;
        results.cog_rmse(i) = NaN;
    end
end

% Find optimal q values
[min_pos_rmse, min_pos_idx] = min(results.pos_rmse);
[min_vel_rmse, min_vel_idx] = min(results.vel_rmse);
[min_sog_rmse, min_sog_idx] = min(results.sog_rmse);
[min_cog_rmse, min_cog_idx] = min(results.cog_rmse);

% Calculate combined score (equal weighting)
valid_idx = ~isnan(results.pos_rmse);
if sum(valid_idx) > 0
    % Normalize each metric by its maximum value
    norm_pos = results.pos_rmse / max(results.pos_rmse(valid_idx));
    norm_vel = results.vel_rmse / max(results.vel_rmse(valid_idx));
    norm_sog = results.sog_rmse / max(results.sog_rmse(valid_idx));
    norm_cog = results.cog_rmse / max(results.cog_rmse(valid_idx));
    
    % Combined score (lower is better)
    combined_score = norm_pos + norm_vel + norm_sog + norm_cog;
    [min_combined_score, min_combined_idx] = min(combined_score);
    
    optimal_q = q_values(min_combined_idx);
else
    optimal_q = NaN;
end

% Display results
fprintf('\n========= OPTIMIZATION RESULTS =========\n');
fprintf('Filter tested: %s\n', filter_to_test);
fprintf('Q values tested: %d (from %.3f to %.3f)\n', num_q_values, q_min, q_max);
fprintf('\nBest q values for each metric:\n');
fprintf('Position RMSE: q = %.3f (RMSE = %.2f m)\n', q_values(min_pos_idx), min_pos_rmse);
fprintf('Velocity RMSE: q = %.3f (RMSE = %.2f m/s)\n', q_values(min_vel_idx), min_vel_rmse);
fprintf('SOG RMSE:      q = %.3f (RMSE = %.2f m/s)\n', q_values(min_sog_idx), min_sog_rmse);
fprintf('COG RMSE:      q = %.3f (RMSE = %.2f deg)\n', q_values(min_cog_idx), min_cog_rmse);

if ~isnan(optimal_q)
    fprintf('\nOptimal q (combined score): %.3f\n', optimal_q);
    fprintf('Combined score: %.3f\n', min_combined_score);
else
    fprintf('\nNo valid results found!\n');
end

% Create visualization
figure;

% Plot position RMSE vs q
subplot(2, 2, 1);
builtin('plot', q_values, results.pos_rmse, 'b-', 'LineWidth', 2, 'MarkerSize', 6);
xlabel('Process Noise Intensity (q)');
ylabel('Position RMSE (m)');
title('Position RMSE vs Process Noise');
grid on;
hold on;
builtin('plot', q_values(min_pos_idx), min_pos_rmse, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
legend('RMSE', 'Optimal q', 'Location', 'best');

% Plot velocity RMSE vs q
subplot(2, 2, 2);
builtin('plot', q_values, results.vel_rmse, 'g-o', 'LineWidth', 2, 'MarkerSize', 6);
xlabel('Process Noise Intensity (q)');
ylabel('Velocity RMSE (m/s)');
title('Velocity RMSE vs Process Noise');
grid on;
hold on;
builtin('plot', q_values(min_vel_idx), min_vel_rmse, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
legend('RMSE', 'Optimal q', 'Location', 'best');

% Plot SOG RMSE vs q
subplot(2, 2, 3);
builtin('plot', q_values, results.sog_rmse, 'k-o', 'LineWidth', 2, 'MarkerSize', 6);
xlabel('Process Noise Intensity (q)');
ylabel('SOG RMSE (m/s)');
title('SOG RMSE vs Process Noise');
grid on;
hold on;
builtin('plot', q_values(min_sog_idx), min_sog_rmse, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
legend('RMSE', 'Optimal q', 'Location', 'best');

% Plot COG RMSE vs q
subplot(2, 2, 4);
builtin('plot', q_values, results.cog_rmse, 'm-o', 'LineWidth', 2, 'MarkerSize', 6);
xlabel('Process Noise Intensity (q)');
ylabel('COG RMSE (degrees)');
title('COG RMSE vs Process Noise');
grid on;
hold on;
builtin('plot', q_values(min_cog_idx), min_cog_rmse, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
legend('RMSE', 'Optimal q', 'Location', 'best');

sgtitle(sprintf('%s Filter: Performance vs Process Noise Intensity', filter_to_test));

% Save results
if ~exist('figures', 'dir')
    mkdir('figures');
end

filename = sprintf('q_optimization_%s', lower(filter_to_test));
saveas(gcf, fullfile('figures', [filename '.png']));
saveas(gcf, fullfile('figures', [filename '.fig']));

% Save data
save(fullfile('figures', [filename '_data.mat']), 'results', 'filter_to_test', 'optimal_q');

fprintf('\nResults saved to: %s\n', fullfile('figures', [filename '_data.mat']));
fprintf('Plots saved to: %s\n', fullfile('figures', [filename '.png']));

% Also create a combined score plot
figure;
if ~isnan(optimal_q)
    builtin('plot', q_values, combined_score, 'r-o', 'LineWidth', 2, 'MarkerSize', 6);
    hold on;
    builtin('plot', optimal_q, min_combined_score, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    xlabel('Process Noise Intensity (q)');
    ylabel('Combined Normalized Score');
    title(sprintf('%s Filter: Combined Performance Score vs Process Noise', filter_to_test));
    legend('Combined Score', 'Optimal q', 'Location', 'best');
    grid on;
    
    saveas(gcf, fullfile('figures', [filename '_combined.png']));
    saveas(gcf, fullfile('figures', [filename '_combined.fig']));
end

fprintf('\n========= Optimization Complete =========\n');

end