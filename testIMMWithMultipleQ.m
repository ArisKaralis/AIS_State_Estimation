function testIMMWithMultipleQ()
% TESTIMMWITHMULTIPLEQ - Test IMM filter with multiple Q value combinations
% This script optimizes the Q values for a 2-model IMM filter (CV and CTRV only)

close all;
addpath('functions');

% Display banner
fprintf('\n========= 2-Model IMM Filter Q-Value Optimization =========\n\n');

% Load the AIS data
fprintf('Loading AIS data...\n');
dataPath = fullfile('data', 'monte_carlo_sample.csv');
if ~exist(dataPath, 'file')
    error('AIS data file not found. Run simulateSpecificAISTrack first.');
end

data = readtable(dataPath);
fprintf('Loaded %d AIS points\n', height(data));

% Define Q value ranges for each model (CV and CTRV only)
q_min = 0.01;
q_max = 1.2;
num_q_values = 30;  % Can increase for more thorough optimization

q_values = linspace(q_min, q_max, num_q_values);

fprintf('Testing 2-model IMM filter (CV + CTRV) with %d^2 = %d Q value combinations...\n', num_q_values, num_q_values^2);
fprintf('Q values range from %.3f to %.3f for each model (CV, CTRV)\n\n', q_min, q_max);

% Pre-allocate results
total_combinations = num_q_values^2;
results = struct();
results.q_cv = zeros(1, total_combinations);
results.q_ctrv = zeros(1, total_combinations);
results.pos_rmse = zeros(1, total_combinations);
results.vel_rmse = zeros(1, total_combinations);
results.sog_rmse = zeros(1, total_combinations);
results.cog_rmse = zeros(1, total_combinations);
results.combined_score = zeros(1, total_combinations);

% Test all combinations
combo_idx = 1;
start_time = tic;

for i = 1:num_q_values
    for j = 1:num_q_values
        q_cv = q_values(i);
        q_ctrv = q_values(j);
        
        fprintf('Testing combination %d/%d: q_cv=%.3f, q_ctrv=%.3f... ', ...
                combo_idx, total_combinations, q_cv, q_ctrv);
        
        try
            % Run 2-model IMM filter with current Q values
            [estimates, stats] = runIMMEKF2(data, q_cv, q_ctrv);
            
            % Store Q values
            results.q_cv(combo_idx) = q_cv;
            results.q_ctrv(combo_idx) = q_ctrv;
            
            % Store performance metrics
            results.pos_rmse(combo_idx) = stats.position_rmse;
            results.vel_rmse(combo_idx) = stats.velocity_rmse;
            results.sog_rmse(combo_idx) = stats.sog_rmse;
            results.cog_rmse(combo_idx) = stats.cog_rmse;
            
            fprintf('RMSE: Pos=%.2f, Vel=%.2f, SOG=%.2f, COG=%.2f\n', ...
                    results.pos_rmse(combo_idx), results.vel_rmse(combo_idx), ...
                    results.sog_rmse(combo_idx), results.cog_rmse(combo_idx));
            
        catch ME
            fprintf('FAILED: %s\n', ME.message);
            results.pos_rmse(combo_idx) = NaN;
            results.vel_rmse(combo_idx) = NaN;
            results.sog_rmse(combo_idx) = NaN;
            results.cog_rmse(combo_idx) = NaN;
        end
        
        combo_idx = combo_idx + 1;
        
        % Progress update every 20 combinations
        if mod(combo_idx-1, 20) == 0
            elapsed = toc(start_time);
            avg_time = elapsed / (combo_idx-1);
            remaining = (total_combinations - combo_idx + 1) * avg_time;
            fprintf('Progress: %.1f%% complete, ETA: %.1f minutes\n', ...
                    (combo_idx-1)/total_combinations*100, remaining/60);
        end
    end
end

% Calculate combined score (equal weighting, lower is better)
valid_idx = ~isnan(results.pos_rmse);
if sum(valid_idx) > 0
    % Normalize each metric by its maximum value
    max_pos = max(results.pos_rmse(valid_idx));
    max_vel = max(results.vel_rmse(valid_idx));
    max_sog = max(results.sog_rmse(valid_idx));
    max_cog = max(results.cog_rmse(valid_idx));
    
    norm_pos = results.pos_rmse / max_pos;
    norm_vel = results.vel_rmse / max_vel;
    norm_sog = results.sog_rmse / max_sog;
    norm_cog = results.cog_rmse / max_cog;
    
    % Combined score (lower is better)
    results.combined_score = norm_pos + norm_vel + norm_sog + norm_cog;
    
    % Find optimal combinations
    [min_pos_rmse, min_pos_idx] = min(results.pos_rmse);
    [min_vel_rmse, min_vel_idx] = min(results.vel_rmse);
    [min_sog_rmse, min_sog_idx] = min(results.sog_rmse);
    [min_cog_rmse, min_cog_idx] = min(results.cog_rmse);
    [min_combined_score, min_combined_idx] = min(results.combined_score);
    
    % Display results
    fprintf('\n========= 2-MODEL IMM OPTIMIZATION RESULTS =========\n');
    fprintf('Total combinations tested: %d\n', sum(valid_idx));
    fprintf('Failed combinations: %d\n', sum(~valid_idx));
    
    fprintf('\nBest Q value combinations for each metric:\n');
    fprintf('Position RMSE: q_cv=%.3f, q_ctrv=%.3f (RMSE=%.2f m)\n', ...
            results.q_cv(min_pos_idx), results.q_ctrv(min_pos_idx), min_pos_rmse);
    fprintf('Velocity RMSE: q_cv=%.3f, q_ctrv=%.3f (RMSE=%.2f m/s)\n', ...
            results.q_cv(min_vel_idx), results.q_ctrv(min_vel_idx), min_vel_rmse);
    fprintf('SOG RMSE:      q_cv=%.3f, q_ctrv=%.3f (RMSE=%.2f m/s)\n', ...
            results.q_cv(min_sog_idx), results.q_ctrv(min_sog_idx), min_sog_rmse);
    fprintf('COG RMSE:      q_cv=%.3f, q_ctrv=%.3f (RMSE=%.2f deg)\n', ...
            results.q_cv(min_cog_idx), results.q_ctrv(min_cog_idx), min_cog_rmse);
    
    fprintf('\nOptimal combination (combined score):\n');
    fprintf('q_cv=%.3f, q_ctrv=%.3f\n', ...
            results.q_cv(min_combined_idx), results.q_ctrv(min_combined_idx));
    fprintf('Combined score: %.3f\n', min_combined_score);
    fprintf('Performance: Pos=%.2f m, Vel=%.2f m/s, SOG=%.2f m/s, COG=%.2f deg\n', ...
            results.pos_rmse(min_combined_idx), results.vel_rmse(min_combined_idx), ...
            results.sog_rmse(min_combined_idx), results.cog_rmse(min_combined_idx));
    
    % Create visualizations
    create2ModelIMMOptimizationPlots(results, min_combined_idx);
    
    % Save results
    save_results_filename = save2ModelIMMOptimizationResults(results);
    fprintf('\nResults saved to: %s\n', save_results_filename);
    
else
    fprintf('\nNo valid results found!\n');
end

total_time = toc(start_time);
fprintf('\nTotal optimization time: %.1f minutes\n', total_time/60);
fprintf('========= 2-Model IMM Optimization Complete =========\n');

end

function create2ModelIMMOptimizationPlots(results, optimal_idx)
% Create visualization plots for 2-model IMM optimization results

if ~exist('figures', 'dir')
    mkdir('figures');
end

valid_idx = ~isnan(results.pos_rmse);

% Figure 1: 2D visualization of Q values colored by performance metrics
figure('Name', '2-Model IMM Q-Value Optimization');

subplot(2, 3, 1);
scatter(results.q_cv(valid_idx), results.q_ctrv(valid_idx), ...
        50, results.combined_score(valid_idx), 'filled');
hold on;
scatter(results.q_cv(optimal_idx), results.q_ctrv(optimal_idx), ...
        200, 'r', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
colorbar;
xlabel('Q CV');
ylabel('Q CTRV');
title('Combined Score vs Q Values');
colormap(gca, 'jet');
grid on;

% Individual metric plots
metrics = {'pos_rmse', 'vel_rmse', 'sog_rmse', 'cog_rmse'};
metric_names = {'Position RMSE (m)', 'Velocity RMSE (m/s)', 'SOG RMSE (m/s)', 'COG RMSE (deg)'};

for i = 1:4
    subplot(2, 3, i+1);
    scatter(results.q_cv(valid_idx), results.q_ctrv(valid_idx), ...
            50, results.(metrics{i})(valid_idx), 'filled');
    hold on;
    scatter(results.q_cv(optimal_idx), results.q_ctrv(optimal_idx), ...
            200, 'r', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
    colorbar;
    xlabel('Q CV');
    ylabel('Q CTRV');
    title(metric_names{i});
    colormap(gca, 'jet');
    grid on;
end

% Add performance annotations
subplot(2, 3, 6);
axis off;
text(0.1, 0.9, 'Optimal Performance:', 'FontSize', 14, 'FontWeight', 'bold');
text(0.1, 0.8, sprintf('Q_{CV} = %.3f', results.q_cv(optimal_idx)), 'FontSize', 12);
text(0.1, 0.7, sprintf('Q_{CTRV} = %.3f', results.q_ctrv(optimal_idx)), 'FontSize', 12);
text(0.1, 0.6, sprintf('Position RMSE: %.2f m', results.pos_rmse(optimal_idx)), 'FontSize', 12);
text(0.1, 0.5, sprintf('Velocity RMSE: %.2f m/s', results.vel_rmse(optimal_idx)), 'FontSize', 12);
text(0.1, 0.4, sprintf('SOG RMSE: %.2f m/s', results.sog_rmse(optimal_idx)), 'FontSize', 12);
text(0.1, 0.3, sprintf('COG RMSE: %.2f deg', results.cog_rmse(optimal_idx)), 'FontSize', 12);
text(0.1, 0.2, sprintf('Combined Score: %.3f', results.combined_score(optimal_idx)), 'FontSize', 12);

sgtitle('2-Model IMM Filter: Q-Value Optimization Results (Red = Optimal)');
saveas(gcf, 'figures/imm_2model_q_optimization.png');
saveas(gcf, 'figures/imm_2model_q_optimization.fig');

% Figure 2: Performance analysis
figure('Name', '2-Model IMM Performance Analysis' );

% Performance distribution histograms
subplot(2, 3, 1);
histogram(results.combined_score(valid_idx), 20, 'FaceColor', 'blue', 'EdgeColor', 'black');
hold on;
xline(results.combined_score(optimal_idx), 'r-', 'LineWidth', 3, 'Label', 'Optimal');
xlabel('Combined Score');
ylabel('Frequency');
title('Distribution of Combined Scores');
grid on;

% Best vs worst comparison
[~, worst_idx] = max(results.combined_score(valid_idx));
worst_global_idx = find(valid_idx);
worst_global_idx = worst_global_idx(worst_idx);

subplot(2, 3, 2);
metrics_best = [results.pos_rmse(optimal_idx), results.vel_rmse(optimal_idx), ...
               results.sog_rmse(optimal_idx), results.cog_rmse(optimal_idx)];
metrics_worst = [results.pos_rmse(worst_global_idx), results.vel_rmse(worst_global_idx), ...
                results.sog_rmse(worst_global_idx), results.cog_rmse(worst_global_idx)];

x_pos = 1:4;
bar_width = 0.35;
bar(x_pos - bar_width/2, metrics_best, bar_width, 'FaceColor', 'green', 'DisplayName', 'Best');
hold on;
bar(x_pos + bar_width/2, metrics_worst, bar_width, 'FaceColor', 'red', 'DisplayName', 'Worst');
xlabel('Metric');
ylabel('RMSE Value');
title('Best vs Worst Performance');
xticks(x_pos);
xticklabels({'Pos (m)', 'Vel (m/s)', 'SOG (m/s)', 'COG (deg)'});
legend('Location', 'best');
grid on;

% Q value distributions - FIXED VERSION
subplot(2, 3, 3);
% Create proper data for boxplot
q_cv_data = results.q_cv(valid_idx);
q_ctrv_data = results.q_ctrv(valid_idx);

% Combine data and create group labels
all_q_values = [q_cv_data(:); q_ctrv_data(:)];
group_labels = [repmat({'Q CV'}, length(q_cv_data), 1); ...
                repmat({'Q CTRV'}, length(q_ctrv_data), 1)];

boxplot(all_q_values, group_labels);
hold on;
plot(1, results.q_cv(optimal_idx), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot(2, results.q_ctrv(optimal_idx), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
ylabel('Q Value');
title('Q Value Distributions (Red = Optimal)');
grid on;

% CV vs CTRV model preference
subplot(2, 3, 4);
% Create a heatmap showing performance landscape
q_unique_cv = unique(results.q_cv);
q_unique_ctrv = unique(results.q_ctrv);
[Q_CV, Q_CTRV] = meshgrid(q_unique_cv, q_unique_ctrv);

% Reshape combined scores for heatmap
score_matrix = reshape(results.combined_score, length(q_unique_cv), length(q_unique_ctrv))';
imagesc(q_unique_cv, q_unique_ctrv, score_matrix);
colorbar;
xlabel('Q CV');
ylabel('Q CTRV');
title('Performance Landscape');
hold on;
plot(results.q_cv(optimal_idx), results.q_ctrv(optimal_idx), 'r*', 'MarkerSize', 15, 'LineWidth', 3);

% Model sensitivity analysis
subplot(2, 3, 5);
% Show how performance changes with CV model Q values (fixing CTRV at optimal)
optimal_q_ctrv = results.q_ctrv(optimal_idx);
ctrv_fixed_idx = find(abs(results.q_ctrv - optimal_q_ctrv) < 0.01);
if length(ctrv_fixed_idx) > 1
    plot(results.q_cv(ctrv_fixed_idx), results.combined_score(ctrv_fixed_idx), 'b-o', 'LineWidth', 2);
    xlabel('Q CV (CTRV fixed at optimal)');
    ylabel('Combined Score');
    title('CV Model Sensitivity');
    grid on;
else
    % If we don't have exact matches, show the general trend
    plot(results.q_cv(valid_idx), results.combined_score(valid_idx), 'b.', 'MarkerSize', 8);
    xlabel('Q CV');
    ylabel('Combined Score');
    title('CV Model vs Performance');
    grid on;
end

subplot(2, 3, 6);
% Show how performance changes with CTRV model Q values (fixing CV at optimal)
optimal_q_cv = results.q_cv(optimal_idx);
cv_fixed_idx = find(abs(results.q_cv - optimal_q_cv) < 0.01);
if length(cv_fixed_idx) > 1
    plot(results.q_ctrv(cv_fixed_idx), results.combined_score(cv_fixed_idx), 'r-o', 'LineWidth', 2);
    xlabel('Q CTRV (CV fixed at optimal)');
    ylabel('Combined Score');
    title('CTRV Model Sensitivity');
    grid on;
else
    % If we don't have exact matches, show the general trend
    plot(results.q_ctrv(valid_idx), results.combined_score(valid_idx), 'r.', 'MarkerSize', 8);
    xlabel('Q CTRV');
    ylabel('Combined Score');
    title('CTRV Model vs Performance');
    grid on;
end

sgtitle('2-Model IMM Filter: Performance Analysis');
saveas(gcf, 'figures/imm_2model_performance_analysis.png');
saveas(gcf, 'figures/imm_2model_performance_analysis.fig');

end

function filename = save2ModelIMMOptimizationResults(results)
% Save optimization results to file

if ~exist('figures', 'dir')
    mkdir('figures');
end

filename = fullfile('figures', 'imm_2model_q_optimization_results.mat');
save(filename, 'results');

% Also save a summary CSV
valid_idx = ~isnan(results.pos_rmse);
summary_table = table();
summary_table.q_cv = results.q_cv(valid_idx)';
summary_table.q_ctrv = results.q_ctrv(valid_idx)';
summary_table.pos_rmse = results.pos_rmse(valid_idx)';
summary_table.vel_rmse = results.vel_rmse(valid_idx)';
summary_table.sog_rmse = results.sog_rmse(valid_idx)';
summary_table.cog_rmse = results.cog_rmse(valid_idx)';
summary_table.combined_score = results.combined_score(valid_idx)';

% Sort by combined score
summary_table = sortrows(summary_table, 'combined_score');

csv_filename = fullfile('figures', 'imm_2model_q_optimization_summary.csv');
writetable(summary_table, csv_filename);

fprintf('\nTop 10 Q-value combinations for 2-model IMM (CV + CTRV):\n');
fprintf('Rank | Q_CV   | Q_CTRV | Pos_RMSE | Vel_RMSE | SOG_RMSE | COG_RMSE | Score\n');
fprintf('--------------------------------------------------------------------------------\n');
for i = 1:min(10, height(summary_table))
    fprintf('%4d | %6.3f | %6.3f | %8.2f | %8.2f | %8.2f | %8.2f | %5.3f\n', ...
            i, summary_table.q_cv(i), summary_table.q_ctrv(i), ...
            summary_table.pos_rmse(i), summary_table.vel_rmse(i), summary_table.sog_rmse(i), ...
            summary_table.cog_rmse(i), summary_table.combined_score(i));
end

end