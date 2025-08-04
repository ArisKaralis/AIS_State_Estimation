function compareFilterPerformance(data, filter_estimates)
% COMPAREFILTERPERFORMANCE - Compare and visualize performance across filters
% 
% Inputs:
%   data - Table with ground truth and measurement data
%   filter_estimates - Struct where each field is a filter name containing estimates
%                     Example: filter_estimates.KF.x_est, filter_estimates.EKF_CV.x_est, etc.

% Get filter names from the struct
filter_names = fieldnames(filter_estimates);
num_filters = length(filter_names);

% Validate that all filters have required fields
required_fields = {'x_est', 'y_est', 'vx_est', 'vy_est', 'sog_est', 'cog_est'};
for i = 1:num_filters
    for j = 1:length(required_fields)
        if ~isfield(filter_estimates.(filter_names{i}), required_fields{j})
            error('Filter %s is missing required field: %s', filter_names{i}, required_fields{j});
        end
    end
end

fprintf('\n===== Enhanced Filter Performance Comparison =====\n');
fprintf('Analyzing %d filters across %d data points...\n', num_filters, height(data));

% Create enhanced comparison plot
plotFilterComparison(data, filter_estimates);

% Calculate comprehensive performance metrics
rmse_summary = zeros(num_filters, 4);
filter_labels = cell(num_filters, 1);

fprintf('\n%-15s | %-12s | %-12s | %-12s | %-12s\n', ...
        'Filter', 'Pos RMSE (m)', 'Vel RMSE (m/s)', 'SOG RMSE (m/s)', 'COG RMSE (deg)');
fprintf('--------------------------------------------------------------------------------\n');

for i = 1:num_filters
    filter_name = filter_names{i};
    est = filter_estimates.(filter_name);
    filter_labels{i} = strrep(filter_name, '_', '-');
    
    % Calculate RMSEs
    pos_rmse = sqrt(mean((est.x_est - data.x_true).^2 + (est.y_est - data.y_true).^2));
    vel_rmse = sqrt(mean((est.vx_est - data.vx_true).^2 + (est.vy_est - data.vy_true).^2));
    sog_rmse = sqrt(mean((est.sog_est - data.sog_true).^2));
    
    % Handle COG wrapping for RMSE calculation
    cog_diff = angdiff(deg2rad(est.cog_est), deg2rad(data.cog_true)) * 180/pi;
    cog_rmse = sqrt(mean(cog_diff.^2));
    
    rmse_summary(i, :) = [pos_rmse, vel_rmse, sog_rmse, cog_rmse];
    
    fprintf('%-15s | %-12.2f | %-12.2f | %-12.2f | %-12.2f\n', ...
            filter_labels{i}, pos_rmse, vel_rmse, sog_rmse, cog_rmse);
end

% Find best performers
[~, best_pos_idx] = min(rmse_summary(:, 1));
[~, best_vel_idx] = min(rmse_summary(:, 2));
[~, best_sog_idx] = min(rmse_summary(:, 3));
[~, best_cog_idx] = min(rmse_summary(:, 4));

% Calculate overall ranking
normalized_rmse = rmse_summary ./ max(rmse_summary, [], 1);
overall_scores = mean(normalized_rmse, 2);
[~, best_overall_idx] = min(overall_scores);

fprintf('\n===== Best Performers =====\n');
fprintf('Position:   %s (%.2f m)\n', filter_labels{best_pos_idx}, rmse_summary(best_pos_idx, 1));
fprintf('Velocity:   %s (%.2f m/s)\n', filter_labels{best_vel_idx}, rmse_summary(best_vel_idx, 2));
fprintf('SOG:        %s (%.2f m/s)\n', filter_labels{best_sog_idx}, rmse_summary(best_sog_idx, 3));
fprintf('COG:        %s (%.2f deg)\n', filter_labels{best_cog_idx}, rmse_summary(best_cog_idx, 4));
fprintf('Overall:    %s (normalized score: %.3f)\n', filter_labels{best_overall_idx}, overall_scores(best_overall_idx));

% Performance ranking
fprintf('\n===== Overall Performance Ranking =====\n');
[sorted_scores, rank_idx] = sort(overall_scores);
for i = 1:num_filters
    idx = rank_idx(i);
    fprintf('%d. %s (score: %.3f)\n', i, filter_labels{idx}, sorted_scores(i));
end

% Check if we have segment information for detailed analysis
if isfield(data, 'segment') && length(unique(data.segment)) > 1
    fprintf('\nCreating segment-wise analysis...\n');
    plotSegmentAnalysis(data, filter_estimates);
else
    fprintf('\nNo segment information found. Skipping segment analysis.\n');
end

fprintf('\nEnhanced visualizations saved to figures/ directory\n');
fprintf('- Enhanced filter comparison: figures/enhanced_filter_comparison.png\n');
if isfield(data, 'segment') && length(unique(data.segment)) > 1
    fprintf('- Enhanced segment analysis: figures/enhanced_segment_analysis.png\n');
end

end