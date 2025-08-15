function compareFilterPerformance(data, filter_estimates)
% COMPAREFILTERPERFORMANCE - Compare and visualize performance across filters with NEES/NIS
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
nees_summary = zeros(num_filters, 4); % [mean, median, std, consistency]
nis_summary = zeros(num_filters, 4);  % [mean, median, std, consistency]
filter_labels = cell(num_filters, 1);

fprintf('\n%-15s | %-12s | %-12s | %-12s | %-12s | %-10s | %-10s | %-8s | %-8s\n', ...
        'Filter', 'Pos RMSE', 'Vel RMSE', 'SOG RMSE', 'COG RMSE ', ...
        'NEES', 'NIS', 'NEES%', 'NIS%');
fprintf('---------------------------------------------------------------------------------------------------------------------------\n');

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
    
    % NEES/NIS analysis (if available)
    if isfield(est, 'nees') && isfield(est, 'nis')
        % NEES statistics
        valid_nees = ~isnan(est.nees);
        if any(valid_nees)
            nees_mean_val = mean(est.nees(valid_nees));
            nees_median_val = median(est.nees(valid_nees));
            nees_std_val = std(est.nees(valid_nees));
            
            % NEES consistency (2-DOF position-only system)
            alpha = 0.05;
            dof_state = 2;
            lower_bound = chi2inv(alpha/2, dof_state);
            upper_bound = chi2inv(1-alpha/2, dof_state);
            within_bounds = sum(est.nees(valid_nees) >= lower_bound & est.nees(valid_nees) <= upper_bound);
            nees_consistency = (within_bounds / sum(valid_nees)) * 100;
            
            nees_summary(i, :) = [nees_mean_val, nees_median_val, nees_std_val, nees_consistency];
        else
            nees_summary(i, :) = [NaN, NaN, NaN, NaN];
        end
        
        % NIS statistics
        valid_nis = ~isnan(est.nis) & est.nis ~= 0;
        if any(valid_nis)
            nis_mean_val = mean(est.nis(valid_nis));
            nis_median_val = median(est.nis(valid_nis));
            nis_std_val = std(est.nis(valid_nis));
            
            % NIS consistency (2-DOF system)
            dof_meas = 2;
            lower_bound_nis = chi2inv(alpha/2, dof_meas);
            upper_bound_nis = chi2inv(1-alpha/2, dof_meas);
            within_bounds_nis = sum(est.nis(valid_nis) >= lower_bound_nis & est.nis(valid_nis) <= upper_bound_nis);
            nis_consistency = (within_bounds_nis / sum(valid_nis)) * 100;
            
            nis_summary(i, :) = [nis_mean_val, nis_median_val, nis_std_val, nis_consistency];
        else
            nis_summary(i, :) = [NaN, NaN, NaN, NaN];
        end
        
        % Format for display
        if isnan(nees_summary(i, 1))
            nees_str = 'N/A';
            nees_cons_str = 'N/A';
        else
            nees_str = sprintf('%.3f', nees_summary(i, 1));
            nees_cons_str = sprintf('%.1f', nees_summary(i, 4));
        end
        
        if isnan(nis_summary(i, 1))
            nis_str = 'N/A';
            nis_cons_str = 'N/A';
        else
            nis_str = sprintf('%.3f', nis_summary(i, 1));
            nis_cons_str = sprintf('%.1f', nis_summary(i, 4));
        end
    else
        % No NEES/NIS data
        nees_summary(i, :) = [NaN, NaN, NaN, NaN];
        nis_summary(i, :) = [NaN, NaN, NaN, NaN];
        nees_str = 'N/A';
        nis_str = 'N/A';
        nees_cons_str = 'N/A';
        nis_cons_str = 'N/A';
    end
    
    fprintf('%-15s | %-12.2f | %-12.2f | %-12.2f | %-12.2f | %-10s | %-10s | %-8s | %-8s\n', ...
            filter_labels{i}, pos_rmse, vel_rmse, sog_rmse, cog_rmse, ...
            nees_str, nis_str, nees_cons_str, nis_cons_str);
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

% Create NEES/NIS comparison visualization
createNeesNisComparison(filter_estimates, filter_labels);

% Check if we have segment information for detailed analysis
if isfield(data, 'segment') && length(unique(data.segment)) > 1
    fprintf('\nCreating segment-wise analysis...\n');
    plotSegmentAnalysis(data, filter_estimates);
else
    fprintf('\nNo segment information found. Skipping segment analysis.\n');
end

fprintf('\nEnhanced visualizations saved to figures/ directory\n');
fprintf('- Enhanced filter comparison: figures/enhanced_filter_comparison.png\n');
fprintf('- NEES/NIS comparison: figures/nees_nis_comparison.png\n');
if isfield(data, 'segment') && length(unique(data.segment)) > 1
    fprintf('- Enhanced segment analysis: figures/enhanced_segment_analysis.png\n');
end

end

function createNeesNisComparison(filter_estimates, filter_labels)
% CREATENEESNISCOMPARISON - Create detailed NEES/NIS comparison plots

filter_names = fieldnames(filter_estimates);
num_filters = length(filter_names);

% Check which filters have NEES/NIS data
has_nees_nis = false(num_filters, 1);
for i = 1:num_filters
    est = filter_estimates.(filter_names{i});
    has_nees_nis(i) = isfield(est, 'nees') && isfield(est, 'nis');
end

if ~any(has_nees_nis)
    fprintf('No NEES/NIS data available for comparison.\n');
    return;
end

% Create figure
figure;

% Define colors for filters
colors = [
    0.0000, 0.4470, 0.7410;  % Blue
    0.8500, 0.3250, 0.0980;  % Orange  
    0.9290, 0.6940, 0.1250;  % Yellow
    0.4940, 0.1840, 0.5560;  % Purple
    0.4660, 0.6740, 0.1880;  % Green
    0.3010, 0.7450, 0.9330;  % Cyan
    0.6350, 0.0780, 0.1840;  % Dark Red
    0.2500, 0.2500, 0.2500;  % Dark Gray
];

if num_filters > size(colors, 1)
    additional_colors = hsv(num_filters - size(colors, 1));
    colors = [colors; additional_colors];
end

% Get data length from first filter with NEES/NIS data
first_valid_idx = find(has_nees_nis, 1);
if ~isempty(first_valid_idx)
    est = filter_estimates.(filter_names{first_valid_idx});
    n_samples = length(est.nees);
    t = 1:n_samples;
end

% Plot 1: NEES time series
subplot(2, 2, 1);
valid_count = 0;
for i = 1:num_filters
    if has_nees_nis(i)
        est = filter_estimates.(filter_names{i});
        valid_nees = ~isnan(est.nees);
        if any(valid_nees)
            plot(t(valid_nees), est.nees(valid_nees), '-', 'Color', colors(i, :), ...
                 'LineWidth', 1.5, 'DisplayName', filter_labels{i});
            hold on;
            valid_count = valid_count + 1;
        end
    end
end

if valid_count > 0
    % Chi-squared bounds for 2-DOF position-only system
    alpha = 0.05;
    dof_state = 2;
    lower_bound = chi2inv(alpha/2, dof_state);
    upper_bound = chi2inv(1-alpha/2, dof_state);
    
    yline(lower_bound, 'r--', 'Lower Bound (95%)', 'HandleVisibility', 'off');
    yline(upper_bound, 'r--', 'Upper Bound (95%)', 'HandleVisibility', 'off');
    yline(dof_state, 'g--', 'Expected Value', 'HandleVisibility', 'off');
    
    xlabel('Sample Number');
    ylabel('NEES');
    title('Normalized Estimation Error Squared (NEES)');
    legend('Location', 'best');
    grid on;
end

% Plot 2: NIS time series
subplot(2, 2, 2);
valid_count = 0;
for i = 1:num_filters
    if has_nees_nis(i)
        est = filter_estimates.(filter_names{i});
        valid_nis = ~isnan(est.nis) & est.nis ~= 0;
        if any(valid_nis)
            plot(t(valid_nis), est.nis(valid_nis), '-', 'Color', colors(i, :), ...
                 'LineWidth', 1.5, 'DisplayName', filter_labels{i});
            hold on;
            valid_count = valid_count + 1;
        end
    end
end

if valid_count > 0
    % Chi-squared bounds for 2-DOF system
    dof_meas = 2;
    lower_bound_nis = chi2inv(alpha/2, dof_meas);
    upper_bound_nis = chi2inv(1-alpha/2, dof_meas);
    
    yline(lower_bound_nis, 'r--', 'Lower Bound (95%)', 'HandleVisibility', 'off');
    yline(upper_bound_nis, 'r--', 'Upper Bound (95%)', 'HandleVisibility', 'off');
    yline(dof_meas, 'g--', 'Expected Value', 'HandleVisibility', 'off');
    
    xlabel('Sample Number');
    ylabel('NIS');
    title('Normalized Innovation Squared (NIS)');
    legend('Location', 'best');
    grid on;
end

% Plot 3: NEES mean comparison
subplot(2, 2, 3);
nees_means = [];
nis_means = [];
valid_filter_labels = {};

for i = 1:num_filters
    if has_nees_nis(i)
        est = filter_estimates.(filter_names{i});
        valid_nees = ~isnan(est.nees);
        valid_nis = ~isnan(est.nis) & est.nis ~= 0;
        
        if any(valid_nees)
            nees_means(end+1) = mean(est.nees(valid_nees));
            if any(valid_nis)
                nis_means(end+1) = mean(est.nis(valid_nis));
            else
                nis_means(end+1) = NaN;
            end
            valid_filter_labels{end+1} = filter_labels{i};
        end
    end
end

if ~isempty(nees_means)
    bar_data = [nees_means', nis_means'];
    h = bar(bar_data);
    
    % Only set FaceColor if we have valid bar handles
    if ~isempty(h) && length(h) >= 2
        if isvalid(h(1))
            h(1).FaceColor = [0.2, 0.4, 0.8];  % Blue for NEES
        end
        if isvalid(h(2))
            h(2).FaceColor = [0.8, 0.4, 0.2];  % Orange for NIS
        end
    end
    
    % Add expected value lines
    yline(4, 'g--', 'Expected NEES', 'LineWidth', 2);
    yline(2, 'm--', 'Expected NIS', 'LineWidth', 2);
    
    xlabel('Filter Type');
    ylabel('Mean Value');
    title('Mean NEES and NIS Comparison');
    set(gca, 'XTick', 1:length(valid_filter_labels), ...
             'XTickLabel', valid_filter_labels, 'XTickLabelRotation', 45);
    legend({'NEES', 'NIS'}, 'Location', 'best');
    grid on;
end

% Plot 4: Consistency comparison
subplot(2, 2, 4);
consistency_nees = [];
consistency_nis = [];

for i = 1:num_filters
    if has_nees_nis(i)
        est = filter_estimates.(filter_names{i});
        
        % NEES consistency
        valid_nees = ~isnan(est.nees);
        if any(valid_nees)
            lower_bound = chi2inv(alpha/2, dof_state);
            upper_bound = chi2inv(1-alpha/2, dof_state);
            within_bounds = sum(est.nees(valid_nees) >= lower_bound & est.nees(valid_nees) <= upper_bound);
            consistency_nees(end+1) = (within_bounds / sum(valid_nees)) * 100;
            
            % NIS consistency
            valid_nis = ~isnan(est.nis) & est.nis ~= 0;
            if any(valid_nis)
                lower_bound_nis = chi2inv(alpha/2, dof_meas);
                upper_bound_nis = chi2inv(1-alpha/2, dof_meas);
                within_bounds_nis = sum(est.nis(valid_nis) >= lower_bound_nis & est.nis(valid_nis) <= upper_bound_nis);
                consistency_nis(end+1) = (within_bounds_nis / sum(valid_nis)) * 100;
            else
                consistency_nis(end+1) = NaN;
            end
        end
    end
end

if ~isempty(consistency_nees)
    bar_data = [consistency_nees', consistency_nis'];
    h = bar(bar_data);
    
    % Only set FaceColor if we have valid bar handles
    if ~isempty(h) && length(h) >= 2
        if isvalid(h(1))
            h(1).FaceColor = [0.2, 0.4, 0.8];  % Blue for NEES
        end
        if isvalid(h(2))
            h(2).FaceColor = [0.8, 0.4, 0.2];  % Orange for NIS
        end
    end
    
    % Add 95% target line
    yline(95, 'r--', '95% Target', 'LineWidth', 2);
    
    xlabel('Filter Type');
    ylabel('Consistency (%)');
    title('Filter Consistency (% within bounds)');
    set(gca, 'XTick', 1:length(valid_filter_labels), ...
             'XTickLabel', valid_filter_labels, 'XTickLabelRotation', 45);
    legend({'NEES', 'NIS'}, 'Location', 'best');
    grid on;
    ylim([0, 100]);
end

% Add overall title
sgtitle('NEES and NIS Filter Comparison', 'FontSize', 16, 'FontWeight', 'bold');

% Create output directory if needed
if ~exist('figures', 'dir')
    mkdir('figures');
end

% Save the figure
saveas(gcf, 'figures/nees_nis_comparison.png');
savefig('figures/nees_nis_comparison.fig');

fprintf('NEES/NIS comparison plots saved to figures/nees_nis_comparison.png/.fig\n');

end