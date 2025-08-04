function runEnhancedVisualization()
% RUNENHANCEDVISUALIZATION - Create high-quality visualizations after running filters
%
% This function assumes you have already run runAISFilters() and have the
% data and filter_estimates variables in your workspace


% Check if data and filter_estimates exist in workspace
if ~evalin('caller', 'exist(''data'', ''var'')')
    error('Variable ''data'' not found. Please run runAISFilters() first.');
end

if ~evalin('caller', 'exist(''filter_estimates'', ''var'')')
    error('Variable ''filter_estimates'' not found. Please run runAISFilters() first.');
end

% Get variables from calling workspace
data = evalin('caller', 'data');
filter_estimates = evalin('caller', 'filter_estimates');


% 1. Enhanced overall comparison
plotFilterComparison(data, filter_estimates);

% 2. Enhanced segment analysis (if segments exist)
if isfield(data, 'segment') && length(unique(data.segment)) > 1
    plotSegmentAnalysis(data, filter_estimates);
end

% 3. Create combined plot for the best 3 performers
filter_names = fieldnames(filter_estimates);

% Calculate which filters to highlight (top 3 performers)
rmse_summary = zeros(length(filter_names), 4);
for i = 1:length(filter_names)
    filter_name = filter_names{i};
    est = filter_estimates.(filter_name);
    
    pos_rmse = sqrt(mean((est.x_est - data.x_true).^2 + (est.y_est - data.y_true).^2));
    vel_rmse = sqrt(mean((est.vx_est - data.vx_true).^2 + (est.vy_est - data.vy_true).^2));
    sog_rmse = sqrt(mean((est.sog_est - data.sog_true).^2));
    cog_diff = angdiff(deg2rad(est.cog_est), deg2rad(data.cog_true)) * 180/pi;
    cog_rmse = sqrt(mean(cog_diff.^2));
    
    rmse_summary(i, :) = [pos_rmse, vel_rmse, sog_rmse, cog_rmse];
end

% Calculate overall scores and find top 3
normalized_rmse = rmse_summary ./ max(rmse_summary, [], 1);
overall_scores = mean(normalized_rmse, 2);
[~, rank_idx] = sort(overall_scores);

% Create combined plot for top 3 filters
top_filters = min(3, length(filter_names));
plotBest3FiltersCombined(data, filter_estimates, filter_names, rank_idx, top_filters);

end

function plotBest3FiltersCombined(data, filter_estimates, filter_names, rank_idx, top_filters)
% PLOTBEST3FILTERSCOMBINED - Create a combined visualization of the best 3 filters
%
% This function creates a single figure with all best 3 filters plotted together

figure;

% Define colors for the 3 best filters
filter_colors = [
    0.0000, 0.4470, 0.7410;  % Blue
    0.6350, 0.0780, 0.1840;  % Dark Red 
    0.8000, 0.4000, 0.8000;  % Light Purple
];

% Get filter information for top 3
top_filter_names = cell(top_filters, 1);
top_filter_titles = cell(top_filters, 1);
top_filter_estimates = cell(top_filters, 1);

for i = 1:top_filters
    filter_idx = rank_idx(i);
    top_filter_names{i} = filter_names{filter_idx};
    top_filter_titles{i} = strrep(filter_names{filter_idx}, '_', '-');
    top_filter_estimates{i} = filter_estimates.(filter_names{filter_idx});
end

% Sample indices
t = 1:height(data);

% Plot 1: Trajectory comparison
subplot(2, 3, 1);
plot(data.x, data.y, 'k.', 'MarkerSize', 8, 'DisplayName', 'Measurements');
hold on;
plot(data.x_true, data.y_true, 'k-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');

est = top_filter_estimates{1};
plot(est.x_est, est.y_est, '-', 'Color', filter_colors(1,:), ...
         'LineWidth', 2, 'DisplayName', top_filter_titles{1});

for i = 2:top_filters
    est = top_filter_estimates{i};
    plot(est.x_est, est.y_est, '-', 'Color', filter_colors(i,:), ...
         'LineWidth', 1, 'DisplayName', top_filter_titles{i});
end

xlabel('X (meters)');
ylabel('Y (meters)');
title('Trajectory Comparison - Best 3 Filters');
legend('Location', 'best');
grid on;
axis equal;

% Plot 2: Speed Over Ground
subplot(2, 3, 2);
plot(t, data.SOG, 'k.', 'MarkerSize', 4, 'DisplayName', 'Measurements');
hold on;
plot(t, data.sog_true, 'k-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');

est = top_filter_estimates{1};
plot(est.sog_est, '-', 'Color', filter_colors(1,:), ...
         'LineWidth', 2, 'DisplayName', top_filter_titles{1});

for i = 2:top_filters
    est = top_filter_estimates{i};
    plot(est.sog_est, '-', 'Color', filter_colors(i,:), ...
         'LineWidth', 1, 'DisplayName', top_filter_titles{i});
end

% Add segment boundaries
if isfield(data, 'segment')
    segmentChanges = find(diff(data.segment) ~= 0);
    for i = 1:length(segmentChanges)
        xline(segmentChanges(i)+1, 'k--', 'HandleVisibility', 'off');
    end
end

xlabel('Sample number');
ylabel('Speed (m/s)');
title('Speed Over Ground Comparison');
legend('Location', 'best');
grid on;

% Plot 3: Course Over Ground (with unwrapping fix)
subplot(2, 3, 3);

% Convert all course data to radians first
cog_meas_rad = deg2rad(data.COG);
cog_true_rad = deg2rad(data.cog_true);

% Unwrap all angles to remove discontinuities
cog_meas_unwrapped = rad2deg(unwrap(cog_meas_rad));
cog_true_unwrapped = rad2deg(unwrap(cog_true_rad));

plot(t, cog_meas_unwrapped, 'k.', 'MarkerSize', 4, 'DisplayName', 'Measurements');
hold on;
plot(t, cog_true_unwrapped, 'k-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');

est = top_filter_estimates{1};
cog_est_rad = deg2rad(est.cog_est);
cog_est_unwrapped = rad2deg(unwrap(cog_est_rad));
plot(t, cog_est_unwrapped, '-', 'Color', filter_colors(1,:), ...
     'LineWidth', 2, 'DisplayName', top_filter_titles{1});

for i = 2:top_filters
    est = top_filter_estimates{i};
    cog_est_rad = deg2rad(est.cog_est);
    cog_est_unwrapped = rad2deg(unwrap(cog_est_rad));
    plot(t, cog_est_unwrapped, '-', 'Color', filter_colors(i,:), ...
         'LineWidth', 1, 'DisplayName', top_filter_titles{i});
end

% Add segment boundaries
if isfield(data, 'segment')
    for i = 1:length(segmentChanges)
        xline(segmentChanges(i)+1, 'k--', 'HandleVisibility', 'off');
    end
end

xlabel('Sample number');
ylabel('Course (degrees)');
title('Course Over Ground Comparison');
legend('Location', 'best');
grid on;

% Plot 4: Position Error comparison
subplot(2, 3, 4);
est = top_filter_estimates{1};
posError = sqrt((est.x_est - data.x_true).^2 + (est.y_est - data.y_true).^2);
plot(t, posError, '-', 'Color', filter_colors(1,:), ...
     'LineWidth', 2, 'DisplayName', top_filter_titles{1});
hold on;
for i = 2:top_filters
    est = top_filter_estimates{i};
    posError = sqrt((est.x_est - data.x_true).^2 + (est.y_est - data.y_true).^2);
    plot(t, posError, '-', 'Color', filter_colors(i,:), ...
         'LineWidth', 1, 'DisplayName', top_filter_titles{i});
    hold on;
end

% Add segment boundaries
if isfield(data, 'segment')
    for i = 1:length(segmentChanges)
        xline(segmentChanges(i)+1, 'k--', 'HandleVisibility', 'off');
    end
end

xlabel('Sample number');
ylabel('Error (m)');
title('Position Error Comparison');
legend('Location', 'best');
grid on;

% Plot 5: Velocity comparison
subplot(2, 3, 5);
plot(t, data.vx_true, 'k--', 'LineWidth', 1, 'DisplayName', 'True V_x');
hold on;
plot(t, data.vy_true, 'k:', 'LineWidth', 1, 'DisplayName', 'True V_y');

est = top_filter_estimates{1};
plot(t, est.vx_est, '-', 'Color', filter_colors(1,:), ...
    'LineWidth', 2, 'DisplayName', [top_filter_titles{1} ' V_x']);
plot(t, est.vy_est, '--', 'Color', filter_colors(1,:), ...
    'LineWidth', 2, 'DisplayName', [top_filter_titles{1} ' V_y']);

for i = 2:top_filters
    est = top_filter_estimates{i};
    plot(t, est.vx_est, '-', 'Color', filter_colors(i,:), ...
         'LineWidth', 1.2, 'DisplayName', [top_filter_titles{i} ' V_x']);
    plot(t, est.vy_est, '--', 'Color', filter_colors(i,:), ...
         'LineWidth', 1.2, 'DisplayName', [top_filter_titles{i} ' V_y']);
end

% Add segment boundaries
if isfield(data, 'segment')
    for i = 1:length(segmentChanges)
        xline(segmentChanges(i)+1, 'k--', 'HandleVisibility', 'off');
    end
end

xlabel('Sample number');
ylabel('Velocity (m/s)');
title('Velocity Components Comparison');
legend('Location', 'best');
grid on;

% Plot 6: Performance summary table
subplot(2, 3, 6);
axis off;

% Create performance summary
summary_text = {'Best 3 Filters Performance Summary:', ''};
for i = 1:top_filters
    filter_idx = rank_idx(i);
    filter_name = filter_names{filter_idx};
    est = filter_estimates.(filter_name);
    
    pos_rmse = sqrt(mean((est.x_est - data.x_true).^2 + (est.y_est - data.y_true).^2));
    vel_rmse = sqrt(mean((est.vx_est - data.vx_true).^2 + (est.vy_est - data.vy_true).^2));
    sog_rmse = sqrt(mean((est.sog_est - data.sog_true).^2));
    cog_diff = angdiff(deg2rad(est.cog_est), deg2rad(data.cog_true)) * 180/pi;
    cog_rmse = sqrt(mean(cog_diff.^2));
    
    summary_text{end+1} = sprintf('%d. %s:', i, strrep(filter_name, '_', '-'));
    summary_text{end+1} = sprintf('   Position RMSE: %.2f m', pos_rmse);
    summary_text{end+1} = sprintf('   Velocity RMSE: %.2f m/s', vel_rmse);
    summary_text{end+1} = sprintf('   SOG RMSE: %.2f m/s', sog_rmse);
    summary_text{end+1} = sprintf('   COG RMSE: %.2f deg', cog_rmse);
    summary_text{end+1} = '';
end

text(0.05, 0.95, summary_text, 'Units', 'normalized', ...
     'VerticalAlignment', 'top', 'FontSize', 10, 'FontName', 'monospace');

% Add overall title
sgtitle('Best 3 Filters Comparison', 'FontSize', 16, 'FontWeight', 'bold');

% Create output directory if needed
if ~exist('figures', 'dir')
    mkdir('figures');
end

% Save the combined figure
saveas(gcf, fullfile('figures', 'best_3_filters_combined.png'));
savefig(fullfile('figures', 'best_3_filters_combined.fig'));

fprintf('Combined plot for best 3 filters saved to figures/best_3_filters_combined.png/.fig\n');

end