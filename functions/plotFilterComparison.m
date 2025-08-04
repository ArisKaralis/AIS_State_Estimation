function plotFilterComparison(data, filter_estimates)
% PLOTFILTERCOMPARISON - Enhanced visualization for filter comparison
% 
% Creates high-quality plots with improved color schemes, line styles,
% and proper handling of overlapping data
%
% Inputs:
%   data - Table with ground truth and measurement data
%   filter_estimates - Struct where each field is a filter name containing estimates

% Get filter names from the struct
filter_names = fieldnames(filter_estimates);
num_filters = length(filter_names);

% Enhanced color palette - distinct and colorblind-friendly
colors = [
    0.0000, 0.4470, 0.7410;  % Blue
    0.8500, 0.3250, 0.0980;  % Orange  
    0.9290, 0.6940, 0.1250;  % Yellow
    0.4940, 0.1840, 0.5560;  % Purple
    0.4660, 0.6740, 0.1880;  % Green
    0.3010, 0.7450, 0.9330;  % Cyan
    0.6350, 0.0780, 0.1840;  % Dark Red
];

% Line styles for better distinction
line_styles = {'-', '--', '-.', ':', '-', '--', '-.'};
line_widths = [2.0, 2.0, 2.0, 2.5, 2.0, 2.0, 2.0];  % Thicker lines for better visibility

% Create figure with better layout
figure;

% Sample indices
t = 1:height(data);

% 1. Trajectory comparison with improved visualization
subplot(2, 3, 1);
% Plot measurements with smaller, semi-transparent markers
scatter(data.x, data.y, 15, [0.7 0.7 0.7], 'filled', 'MarkerFaceAlpha', 0.4, 'DisplayName', 'Measurements');
hold on;

% Plot ground truth with thick black line
plot(data.x_true, data.y_true, 'k-', 'LineWidth', 3, 'DisplayName', 'Ground Truth');

% Plot each filter with distinct colors and styles
for i = 1:num_filters
    filter_name = filter_names{i};
    est = filter_estimates.(filter_name);
    color_idx = mod(i-1, size(colors, 1)) + 1;
    style_idx = mod(i-1, length(line_styles)) + 1;
    
    plot(est.x_est, est.y_est, line_styles{style_idx}, ...
         'Color', colors(color_idx, :), ...
         'LineWidth', line_widths(style_idx), ...
         'DisplayName', strrep(filter_name, '_', '-'));
end

xlabel('X Position (m)', 'FontSize', 12);
ylabel('Y Position (m)', 'FontSize', 12);
title('Trajectory Comparison', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
grid on;
axis equal;
set(gca, 'FontSize', 10);

% 2. Position error comparison with log scale for better visibility
subplot(2, 3, 2);
pos_errors = cell(num_filters, 1);

for i = 1:num_filters
    filter_name = filter_names{i};
    est = filter_estimates.(filter_name);
    pos_errors{i} = sqrt((est.x_est - data.x_true).^2 + (est.y_est - data.y_true).^2);
    
    color_idx = mod(i-1, size(colors, 1)) + 1;
    style_idx = mod(i-1, length(line_styles)) + 1;
    
    semilogy(t, pos_errors{i}, line_styles{style_idx}, ...
             'Color', colors(color_idx, :), ...
             'LineWidth', line_widths(style_idx), ...
             'DisplayName', strrep(filter_name, '_', '-'));
    hold on;
end

% Add segment boundaries if available
if isfield(data, 'segment')
    segmentChanges = find(diff(data.segment) ~= 0);
    for i = 1:length(segmentChanges)
        xline(t(segmentChanges(i)+1), 'k--', 'Alpha', 0.5, 'HandleVisibility', 'off');
    end
end

xlabel('Sample Number', 'FontSize', 12);
ylabel('Position Error (m)', 'FontSize', 12);
title('Position Error (Log Scale)', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
grid on;
set(gca, 'FontSize', 10);

% 3. Speed comparison with improved readability
subplot(2, 3, 3);
plot(t, data.SOG, '.', 'Color', [0.7 0.7 0.7], 'MarkerSize', 8, 'DisplayName', 'Measurements');
hold on;
plot(t, data.sog_true, 'k-', 'LineWidth', 3, 'DisplayName', 'Ground Truth');

for i = 1:num_filters
    filter_name = filter_names{i};
    est = filter_estimates.(filter_name);
    color_idx = mod(i-1, size(colors, 1)) + 1;
    style_idx = mod(i-1, length(line_styles)) + 1;
    
    plot(t, est.sog_est, line_styles{style_idx}, ...
         'Color', colors(color_idx, :), ...
         'LineWidth', line_widths(style_idx), ...
         'DisplayName', strrep(filter_name, '_', '-'));
end

% Add segment boundaries
if isfield(data, 'segment')
    for i = 1:length(segmentChanges)
        xline(t(segmentChanges(i)+1), 'k--', 'Alpha', 0.5, 'HandleVisibility', 'off');
    end
end

xlabel('Sample Number', 'FontSize', 12);
ylabel('Speed (m/s)', 'FontSize', 12);
title('Speed Over Ground', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
grid on;
set(gca, 'FontSize', 10);

% 4. Course comparison with proper COG wrapping fix
subplot(2, 3, 4);

% --- Fix for COG wrapping using MATLAB's unwrap function ---
% Convert all course data to radians first
cog_meas_rad = deg2rad(data.COG);
cog_true_rad = deg2rad(data.cog_true);

% Unwrap angles to remove discontinuities
cog_meas_unwrapped = rad2deg(unwrap(cog_meas_rad));
cog_true_unwrapped = rad2deg(unwrap(cog_true_rad));

% Plot measurements and ground truth
plot(t, cog_meas_unwrapped, '.', 'Color', [0.7 0.7 0.7], 'MarkerSize', 8, 'DisplayName', 'Measurements');
hold on;
plot(t, cog_true_unwrapped, 'k-', 'LineWidth', 3, 'DisplayName', 'Ground Truth');

% Plot each filter with unwrapped course data
for i = 1:num_filters
    filter_name = filter_names{i};
    est = filter_estimates.(filter_name);
    color_idx = mod(i-1, size(colors, 1)) + 1;
    style_idx = mod(i-1, length(line_styles)) + 1;
    
    % Unwrap the filter estimates
    cog_est_rad = deg2rad(est.cog_est);
    cog_est_unwrapped = rad2deg(unwrap(cog_est_rad));
    
    plot(t, cog_est_unwrapped, line_styles{style_idx}, ...
         'Color', colors(color_idx, :), ...
         'LineWidth', line_widths(style_idx), ...
         'DisplayName', strrep(filter_name, '_', '-'));
end
% --- End of fix ---

% Add segment boundaries
if isfield(data, 'segment')
    for i = 1:length(segmentChanges)
        xline(t(segmentChanges(i)+1), 'k--', 'Alpha', 0.5, 'HandleVisibility', 'off');
    end
end

xlabel('Sample Number', 'FontSize', 12);
ylabel('Course (degrees)', 'FontSize', 12);
title('Course Over Ground', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
grid on;
set(gca, 'FontSize', 10);

% 5. Performance summary bar chart
subplot(2, 3, 5);
rmse_data = zeros(num_filters, 4);

for i = 1:num_filters
    filter_name = filter_names{i};
    est = filter_estimates.(filter_name);
    
    % Position RMSE
    rmse_data(i, 1) = sqrt(mean((est.x_est - data.x_true).^2 + (est.y_est - data.y_true).^2));
    
    % Velocity RMSE
    rmse_data(i, 2) = sqrt(mean((est.vx_est - data.vx_true).^2 + (est.vy_est - data.vy_true).^2));
    
    % SOG RMSE
    rmse_data(i, 3) = sqrt(mean((est.sog_est - data.sog_true).^2));
    
    % Handle COG wrapping for RMSE calculation
    cog_diff = angdiff(deg2rad(est.cog_est), deg2rad(data.cog_true)) * 180/pi;
    rmse_data(i, 4) = sqrt(mean(cog_diff.^2));
end

b = bar(rmse_data);
b(1).FaceColor = [0.2 0.6 1.0];  % Position - Light Blue
b(2).FaceColor = [0.9 0.6 0.1];  % Velocity - Orange
b(3).FaceColor = [0.1 0.8 0.2];  % SOG - Green
b(4).FaceColor = [0.8 0.2 0.6];  % COG - Magenta

xlabel('Filter', 'FontSize', 12);
ylabel('RMSE', 'FontSize', 12);
title('Performance Summary', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'XTickLabel', strrep(filter_names, '_', '-'), 'XTickLabelRotation', 45, 'FontSize', 10);
legend({'Position (m)', 'Velocity (m/s)', 'SOG (m/s)', 'COG (deg)'}, 'Location', 'best', 'FontSize', 10);
grid on;

% 6. Overall ranking based on normalized performance
subplot(2, 3, 6);
normalized_rmse = rmse_data ./ max(rmse_data, [], 1);
overall_scores = mean(normalized_rmse, 2);
[sorted_scores, sort_idx] = sort(overall_scores);

barh(sorted_scores);
set(gca, 'YTick', 1:num_filters, 'YTickLabel', strrep(filter_names(sort_idx), '_', '-'), 'FontSize', 10);
xlabel('Filter (Best to Worst)', 'FontSize', 12);
title('Overall Ranking', 'FontSize', 14, 'FontWeight', 'bold');
grid on;

% Add score labels
for i = 1:num_filters
    text(sorted_scores(i) + 0.01, i, sprintf('%.3f', sorted_scores(i)), ...
         'VerticalAlignment', 'middle', 'FontSize', 10);
end

% Overall title
sgtitle('Filter Performance Comparison', 'FontSize', 16, 'FontWeight', 'bold');

% Create output directory if needed
if ~exist('figures', 'dir')
    mkdir('figures');
end

% Save the figure
saveas(gcf, 'figures/filter_comparison.png');
saveas(gcf, 'figures/filter_comparison.fig');

fprintf('Enhanced filter comparison saved to figures/filter_comparison.png/.fig\n');

end