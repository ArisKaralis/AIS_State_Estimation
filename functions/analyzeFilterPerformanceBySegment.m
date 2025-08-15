function analyzeFilterPerformanceBySegment()
% ANALYZEFILTERPERFORMANCEBYSEGMENT - Detailed analysis of filter performance
% across different motion patterns

% Load the dataset
data = readtable('data/monte_carlo_sample.csv');

% Test parameters
pos_std = 3.0;

% Initialize EKF
filter = ekfCV_directScaling(pos_std);

% Initialize storage
nees_values = zeros(height(data), 1);
innovation_norms = zeros(height(data), 1);
covariance_traces = zeros(height(data), 1);
position_errors = zeros(height(data), 1);

fprintf('Analyzing filter performance across motion segments...\n');

% Run filter and collect detailed metrics
for k = 1:height(data)
    % Extract measurements and ground truth
    z = [data.x(k); data.y(k)];
    true_pos = [data.x_true(k); data.y_true(k)];
    true_vel = [data.vx_true(k); data.vy_true(k)];
    
    % Predict step
    predict(filter);
    
    % Calculate innovation before correction
    predicted_measurement = filter.State([1, 3]);  % [x, y]
    innovation = z - predicted_measurement;
    innovation_norms(k) = norm(innovation);
    
    % Correct step
    correct(filter, z);
    
    % Calculate metrics
    estimated_pos = filter.State([1, 3]);
    position_error = estimated_pos - true_pos;
    position_errors(k) = norm(position_error);
    
    % NEES calculation
    P_pos = filter.StateCovariance([1, 3], [1, 3]);
    nees_values(k) = position_error' * (P_pos \ position_error);
    
    % Covariance trace (uncertainty measure)
    covariance_traces(k) = trace(P_pos);
    
    if mod(k, 50) == 0
        fprintf('Processed %d/%d timesteps\n', k, height(data));
    end
end

% Create comprehensive analysis figure
figure;

% Get segment information
segment_data = data.segment;
segment_names = data.segment_name;
timestamps = data.timestamp;

% Define colors for segments
segment_colors = [
    0.2, 0.4, 0.8;    % Blue - Const Vel 1
    0.8, 0.4, 0.2;    % Orange - Accel
    0.8, 0.2, 0.2;    % Red - 45° Turn
    0.2, 0.8, 0.2;    % Green - Const Vel 2
    0.8, 0.2, 0.8;    % Magenta - 90° Turn
    0.6, 0.2, 0.8;    % Purple - Decel
    0.5, 0.5, 0.5     % Gray - Const Vel 3
];

% Plot 1: NEES over time
subplot(3, 2, 1);
plotWithSegmentBackground(timestamps, nees_values, segment_data, segment_colors);
yline(2.0, 'g--', 'LineWidth', 2);
yline(2.215, 'r--', 'LineWidth', 1);
yline(1.796, 'r--', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('NEES Value');
title('NEES Over Time');
grid on;

% Plot 2: Position error over time
subplot(3, 2, 2);
plotWithSegmentBackground(timestamps, position_errors, segment_data, segment_colors);
xlabel('Time (s)');
ylabel('Position Error (m)');
title('Position Error Over Time');
grid on;

% Plot 3: Innovation norm over time
subplot(3, 2, 3);
plotWithSegmentBackground(timestamps, innovation_norms, segment_data, segment_colors);
xlabel('Time (s)');
ylabel('Innovation Norm');
title('Innovation Magnitude Over Time');
grid on;

% Plot 4: Covariance trace over time
subplot(3, 2, 4);
plotWithSegmentBackground(timestamps, covariance_traces, segment_data, segment_colors);
xlabel('Time (s)');
ylabel('Covariance Trace');
title('Filter Uncertainty Over Time');
grid on;

% Plot 5: Distribution of NEES by segment
subplot(3, 2, 5);
unique_segments = unique(segment_data);
segment_nees_data = cell(length(unique_segments), 1);
segment_labels = cell(length(unique_segments), 1);

for i = 1:length(unique_segments)
    seg_idx = segment_data == unique_segments(i);
    segment_nees_data{i} = nees_values(seg_idx);
    segment_labels{i} = char(unique(segment_names(seg_idx)));
end

boxplot([segment_nees_data{:}], 'Labels', segment_labels);
yline(2.0, 'g--', 'LineWidth', 2);
yline(2.215, 'r--', 'LineWidth', 1);
yline(1.796, 'r--', 'LineWidth', 1);
xlabel('Motion Segment');
ylabel('NEES Value');
title('NEES Distribution by Segment');
xtickangle(45);
grid on;

% Plot 6: Correlation between NEES and metrics
subplot(3, 2, 6);
scatter(position_errors, nees_values, 20, segment_data, 'filled');
colormap(segment_colors);
colorbar('Ticks', unique_segments, 'TickLabels', segment_labels);
xlabel('Position Error (m)');
ylabel('NEES Value');
title('NEES vs Position Error (colored by segment)');
grid on;

% Statistical analysis by segment
fprintf('\n=== DETAILED SEGMENT ANALYSIS ===\n');
fprintf('%-15s %8s %8s %8s %8s %8s %8s\n', 'Segment', 'NEES', 'PosErr', 'Innov', 'CovTr', 'MaxNEES', 'Count');
fprintf('%-15s %8s %8s %8s %8s %8s %8s\n', '-------', '----', '------', '-----', '-----', '-------', '-----');

problematic_segments = [];
for i = 1:length(unique_segments)
    seg_idx = segment_data == unique_segments(i);
    seg_name = char(unique(segment_names(seg_idx)));
    
    mean_nees = mean(nees_values(seg_idx));
    mean_pos_err = mean(position_errors(seg_idx));
    mean_innov = mean(innovation_norms(seg_idx));
    mean_cov_trace = mean(covariance_traces(seg_idx));
    max_nees = max(nees_values(seg_idx));
    count = sum(seg_idx);
    
    fprintf('%-15s %8.3f %8.3f %8.3f %8.3f %8.3f %8d\n', ...
        seg_name, mean_nees, mean_pos_err, mean_innov, mean_cov_trace, max_nees, count);
    
    % Identify problematic segments
    if mean_nees > 2.5 || max_nees > 20
        problematic_segments = [problematic_segments; i];
    end
end

% Identify extreme NEES events
extreme_nees_idx = find(nees_values > 20);
if ~isempty(extreme_nees_idx)
    fprintf('\n=== EXTREME NEES EVENTS (>20) ===\n');
    fprintf('%-8s %-15s %8s %8s %8s\n', 'Time', 'Segment', 'NEES', 'PosErr', 'Innov');
    fprintf('%-8s %-15s %8s %8s %8s\n', '----', '-------', '----', '------', '-----');
    
    for i = 1:min(10, length(extreme_nees_idx))  % Show first 10
        idx = extreme_nees_idx(i);
        seg_name = char(segment_names(idx));
        fprintf('%8.1f %-15s %8.3f %8.3f %8.3f\n', ...
            timestamps(idx), seg_name, nees_values(idx), ...
            position_errors(idx), innovation_norms(idx));
    end
    
    if length(extreme_nees_idx) > 10
        fprintf('... and %d more extreme events\n', length(extreme_nees_idx) - 10);
    end
end

% Overall summary
fprintf('\n=== OVERALL SUMMARY ===\n');
fprintf('Total timesteps: %d\n', length(nees_values));
fprintf('Mean NEES: %.3f (Target: 2.0)\n', mean(nees_values));
fprintf('Median NEES: %.3f\n', median(nees_values));
fprintf('NEES > Upper Bound (2.215): %d (%.1f%%)\n', ...
    sum(nees_values > 2.215), 100 * sum(nees_values > 2.215) / length(nees_values));
fprintf('NEES > 10: %d (%.1f%%)\n', ...
    sum(nees_values > 10), 100 * sum(nees_values > 10) / length(nees_values));
fprintf('Mean position error: %.3f m\n', mean(position_errors));

% Save figure
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
filename = sprintf('figures/detailed_filter_analysis_%s.png', timestamp);
saveas(gcf, filename);
fprintf('\nDetailed analysis saved as: %s\n', filename);

end

function plotWithSegmentBackground(x_data, y_data, segment_data, segment_colors)
% Helper function to plot data with colored segment backgrounds

hold on;

% Plot background colors
y_limits = [min(y_data) * 0.95, max(y_data) * 1.05];
current_segment = segment_data(1);
segment_start = 1;

for k = 2:length(segment_data)
    if segment_data(k) ~= current_segment || k == length(segment_data)
        segment_end = k - 1;
        if k == length(segment_data)
            segment_end = k;
        end
        
        % Plot colored background
        x_range = [x_data(segment_start), x_data(segment_end)];
        color_idx = min(current_segment, size(segment_colors, 1));
        fill([x_range(1), x_range(2), x_range(2), x_range(1)], ...
             [y_limits(1), y_limits(1), y_limits(2), y_limits(2)], ...
             segment_colors(color_idx, :), 'EdgeColor', 'none', 'FaceAlpha', 0.2);
        
        current_segment = segment_data(k);
        segment_start = k;
    end
end

% Plot the data
plot(x_data, y_data, 'b-', 'LineWidth', 1.5);
ylim(y_limits);

end