function analyseSegmentPerformance(data, filter_estimates)
% ANALYSESEGMENTPERFORMANCE - Analyze filter performance by motion segment
% 
% This function evaluates how well each filter performs during different
% motion segments (straight line, turns, acceleration phases, etc.)
%
% Input:
%   data - Table containing true trajectory data with segment information
%   filter_estimates - Struct containing estimates from all filters
%
% The function expects data to have the following fields:
%   - x_true, y_true: True position coordinates  
%   - vx_true, vy_true: True velocity components
%   - sog_true: True speed over ground
%   - cog_true: True course over ground
%   - segment: Motion segment ID (optional)
%   - segment_name: Descriptive segment names (optional)

% Get filter names
filter_names = fieldnames(filter_estimates);
num_filters = length(filter_names);

% Validate segment information
if ~isfield(data, 'segment') || ~isfield(data, 'segment_name')
    warning('Segment information not found in data. Creating single segment.');
    data.segment = ones(height(data), 1);
    data.segment_name = repmat("All Data", height(data), 1);
end

% Get unique segments
segments = unique(data.segment);
numSegments = length(segments);
segment_names = cell(numSegments, 1);

% Pre-allocate arrays for RMSE by segment
pos_rmse = zeros(num_filters, numSegments);
vel_rmse = zeros(num_filters, numSegments);
sog_rmse = zeros(num_filters, numSegments);
cog_rmse = zeros(num_filters, numSegments);

% Calculate performance metrics for each segment
for i = 1:numSegments
    seg = segments(i);
    segIdx = data.segment == seg;
    segment_names{i} = char(data.segment_name(find(segIdx, 1)));
    
    % Calculate RMSE for each filter in this segment
    for j = 1:num_filters
        filter_name = filter_names{j};
        est = filter_estimates.(filter_name);
        
        % Position RMSE
        pos_err = sqrt((est.x_est(segIdx) - data.x_true(segIdx)).^2 + ...
                      (est.y_est(segIdx) - data.y_true(segIdx)).^2);
        pos_rmse(j, i) = sqrt(mean(pos_err.^2));
        
        % Velocity RMSE
        vel_err = sqrt((est.vx_est(segIdx) - data.vx_true(segIdx)).^2 + ...
                      (est.vy_est(segIdx) - data.vy_true(segIdx)).^2);
        vel_rmse(j, i) = sqrt(mean(vel_err.^2));
        
        % SOG RMSE
        sog_rmse(j, i) = sqrt(mean((est.sog_est(segIdx) - data.sog_true(segIdx)).^2));
        
        % COG RMSE
        cog_err = angdiff(deg2rad(est.cog_est(segIdx)), deg2rad(data.cog_true(segIdx))) * 180/pi;
        cog_rmse(j, i) = sqrt(mean(cog_err.^2));
    end
end

% Create improved visualization
figure;

% Define colors for filters (colorblind-friendly palette)
filter_colors = [
    0.0000, 0.4470, 0.7410;  % Blue
    0.8500, 0.3250, 0.0980;  % Orange  
    0.9290, 0.6940, 0.1250;  % Yellow
    0.4940, 0.1840, 0.5560;  % Purple
    0.4660, 0.6740, 0.1880;  % Green
    0.3010, 0.7450, 0.9330;  % Cyan
    0.6350, 0.0780, 0.1840;  % Dark Red
    0.2500, 0.2500, 0.2500;  % Dark Gray
    0.8000, 0.4000, 0.8000;  % Light Purple
    0.0000, 0.5000, 0.0000;  % Dark Green
];

% Ensure we have enough colors for all filters
if num_filters > size(filter_colors, 1)
    % Generate additional colors using a colormap
    additional_colors = hsv(num_filters - size(filter_colors, 1));
    filter_colors = [filter_colors; additional_colors];
end

% Create filter legend names
legend_names = cellfun(@(x) strrep(x, '_', '-'), filter_names, 'UniformOutput', false);

if numSegments == 1
    % Special handling for single segment - show filter comparison
    
    % Position RMSE comparison
    subplot(2, 2, 1);
    bar_data = pos_rmse(:, 1);
    h1 = bar(1:num_filters, bar_data);
    h1.FaceColor = 'flat';
    h1.CData = filter_colors(1:num_filters, :);
    xlabel('Filter Type', 'FontSize', 12);
    ylabel('Position RMSE (m)', 'FontSize', 12);
    title('Position Error Comparison', 'FontSize', 14, 'FontWeight', 'bold');
    set(gca, 'XTick', 1:num_filters, 'XTickLabel', legend_names, 'XTickLabelRotation', 45);
    grid on;
    
    % Add value labels on bars
    for i = 1:num_filters
        text(i, bar_data(i) + max(bar_data)*0.01, sprintf('%.2f', bar_data(i)), ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 10);
    end
    
    % Velocity RMSE comparison
    subplot(2, 2, 2);
    bar_data = vel_rmse(:, 1);
    h2 = bar(1:num_filters, bar_data);
    h2.FaceColor = 'flat';
    h2.CData = filter_colors(1:num_filters, :);
    xlabel('Filter Type', 'FontSize', 12);
    ylabel('Velocity RMSE (m/s)', 'FontSize', 12);
    title('Velocity Error Comparison', 'FontSize', 14, 'FontWeight', 'bold');
    set(gca, 'XTick', 1:num_filters, 'XTickLabel', legend_names, 'XTickLabelRotation', 45);
    grid on;
    
    % Add value labels on bars
    for i = 1:num_filters
        text(i, bar_data(i) + max(bar_data)*0.01, sprintf('%.2f', bar_data(i)), ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 10);
    end
    
    % SOG RMSE comparison
    subplot(2, 2, 3);
    bar_data = sog_rmse(:, 1);
    h3 = bar(1:num_filters, bar_data);
    h3.FaceColor = 'flat';
    h3.CData = filter_colors(1:num_filters, :);
    xlabel('Filter Type', 'FontSize', 12);
    ylabel('SOG RMSE (m/s)', 'FontSize', 12);
    title('Speed Over Ground Error Comparison', 'FontSize', 14, 'FontWeight', 'bold');
    set(gca, 'XTick', 1:num_filters, 'XTickLabel', legend_names, 'XTickLabelRotation', 45);
    grid on;
    
    % Add value labels on bars
    for i = 1:num_filters
        text(i, bar_data(i) + max(bar_data)*0.01, sprintf('%.2f', bar_data(i)), ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 10);
    end
    
    % COG RMSE comparison
    subplot(2, 2, 4);
    bar_data = cog_rmse(:, 1);
    h4 = bar(1:num_filters, bar_data);
    h4.FaceColor = 'flat';
    h4.CData = filter_colors(1:num_filters, :);
    xlabel('Filter Type', 'FontSize', 12);
    ylabel('COG RMSE (degrees)', 'FontSize', 12);
    title('Course Over Ground Error Comparison', 'FontSize', 14, 'FontWeight', 'bold');
    set(gca, 'XTick', 1:num_filters, 'XTickLabel', legend_names, 'XTickLabelRotation', 45);
    grid on;
    
    % Add value labels on bars
    for i = 1:num_filters
        text(i, bar_data(i) + max(bar_data)*0.01, sprintf('%.2f', bar_data(i)), ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 10);
    end
    
    sgtitle(sprintf('Filter Performance Comparison - %s', segment_names{1}), 'FontSize', 16, 'FontWeight', 'bold');
    
else
    % Multiple segments - show grouped bar charts
    
    % Position RMSE by segment
    subplot(2, 2, 1);
    h1 = bar(pos_rmse', 'grouped');
    for i = 1:min(num_filters, size(filter_colors, 1))
        h1(i).FaceColor = filter_colors(i, :);
        h1(i).EdgeColor = 'none';
        h1(i).FaceAlpha = 0.8;
    end
    xlabel('Motion Segment', 'FontSize', 12);
    ylabel('Position RMSE (m)', 'FontSize', 12);
    title('Position Error by Segment', 'FontSize', 14, 'FontWeight', 'bold');
    set(gca, 'XTickLabel', segment_names, 'FontSize', 10);
    legend(legend_names, 'Location', 'best', 'FontSize', 9);
    grid on;
    
    % Velocity RMSE by segment
    subplot(2, 2, 2);
    h2 = bar(vel_rmse', 'grouped');
    for i = 1:min(num_filters, size(filter_colors, 1))
        h2(i).FaceColor = filter_colors(i, :);
        h2(i).EdgeColor = 'none';
        h2(i).FaceAlpha = 0.8;
    end
    xlabel('Motion Segment', 'FontSize', 12);
    ylabel('Velocity RMSE (m/s)', 'FontSize', 12);
    title('Velocity Error by Segment', 'FontSize', 14, 'FontWeight', 'bold');
    set(gca, 'XTickLabel', segment_names, 'FontSize', 10);
    legend(legend_names, 'Location', 'best', 'FontSize', 9);
    grid on;
    
    % SOG RMSE by segment
    subplot(2, 2, 3);
    h3 = bar(sog_rmse', 'grouped');
    for i = 1:min(num_filters, size(filter_colors, 1))
        h3(i).FaceColor = filter_colors(i, :);
        h3(i).EdgeColor = 'none';
        h3(i).FaceAlpha = 0.8;
    end
    xlabel('Motion Segment', 'FontSize', 12);
    ylabel('SOG RMSE (m/s)', 'FontSize', 12);
    title('Speed Over Ground Error by Segment', 'FontSize', 14, 'FontWeight', 'bold');
    set(gca, 'XTickLabel', segment_names, 'FontSize', 10);
    legend(legend_names, 'Location', 'best', 'FontSize', 9);
    grid on;
    
    % COG RMSE by segment
    subplot(2, 2, 4);
    h4 = bar(cog_rmse', 'grouped');
    for i = 1:min(num_filters, size(filter_colors, 1))
        h4(i).FaceColor = filter_colors(i, :);
        h4(i).EdgeColor = 'none';
        h4(i).FaceAlpha = 0.8;
    end
    xlabel('Motion Segment', 'FontSize', 12);
    ylabel('COG RMSE (degrees)', 'FontSize', 12);
    title('Course Over Ground Error by Segment', 'FontSize', 14, 'FontWeight', 'bold');
    set(gca, 'XTickLabel', segment_names, 'FontSize', 10);
    legend(legend_names, 'Location', 'best', 'FontSize', 9);
    grid on;
    
    sgtitle('Filter Performance by Motion Segment', 'FontSize', 16, 'FontWeight', 'bold');
end

% Create output directory if needed
if ~exist('figures', 'dir')
    mkdir('figures');
end

% Save figure
saveas(gcf, 'figures/segment_performance.png');
saveas(gcf, 'figures/segment_performance.fig');

% Print segment analysis
fprintf('\n===== Filter Performance by Segment =====\n');
for i = 1:numSegments
    fprintf('\nSegment %d: %s\n', segments(i), segment_names{i});
    fprintf('%-15s | %-10s | %-10s | %-10s | %-10s\n', ...
            'Filter', 'Pos RMSE', 'Vel RMSE', 'SOG RMSE', 'COG RMSE');
    fprintf('--------------------------------------------------------------------------\n');
    
    for j = 1:num_filters
        display_name = strrep(filter_names{j}, '_', '-');
        fprintf('%-15s | %-10.2f | %-10.2f | %-10.2f | %-10.2f\n', ...
                display_name, pos_rmse(j,i), vel_rmse(j,i), sog_rmse(j,i), cog_rmse(j,i));
    end
    
    % Find best filter for each metric in this segment
    [~, bestPosIdx] = min(pos_rmse(:,i));
    [~, bestVelIdx] = min(vel_rmse(:,i));
    [~, bestSogIdx] = min(sog_rmse(:,i));
    [~, bestCogIdx] = min(cog_rmse(:,i));
    
    fprintf('\nBest performers in this segment:\n');
    fprintf('  Position: %s (%.2f m)\n', strrep(filter_names{bestPosIdx}, '_', '-'), pos_rmse(bestPosIdx,i));
    fprintf('  Velocity: %s (%.2f m/s)\n', strrep(filter_names{bestVelIdx}, '_', '-'), vel_rmse(bestVelIdx,i));
    fprintf('  SOG: %s (%.2f m/s)\n', strrep(filter_names{bestSogIdx}, '_', '-'), sog_rmse(bestSogIdx,i));
    fprintf('  COG: %s (%.2f deg)\n', strrep(filter_names{bestCogIdx}, '_', '-'), cog_rmse(bestCogIdx,i));
end

% Determine overall segment winner (best in most metrics)
segment_winners = zeros(1, numSegments);
for i = 1:numSegments
    [~, bestPosIdx] = min(pos_rmse(:,i));
    [~, bestVelIdx] = min(vel_rmse(:,i));
    [~, bestSogIdx] = min(sog_rmse(:,i));
    [~, bestCogIdx] = min(cog_rmse(:,i));
    
    % Count wins for each filter
    wins = zeros(num_filters, 1);
    wins(bestPosIdx) = wins(bestPosIdx) + 1;
    wins(bestVelIdx) = wins(bestVelIdx) + 1;
    wins(bestSogIdx) = wins(bestSogIdx) + 1;
    wins(bestCogIdx) = wins(bestCogIdx) + 1;
    
    [~, bestIdx] = max(wins);
    segment_winners(i) = bestIdx;
    
    fprintf('\nOverall winner for Segment %d (%s): %s\n', ...
            segments(i), segment_names{i}, strrep(filter_names{bestIdx}, '_', '-'));
end

% Create summary statistics across all segments
fprintf('\n===== Summary Statistics Across All Segments =====\n');
fprintf('%-15s | %-12s | %-12s | %-12s | %-12s\n', ...
        'Filter', 'Avg Pos RMSE', 'Avg Vel RMSE', 'Avg SOG RMSE', 'Avg COG RMSE');
fprintf('------------------------------------------------------------------------------------\n');

for j = 1:num_filters
    display_name = strrep(filter_names{j}, '_', '-');
    avg_pos = mean(pos_rmse(j,:));
    avg_vel = mean(vel_rmse(j,:));
    avg_sog = mean(sog_rmse(j,:));
    avg_cog = mean(cog_rmse(j,:));
    
    fprintf('%-15s | %-12.2f | %-12.2f | %-12.2f | %-12.2f\n', ...
            display_name, avg_pos, avg_vel, avg_sog, avg_cog);
end

% Show segment winner summary
fprintf('\n===== Segment Winner Summary =====\n');
for j = 1:num_filters
    wins = sum(segment_winners == j);
    display_name = strrep(filter_names{j}, '_', '-');
    fprintf('%s: %d out of %d segments\n', display_name, wins, numSegments);
end

end