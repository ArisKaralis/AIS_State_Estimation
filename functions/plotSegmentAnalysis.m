function plotSegmentAnalysis(data, filter_estimates)
% PLOTSEGMENTANALYSIS - Enhanced segment-wise performance visualization
% 
% Creates publication-quality plots showing filter performance across
% different motion segments with improved readability and styling

% Get filter names and segment information
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
        
        % COG RMSE with proper angle handling
        cog_err = angdiff(deg2rad(est.cog_est(segIdx)), deg2rad(data.cog_true(segIdx))) * 180/pi;
        cog_rmse(j, i) = sqrt(mean(cog_err.^2));
    end
end

% Create enhanced figure
figure;

% Enhanced color scheme for filters
filter_colors = [
    0.0000, 0.4470, 0.7410;  % Blue
    0.8500, 0.3250, 0.0980;  % Orange  
    0.9290, 0.6940, 0.1250;  % Yellow
    0.4940, 0.1840, 0.5560;  % Purple
    0.4660, 0.6740, 0.1880;  % Green
    0.3010, 0.7450, 0.9330;  % Cyan
    0.6350, 0.0780, 0.1840;  % Dark Red
];

% Create filter legend names
legend_names = cellfun(@(x) strrep(x, '_', '-'), filter_names, 'UniformOutput', false);

% 1. Position RMSE by segment
subplot(2, 3, 1);
b1 = bar(pos_rmse', 'grouped');
for i = 1:min(num_filters, size(filter_colors, 1))
    b1(i).FaceColor = filter_colors(i, :);
    b1(i).EdgeColor = 'none';
    b1(i).FaceAlpha = 0.8;
end

xlabel('Motion Segment', 'FontSize', 12);
ylabel('Position RMSE (m)', 'FontSize', 12);
title('Position Error by Segment', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'XTickLabel', segment_names, 'FontSize', 10);
legend(legend_names, 'Location', 'best', 'FontSize', 9);
grid on;
grid minor;

% 2. Velocity RMSE by segment
subplot(2, 3, 2);
b2 = bar(vel_rmse', 'grouped');
for i = 1:min(num_filters, size(filter_colors, 1))
    b2(i).FaceColor = filter_colors(i, :);
    b2(i).EdgeColor = 'none';
    b2(i).FaceAlpha = 0.8;
end

xlabel('Motion Segment', 'FontSize', 12);
ylabel('Velocity RMSE (m/s)', 'FontSize', 12);
title('Velocity Error by Segment', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'XTickLabel', segment_names, 'FontSize', 10);
legend(legend_names, 'Location', 'best', 'FontSize', 9);
grid on;
grid minor;

% 3. SOG RMSE by segment
subplot(2, 3, 3);
b3 = bar(sog_rmse', 'grouped');
for i = 1:min(num_filters, size(filter_colors, 1))
    b3(i).FaceColor = filter_colors(i, :);
    b3(i).EdgeColor = 'none';
    b3(i).FaceAlpha = 0.8;
end

xlabel('Motion Segment', 'FontSize', 12);
ylabel('SOG RMSE (m/s)', 'FontSize', 12);
title('Speed Over Ground Error by Segment', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'XTickLabel', segment_names, 'FontSize', 10);
legend(legend_names, 'Location', 'best', 'FontSize', 9);
grid on;
grid minor;

% 4. COG RMSE by segment
subplot(2, 3, 4);
b4 = bar(cog_rmse', 'grouped');
for i = 1:min(num_filters, size(filter_colors, 1))
    b4(i).FaceColor = filter_colors(i, :);
    b4(i).EdgeColor = 'none';
    b4(i).FaceAlpha = 0.8;
end

xlabel('Motion Segment', 'FontSize', 12);
ylabel('COG RMSE (degrees)', 'FontSize', 12);
title('Course Over Ground Error by Segment', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'XTickLabel', segment_names, 'FontSize', 10);
legend(legend_names, 'Location', 'best', 'FontSize', 9);
grid on;
grid minor;

% 5. Segment winners heatmap
subplot(2, 3, 5);
winner_matrix = zeros(4, numSegments);  % 4 metrics x numSegments

for i = 1:numSegments
    [~, pos_winner] = min(pos_rmse(:, i));
    [~, vel_winner] = min(vel_rmse(:, i));
    [~, sog_winner] = min(sog_rmse(:, i));
    [~, cog_winner] = min(cog_rmse(:, i));
    
    winner_matrix(1, i) = pos_winner;
    winner_matrix(2, i) = vel_winner;
    winner_matrix(3, i) = sog_winner;
    winner_matrix(4, i) = cog_winner;
end

% Create custom colormap for winners
cmap = filter_colors(1:num_filters, :);
imagesc(winner_matrix);
colormap(cmap);
colorbar('Ticks', 1:num_filters, 'TickLabels', legend_names);

set(gca, 'XTick', 1:numSegments, 'XTickLabel', segment_names, ...
         'YTick', 1:4, 'YTickLabel', {'Position', 'Velocity', 'SOG', 'COG'}, ...
         'FontSize', 10);
xlabel('Motion Segment', 'FontSize', 12);
ylabel('Performance Metric', 'FontSize', 12);
title('Best Filter by Metric and Segment', 'FontSize', 14, 'FontWeight', 'bold');

% 6. Overall performance ranking by segment
subplot(2, 3, 6);
overall_scores = zeros(num_filters, numSegments);

for i = 1:numSegments
    % Normalize scores by maximum in each metric for this segment
    norm_pos = pos_rmse(:, i) / max(pos_rmse(:, i));
    norm_vel = vel_rmse(:, i) / max(vel_rmse(:, i));
    norm_sog = sog_rmse(:, i) / max(sog_rmse(:, i));
    norm_cog = cog_rmse(:, i) / max(cog_rmse(:, i));
    
    % Handle edge cases where max is 0
    if max(pos_rmse(:, i)) == 0, norm_pos(:) = 0; end
    if max(vel_rmse(:, i)) == 0, norm_vel(:) = 0; end
    if max(sog_rmse(:, i)) == 0, norm_sog(:) = 0; end
    if max(cog_rmse(:, i)) == 0, norm_cog(:) = 0; end
    
    % Composite score (equal weighting)
    overall_scores(:, i) = (norm_pos + norm_vel + norm_sog + norm_cog) / 4;
end

% Plot overall scores as grouped bar chart
b6 = bar(overall_scores', 'grouped');
for i = 1:min(num_filters, size(filter_colors, 1))
    b6(i).FaceColor = filter_colors(i, :);
    b6(i).EdgeColor = 'none';
    b6(i).FaceAlpha = 0.8;
end

xlabel('Motion Segment', 'FontSize', 12);
ylabel('Normalized Score (Lower is Better)', 'FontSize', 12);
title('Overall Performance Ranking', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'XTickLabel', segment_names, 'FontSize', 10);
legend(legend_names, 'Location', 'best', 'FontSize', 9);
grid on;
grid minor;

% Overall title
sgtitle('Filter Performance Analysis by Motion Segment', 'FontSize', 16, 'FontWeight', 'bold');

% Create output directory if needed
if ~exist('figures', 'dir')
    mkdir('figures');
end

% Save with high quality
saveas(gcf, 'figures/enhanced_segment_analysis.png');
saveas(gcf, 'figures/enhanced_segment_analysis.fig');

fprintf('\nEnhanced segment analysis plot saved to figures/enhanced_segment_analysis.png\n');

% Print detailed analysis to console
fprintf('\n===== Enhanced Filter Performance Analysis by Segment =====\n');
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
    
    % Find and display best filter for each metric
    [~, bestPosIdx] = min(pos_rmse(:,i));
    [~, bestVelIdx] = min(vel_rmse(:,i));
    [~, bestSogIdx] = min(sog_rmse(:,i));
    [~, bestCogIdx] = min(cog_rmse(:,i));
    [~, bestOverallIdx] = min(overall_scores(:,i));
    
    fprintf('\nBest performers in this segment:\n');
    fprintf('  Position: %s (%.2f m)\n', strrep(filter_names{bestPosIdx}, '_', '-'), pos_rmse(bestPosIdx,i));
    fprintf('  Velocity: %s (%.2f m/s)\n', strrep(filter_names{bestVelIdx}, '_', '-'), vel_rmse(bestVelIdx,i));
    fprintf('  SOG: %s (%.2f m/s)\n', strrep(filter_names{bestSogIdx}, '_', '-'), sog_rmse(bestSogIdx,i));
    fprintf('  COG: %s (%.2f degrees)\n', strrep(filter_names{bestCogIdx}, '_', '-'), cog_rmse(bestCogIdx,i));
    fprintf('  Overall: %s (score: %.3f)\n', strrep(filter_names{bestOverallIdx}, '_', '-'), overall_scores(bestOverallIdx,i));
end

end