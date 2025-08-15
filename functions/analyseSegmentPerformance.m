function analyseSegmentPerformance(data, filter_estimates)
% ANALYSESEGMENTPERFORMANCE - Analyze filter performance by motion segment with NEES/NIS
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
if ~ismember('segment', data.Properties.VariableNames) || ~ismember('segment_name', data.Properties.VariableNames)
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

% Pre-allocate arrays for NEES/NIS by segment
nees_mean = zeros(num_filters, numSegments);
nis_mean = zeros(num_filters, numSegments);
nees_median = zeros(num_filters, numSegments);
nis_median = zeros(num_filters, numSegments);
nees_consistency = zeros(num_filters, numSegments);  % Percentage within bounds
nis_consistency = zeros(num_filters, numSegments);   % Percentage within bounds

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
        
        % NEES/NIS analysis (if available)
        if isfield(est, 'nees') && isfield(est, 'nis')
            nees_seg = est.nees(segIdx);
            nis_seg = est.nis(segIdx);
            
            % Remove NaN values for statistics
            valid_nees = ~isnan(nees_seg);
            valid_nis = ~isnan(nis_seg) & nis_seg ~= 0;
            
            if any(valid_nees)
                nees_mean(j, i) = mean(nees_seg(valid_nees));
                nees_median(j, i) = median(nees_seg(valid_nees));
                
                % NEES consistency (4-DOF system)
                alpha = 0.05;
                dof_state = 4;
                lower_bound = chi2inv(alpha/2, dof_state);
                upper_bound = chi2inv(1-alpha/2, dof_state);
                within_bounds = sum(nees_seg(valid_nees) >= lower_bound & nees_seg(valid_nees) <= upper_bound);
                nees_consistency(j, i) = (within_bounds / sum(valid_nees)) * 100;
            else
                nees_mean(j, i) = NaN;
                nees_median(j, i) = NaN;
                nees_consistency(j, i) = NaN;
            end
            
            if any(valid_nis)
                nis_mean(j, i) = mean(nis_seg(valid_nis));
                nis_median(j, i) = median(nis_seg(valid_nis));
                
                % NIS consistency (2-DOF system)
                dof_meas = 2;
                lower_bound_nis = chi2inv(alpha/2, dof_meas);
                upper_bound_nis = chi2inv(1-alpha/2, dof_meas);
                within_bounds_nis = sum(nis_seg(valid_nis) >= lower_bound_nis & nis_seg(valid_nis) <= upper_bound_nis);
                nis_consistency(j, i) = (within_bounds_nis / sum(valid_nis)) * 100;
            else
                nis_mean(j, i) = NaN;
                nis_median(j, i) = NaN;
                nis_consistency(j, i) = NaN;
            end
        else
            % No NEES/NIS data available
            nees_mean(j, i) = NaN;
            nees_median(j, i) = NaN;
            nis_mean(j, i) = NaN;
            nis_median(j, i) = NaN;
            nees_consistency(j, i) = NaN;
            nis_consistency(j, i) = NaN;
        end
    end
end

% Create improved visualization with NEES/NIS
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
    % Special handling for single segment - show filter comparison in 2x3 layout
    
    % Position RMSE comparison
    subplot(2, 3, 1);
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
    for k = 1:num_filters
        text(k, bar_data(k) + max(bar_data)*0.01, sprintf('%.2f', bar_data(k)), ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 10);
    end
    
    % Velocity RMSE comparison
    subplot(2, 3, 2);
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
    for k = 1:num_filters
        text(k, bar_data(k) + max(bar_data)*0.01, sprintf('%.2f', bar_data(k)), ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 10);
    end
    
    % SOG RMSE comparison
    subplot(2, 3, 3);
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
    for k = 1:num_filters
        text(k, bar_data(k) + max(bar_data)*0.01, sprintf('%.2f', bar_data(k)), ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 10);
    end
    
    % NEES comparison
    subplot(2, 3, 4);
    bar_data = nees_mean(:, 1);
    valid_data = ~isnan(bar_data);
    if any(valid_data)
        h4 = bar(1:num_filters, bar_data);
        h4.FaceColor = 'flat';
        h4.CData = filter_colors(1:num_filters, :);
        
        % Add expected NEES line (4 DOF)
        yline(4, 'r--', 'Expected NEES', 'LineWidth', 2);
        
        xlabel('Filter Type', 'FontSize', 12);
        ylabel('Mean NEES', 'FontSize', 12);
        title('NEES Comparison', 'FontSize', 14, 'FontWeight', 'bold');
        set(gca, 'XTick', 1:num_filters, 'XTickLabel', legend_names, 'XTickLabelRotation', 45);
        grid on;
        
        % Add value labels on bars
        for k = 1:num_filters
            if ~isnan(bar_data(k))
                text(k, bar_data(k) + max(bar_data(valid_data))*0.01, sprintf('%.2f', bar_data(k)), ...
                     'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 10);
            end
        end
    else
        text(0.5, 0.5, 'No NEES data available', 'Units', 'normalized', ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 14);
        title('NEES Comparison', 'FontSize', 14, 'FontWeight', 'bold');
    end
    
    % NIS comparison
    subplot(2, 3, 5);
    bar_data = nis_mean(:, 1);
    valid_data = ~isnan(bar_data);
    if any(valid_data)
        h5 = bar(1:num_filters, bar_data);
        h5.FaceColor = 'flat';
        h5.CData = filter_colors(1:num_filters, :);
        
        % Add expected NIS line (2 DOF)
        yline(2, 'r--', 'Expected NIS', 'LineWidth', 2);
        
        xlabel('Filter Type', 'FontSize', 12);
        ylabel('Mean NIS', 'FontSize', 12);
        title('NIS Comparison', 'FontSize', 14, 'FontWeight', 'bold');
        set(gca, 'XTick', 1:num_filters, 'XTickLabel', legend_names, 'XTickLabelRotation', 45);
        grid on;
        
        % Add value labels on bars
        for k = 1:num_filters
            if ~isnan(bar_data(k))
                text(k, bar_data(k) + max(bar_data(valid_data))*0.01, sprintf('%.2f', bar_data(k)), ...
                     'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 10);
            end
        end
    else
        text(0.5, 0.5, 'No NIS data available', 'Units', 'normalized', ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 14);
        title('NIS Comparison', 'FontSize', 14, 'FontWeight', 'bold');
    end
    
    % Consistency comparison
    subplot(2, 3, 6);
    consistency_data = [nees_consistency(:, 1), nis_consistency(:, 1)];
    valid_data = ~isnan(consistency_data(:,1)) | ~isnan(consistency_data(:,2));
    
    if any(valid_data(:))
        h6 = bar(consistency_data);
        h6(1).FaceColor = [0.2, 0.4, 0.8];  % Blue for NEES
        h6(2).FaceColor = [0.8, 0.4, 0.2];  % Orange for NIS
        
        % Add 95% line
        yline(95, 'r--', '95% Target', 'LineWidth', 2);
        
        xlabel('Filter Type', 'FontSize', 12);
        ylabel('Consistency (%)', 'FontSize', 12);
        title('Filter Consistency Comparison', 'FontSize', 14, 'FontWeight', 'bold');
        set(gca, 'XTick', 1:num_filters, 'XTickLabel', legend_names, 'XTickLabelRotation', 45);
        legend({'NEES', 'NIS'}, 'Location', 'best');
        grid on;
        ylim([0, 100]);
    else
        text(0.5, 0.5, 'No consistency data available', 'Units', 'normalized', ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 14);
        title('Filter Consistency Comparison', 'FontSize', 14, 'FontWeight', 'bold');
    end
    
    sgtitle(sprintf('Filter Performance Comparison - %s', segment_names{1}), 'FontSize', 16, 'FontWeight', 'bold');
    
else
    % Multiple segments - show grouped bar charts in 2x3 layout
    
    % Position RMSE by segment
    subplot(2, 3, 1);
    h1 = bar(pos_rmse', 'grouped');
    for k = 1:min(num_filters, size(filter_colors, 1))
        h1(k).FaceColor = filter_colors(k, :);
        h1(k).EdgeColor = 'none';
        h1(k).FaceAlpha = 0.8;
    end
    xlabel('Motion Segment', 'FontSize', 12);
    ylabel('Position RMSE (m)', 'FontSize', 12);
    title('Position Error by Segment', 'FontSize', 14, 'FontWeight', 'bold');
    set(gca, 'XTickLabel', segment_names, 'FontSize', 10);
    legend(legend_names, 'Location', 'best', 'FontSize', 9);
    grid on;
    
    % Velocity RMSE by segment
    subplot(2, 3, 2);
    h2 = bar(vel_rmse', 'grouped');
    for k = 1:min(num_filters, size(filter_colors, 1))
        h2(k).FaceColor = filter_colors(k, :);
        h2(k).EdgeColor = 'none';
        h2(k).FaceAlpha = 0.8;
    end
    xlabel('Motion Segment', 'FontSize', 12);
    ylabel('Velocity RMSE (m/s)', 'FontSize', 12);
    title('Velocity Error by Segment', 'FontSize', 14, 'FontWeight', 'bold');
    set(gca, 'XTickLabel', segment_names, 'FontSize', 10);
    legend(legend_names, 'Location', 'best', 'FontSize', 9);
    grid on;
    
    % SOG RMSE by segment
    subplot(2, 3, 3);
    h3 = bar(sog_rmse', 'grouped');
    for k = 1:min(num_filters, size(filter_colors, 1))
        h3(k).FaceColor = filter_colors(k, :);
        h3(k).EdgeColor = 'none';
        h3(k).FaceAlpha = 0.8;
    end
    xlabel('Motion Segment', 'FontSize', 12);
    ylabel('SOG RMSE (m/s)', 'FontSize', 12);
    title('Speed Over Ground Error by Segment', 'FontSize', 14, 'FontWeight', 'bold');
    set(gca, 'XTickLabel', segment_names, 'FontSize', 10);
    legend(legend_names, 'Location', 'best', 'FontSize', 9);
    grid on;
    
    % NEES by segment
    subplot(2, 3, 4);
    valid_nees = ~all(isnan(nees_mean), 1);
    if any(valid_nees)
        h4 = bar(nees_mean(:, valid_nees)', 'grouped');
        for k = 1:min(num_filters, size(filter_colors, 1))
            h4(k).FaceColor = filter_colors(k, :);
            h4(k).EdgeColor = 'none';
            h4(k).FaceAlpha = 0.8;
        end
        xlabel('Motion Segment', 'FontSize', 12);
        ylabel('Mean NEES', 'FontSize', 12);
        title('NEES by Segment', 'FontSize', 14, 'FontWeight', 'bold');
        valid_segment_names = segment_names(valid_nees);
        set(gca, 'XTickLabel', valid_segment_names, 'FontSize', 10);
        legend(legend_names, 'Location', 'best', 'FontSize', 9);
        yline(4, 'r--', 'Expected', 'LineWidth', 2);
        grid on;
    else
        text(0.5, 0.5, 'No NEES data available', 'Units', 'normalized', ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 14);
        title('NEES by Segment', 'FontSize', 14, 'FontWeight', 'bold');
    end
    
    % NIS by segment
    subplot(2, 3, 5);
    valid_nis = ~all(isnan(nis_mean), 1);
    if any(valid_nis)
        h5 = bar(nis_mean(:, valid_nis)', 'grouped');
        for k = 1:min(num_filters, size(filter_colors, 1))
            h5(k).FaceColor = filter_colors(k, :);
            h5(k).EdgeColor = 'none';
            h5(k).FaceAlpha = 0.8;
        end
        xlabel('Motion Segment', 'FontSize', 12);
        ylabel('Mean NIS', 'FontSize', 12);
        title('NIS by Segment', 'FontSize', 14, 'FontWeight', 'bold');
        valid_segment_names = segment_names(valid_nis);
        set(gca, 'XTickLabel', valid_segment_names, 'FontSize', 10);
        legend(legend_names, 'Location', 'best', 'FontSize', 9);
        yline(2, 'r--', 'Expected', 'LineWidth', 2);
        grid on;
    else
        text(0.5, 0.5, 'No NIS data available', 'Units', 'normalized', ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 14);
        title('NIS by Segment', 'FontSize', 14, 'FontWeight', 'bold');
    end
    
    % Consistency by segment
    subplot(2, 3, 6);
    valid_cons = ~all(isnan(nees_consistency), 1) | ~all(isnan(nis_consistency), 1);
    if any(valid_cons)
        % Show average consistency across all segments
        avg_nees_cons = nanmean(nees_consistency, 2);
        avg_nis_cons = nanmean(nis_consistency, 2);
        consistency_data = [avg_nees_cons, avg_nis_cons];
        
        h6 = bar(consistency_data);
        h6(1).FaceColor = [0.2, 0.4, 0.8];  % Blue for NEES
        h6(2).FaceColor = [0.8, 0.4, 0.2];  % Orange for NIS
        
        yline(95, 'r--', '95% Target', 'LineWidth', 2);
        
        xlabel('Filter Type', 'FontSize', 12);
        ylabel('Avg Consistency (%)', 'FontSize', 12);
        title('Average Filter Consistency', 'FontSize', 14, 'FontWeight', 'bold');
        set(gca, 'XTick', 1:num_filters, 'XTickLabel', legend_names, 'XTickLabelRotation', 45);
        legend({'NEES', 'NIS'}, 'Location', 'best');
        grid on;
        ylim([0, 100]);
    else
        text(0.5, 0.5, 'No consistency data available', 'Units', 'normalized', ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 14);
        title('Average Filter Consistency', 'FontSize', 14, 'FontWeight', 'bold');
    end
    
    sgtitle('Filter Performance by Motion Segment', 'FontSize', 16, 'FontWeight', 'bold');
end

% Create output directory if needed
if ~exist('figures', 'dir')
    mkdir('figures');
end

% Save figure
saveas(gcf, 'figures/segment_performance.png');
saveas(gcf, 'figures/segment_performance.fig');

% Print segment analysis with NEES/NIS
fprintf('\n===== Filter Performance by Segment with NEES/NIS =====\n');
for i = 1:numSegments
    fprintf('\nSegment %d: %s\n', segments(i), segment_names{i});
    fprintf('%-15s | %-10s | %-10s | %-10s | %-10s | %-10s | %-10s | %-8s | %-8s\n', ...
            'Filter', 'Pos RMSE', 'Vel RMSE', 'SOG RMSE', 'COG RMSE', 'NEES', 'NIS', 'NEES%', 'NIS%');
    fprintf('----------------------------------------------------------------------------------------------------------------------\n');
    
    for j = 1:num_filters
        display_name = strrep(filter_names{j}, '_', '-');
        
        % Get values
        nees_val = nees_mean(j, i);
        nis_val = nis_mean(j, i);
        nees_cons = nees_consistency(j, i);
        nis_cons = nis_consistency(j, i);
        
        % Format NaN values for display
        if isnan(nees_val)
            nees_str = 'N/A';
        else
            nees_str = sprintf('%.3f', nees_val);
        end
        
        if isnan(nis_val)
            nis_str = 'N/A';
        else
            nis_str = sprintf('%.3f', nis_val);
        end
        
        if isnan(nees_cons)
            nees_cons_str = 'N/A';
        else
            nees_cons_str = sprintf('%.1f', nees_cons);
        end
        
        if isnan(nis_cons)
            nis_cons_str = 'N/A';
        else
            nis_cons_str = sprintf('%.1f', nis_cons);
        end
        
        fprintf('%-15s | %-10.2f | %-10.2f | %-10.2f | %-10.2f | %-10s | %-10s | %-8s | %-8s\n', ...
                display_name, pos_rmse(j,i), vel_rmse(j,i), sog_rmse(j,i), cog_rmse(j,i), ...
                nees_str, nis_str, nees_cons_str, nis_cons_str);
    end
    
    % Find best filter for each metric in this segment
    [~, bestPosIdx] = min(pos_rmse(:,i));
    [~, bestVelIdx] = min(vel_rmse(:,i));
    [~, bestSogIdx] = min(sog_rmse(:,i));
    [~, bestCogIdx] = min(cog_rmse(:,i));
    
    % Find most consistent filters
    valid_nees_cons = ~isnan(nees_consistency(:,i));
    valid_nis_cons = ~isnan(nis_consistency(:,i));
    
    fprintf('\nBest performers in this segment:\n');
    fprintf('  Position: %s (%.2f m)\n', strrep(filter_names{bestPosIdx}, '_', '-'), pos_rmse(bestPosIdx,i));
    fprintf('  Velocity: %s (%.2f m/s)\n', strrep(filter_names{bestVelIdx}, '_', '-'), vel_rmse(bestVelIdx,i));
    fprintf('  SOG: %s (%.2f m/s)\n', strrep(filter_names{bestSogIdx}, '_', '-'), sog_rmse(bestSogIdx,i));
    fprintf('  COG: %s (%.2f deg)\n', strrep(filter_names{bestCogIdx}, '_', '-'), cog_rmse(bestCogIdx,i));
    
    if any(valid_nees_cons)
        [~, bestNeesConsIdx] = max(nees_consistency(valid_nees_cons,i));
        valid_indices = find(valid_nees_cons);
        actual_best_idx = valid_indices(bestNeesConsIdx);
        fprintf('  NEES Consistency: %s (%.1f%%)\n', strrep(filter_names{actual_best_idx}, '_', '-'), nees_consistency(actual_best_idx,i));
    end
    
    if any(valid_nis_cons)
        [~, bestNisConsIdx] = max(nis_consistency(valid_nis_cons,i));
        valid_indices = find(valid_nis_cons);
        actual_best_idx = valid_indices(bestNisConsIdx);
        fprintf('  NIS Consistency: %s (%.1f%%)\n', strrep(filter_names{actual_best_idx}, '_', '-'), nis_consistency(actual_best_idx,i));
    end
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
fprintf('%-15s | %-12s | %-12s | %-12s | %-12s | %-10s | %-10s\n', ...
        'Filter', 'Avg Pos RMSE', 'Avg Vel RMSE', 'Avg SOG RMSE', 'Avg COG RMSE', 'Avg NEES', 'Avg NIS');
fprintf('--------------------------------------------------------------------------------------------------------------\n');

for j = 1:num_filters
    display_name = strrep(filter_names{j}, '_', '-');
    avg_pos = mean(pos_rmse(j,:));
    avg_vel = mean(vel_rmse(j,:));
    avg_sog = mean(sog_rmse(j,:));
    avg_cog = mean(cog_rmse(j,:));
    avg_nees = nanmean(nees_mean(j,:));
    avg_nis = nanmean(nis_mean(j,:));
    
    % Format NaN values
    if isnan(avg_nees)
        nees_str = 'N/A';
    else
        nees_str = sprintf('%.3f', avg_nees);
    end
    
    if isnan(avg_nis)
        nis_str = 'N/A';
    else
        nis_str = sprintf('%.3f', avg_nis);
    end
    
    fprintf('%-15s | %-12.2f | %-12.2f | %-12.2f | %-12.2f | %-10s | %-10s\n', ...
            display_name, avg_pos, avg_vel, avg_sog, avg_cog, nees_str, nis_str);
end

% Show segment winner summary
fprintf('\n===== Segment Winner Summary =====\n');
for j = 1:num_filters
    wins = sum(segment_winners == j);
    display_name = strrep(filter_names{j}, '_', '-');
    fprintf('%s: %d out of %d segments\n', display_name, wins, numSegments);
end

end