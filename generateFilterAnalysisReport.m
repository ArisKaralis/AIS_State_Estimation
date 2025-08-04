function generateFilterAnalysisReport(results)
% Generate comprehensive visual analysis report with console summary instead of figure text
figure;

filterNames = results.comparison.filterTypes;

% Plot 1: Position RMSE comparison
subplot(3, 4, 1);
pos_means = results.comparison.position_rmse.means;
pos_stds = results.comparison.position_rmse.stds;
validIdx = ~isnan(pos_means);
if any(validIdx)
    errorbar(find(validIdx), pos_means(validIdx), pos_stds(validIdx), 'o-', 'LineWidth', 2, 'MarkerSize', 8);
    set(gca, 'XTick', find(validIdx), 'XTickLabel', filterNames(validIdx));
end
ylabel('Position RMSE (m)');
title('Position RMSE Comparison');
grid on;

% Plot 2: Velocity RMSE comparison
subplot(3, 4, 2);
vel_means = results.comparison.velocity_rmse.means;
vel_stds = results.comparison.velocity_rmse.stds;
validIdx = ~isnan(vel_means);
if any(validIdx)
    errorbar(find(validIdx), vel_means(validIdx), vel_stds(validIdx), 's-', 'LineWidth', 2, 'MarkerSize', 8);
    set(gca, 'XTick', find(validIdx), 'XTickLabel', filterNames(validIdx));
end
ylabel('Velocity RMSE (m/s)');
title('Velocity RMSE Comparison');
grid on;

% Plot 3: SOG RMSE comparison
subplot(3, 4, 3);
sog_means = results.comparison.sog_rmse.means;
sog_stds = results.comparison.sog_rmse.stds;
validIdx = ~isnan(sog_means);
if any(validIdx)
    errorbar(find(validIdx), sog_means(validIdx), sog_stds(validIdx), '^-', 'LineWidth', 2, 'MarkerSize', 8);
    set(gca, 'XTick', find(validIdx), 'XTickLabel', filterNames(validIdx));
end
ylabel('SOG RMSE (m/s)');
title('SOG RMSE Comparison');
grid on;

% Plot 4: COG RMSE comparison
subplot(3, 4, 4);
cog_means = results.comparison.cog_rmse.means;
cog_stds = results.comparison.cog_rmse.stds;
validIdx = ~isnan(cog_means);
if any(validIdx)
    errorbar(find(validIdx), cog_means(validIdx), cog_stds(validIdx), 'd-', 'LineWidth', 2, 'MarkerSize', 8);
    set(gca, 'XTick', find(validIdx), 'XTickLabel', filterNames(validIdx));
end
ylabel('COG RMSE (°)');
title('COG RMSE Comparison');
grid on;

% Plot 5: All metrics bar chart
subplot(3, 4, 5);
if any(~isnan(pos_means))
    % Normalize metrics for comparison (position as reference)
    pos_norm = pos_means / min(pos_means(~isnan(pos_means)));
    vel_norm = vel_means / min(vel_means(~isnan(vel_means)));
    sog_norm = sog_means / min(sog_means(~isnan(sog_means)));
    cog_norm = cog_means / min(cog_means(~isnan(cog_means)));
    
    validFilters = ~isnan(pos_means);
    if any(validFilters)
        metrics_data = [pos_norm(validFilters)', vel_norm(validFilters)', ...
                       sog_norm(validFilters)', cog_norm(validFilters)'];
        bar(metrics_data);
        set(gca, 'XTickLabel', filterNames(validFilters));
        ylabel('Normalized RMSE');
        title('All Metrics Comparison');
        legend({'Position', 'Velocity', 'SOG', 'COG'}, 'Location', 'best');
    end
end
grid on;

% Plot 6: Measurement vs Filter Performance (Position)
subplot(3, 4, 6);
if isfield(results, 'measurementStats') && ~isempty(results.measurementStats.position_rmse)
    measurementRMSE = mean(results.measurementStats.position_rmse);
    data_to_plot = [measurementRMSE];
    labels = {'Measurements'};
    
    for i = 1:length(filterNames)
        if ~isnan(pos_means(i))
            data_to_plot(end+1) = pos_means(i);
            labels{end+1} = filterNames{i};
        end
    end
    
    bar(data_to_plot);
    set(gca, 'XTickLabel', labels, 'XTickLabelRotation', 45);
    ylabel('Position RMSE (m)');
    title('Position: Measurement vs Filter');
    grid on;
end

% Plot 7: Measurement vs Filter Performance (SOG)
subplot(3, 4, 7);
if isfield(results, 'measurementStats') && ~isempty(results.measurementStats.sog_rmse)
    measurementSOGRMSE = mean(results.measurementStats.sog_rmse);
    data_to_plot = [measurementSOGRMSE];
    labels = {'Measurements'};
    
    for i = 1:length(filterNames)
        if ~isnan(sog_means(i))
            data_to_plot(end+1) = sog_means(i);
            labels{end+1} = filterNames{i};
        end
    end
    
    bar(data_to_plot);
    set(gca, 'XTickLabel', labels, 'XTickLabelRotation', 45);
    ylabel('SOG RMSE (m/s)');
    title('SOG: Measurement vs Filter');
    grid on;
end

% Plot 8: Measurement vs Filter Performance (COG)
subplot(3, 4, 8);
if isfield(results, 'measurementStats') && ~isempty(results.measurementStats.cog_rmse)
    measurementCOGRMSE = mean(results.measurementStats.cog_rmse);
    data_to_plot = [measurementCOGRMSE];
    labels = {'Measurements'};
    
    for i = 1:length(filterNames)
        if ~isnan(cog_means(i))
            data_to_plot(end+1) = cog_means(i);
            labels{end+1} = filterNames{i};
        end
    end
    
    bar(data_to_plot);
    set(gca, 'XTickLabel', labels, 'XTickLabelRotation', 45);
    ylabel('COG RMSE (°)');
    title('COG: Measurement vs Filter');
    grid on;
end

% Plot 9: Runtime comparison
subplot(3, 4, 9);
runtime_means = [];
runtime_stds = [];
for i = 1:length(filterNames)
    filterName = filterNames{i};
    if isfield(results.filterStats.(filterName), 'summary') && isfield(results.filterStats.(filterName).summary, 'runtime')
        runtime_means(i) = results.filterStats.(filterName).summary.runtime.mean;
        runtime_stds(i) = results.filterStats.(filterName).summary.runtime.std;
    else
        runtime_means(i) = NaN;
        runtime_stds(i) = NaN;
    end
end

validIdx = ~isnan(runtime_means);
if any(validIdx)
    errorbar(find(validIdx), runtime_means(validIdx), runtime_stds(validIdx), 'p-', 'LineWidth', 2, 'MarkerSize', 8);
    set(gca, 'XTick', find(validIdx), 'XTickLabel', filterNames(validIdx));
end
ylabel('Runtime (s)');
title('Computational Performance');
grid on;

% Plot 10: Measurement quality statistics histograms
subplot(3, 4, 10);
if isfield(results, 'measurementStats') && ~isempty(results.measurementStats.position_rmse)
    histogram(results.measurementStats.position_rmse, min(20, length(results.measurementStats.position_rmse)));
    xlabel('Position Measurement RMSE (m)');
    ylabel('Frequency');
    title('Measurement Quality Distribution');
    grid on;
end

% Plot 11: Best filter error evolution
subplot(3, 4, 11);
if ~strcmp(results.comparison.best_filter, 'None')
    bestFilter = results.comparison.best_filter;
    runsToShow = min(3, size(results.rawResults, 1));
    plotCount = 0;
    for run = 1:size(results.rawResults, 1)
        for f = 1:length(filterNames)
            if strcmp(filterNames{f}, bestFilter) && ~isempty(results.rawResults{run, f})
                if isfield(results.rawResults{run, f}, 'position_errors')
                    plot(results.rawResults{run, f}.position_errors, 'DisplayName', sprintf('Run %d', run));
                    hold on;
                    plotCount = plotCount + 1;
                    if plotCount >= runsToShow
                        break;
                    end
                end
                break;
            end
        end
        if plotCount >= runsToShow
            break;
        end
    end
    if plotCount > 0
        xlabel('Time Step');
        ylabel('Position Error (m)');
        title(sprintf('Error Evolution - %s Filter', bestFilter));
        legend('Location', 'best');
    end
end
grid on;

% Plot 12: Additional analysis plot (instead of text summary)
subplot(3, 4, 12);
if any(~isnan(pos_means)) && isfield(results, 'measurementStats') && ~isempty(results.measurementStats.position_rmse)
    % Show improvement percentages over measurements
    measurementRMSE = mean(results.measurementStats.position_rmse);
    improvements = [];
    valid_filters = {};
    
    for i = 1:length(filterNames)
        if ~isnan(pos_means(i))
            improvement = ((measurementRMSE - pos_means(i)) / measurementRMSE) * 100;
            improvements(end+1) = improvement;
            valid_filters{end+1} = filterNames{i};
        end
    end
    
    if ~isempty(improvements)
        bar(improvements);
        set(gca, 'XTickLabel', valid_filters, 'XTickLabelRotation', 45);
        ylabel('Improvement (%)');
        title('Filter Improvement over Measurements');
        grid on;
        
        % Add horizontal line at 0% (no improvement)
        hold on;
        plot([0.5, length(improvements)+0.5], [0, 0], 'r--', 'LineWidth', 2);
        hold off;
    end
else
    % If no data, just show a simple message
    text(0.5, 0.5, 'Analysis Complete', 'HorizontalAlignment', 'center', ...
         'VerticalAlignment', 'middle', 'FontSize', 14, 'FontWeight', 'bold');
end

sgtitle(sprintf('Monte Carlo Filter Analysis - Separate Metrics (%d runs)', results.metadata.numRuns));

% Print detailed summary to console instead of figure
printDetailedFilterSummary(results);

% Save figure
if ~exist('output', 'dir')
    mkdir('output');
end
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
filename = sprintf('output/best_filters_separate_metrics_%s.png', timestamp);
saveas(gcf, filename);
fprintf('Filter analysis report with separate metrics saved to: %s\n', filename);
end

function printDetailedFilterSummary(results)
% Print comprehensive filter performance summary to console
fprintf('\n');
fprintf('================================================================================\n');
fprintf('                    DETAILED FILTER PERFORMANCE SUMMARY\n');
fprintf('================================================================================\n');

filterNames = results.comparison.filterTypes;
pos_means = results.comparison.position_rmse.means;
pos_stds = results.comparison.position_rmse.stds;
vel_means = results.comparison.velocity_rmse.means;
vel_stds = results.comparison.velocity_rmse.stds;
sog_means = results.comparison.sog_rmse.means;
sog_stds = results.comparison.sog_rmse.stds;
cog_means = results.comparison.cog_rmse.means;
cog_stds = results.comparison.cog_rmse.stds;

% Print best performers for each metric
fprintf('\n===== BEST PERFORMERS BY METRIC =====\n');
if isfield(results.comparison, 'best_filters')
    if isfield(results.comparison.best_filters, 'position')
        fprintf('Position RMSE: %s\n', results.comparison.best_filters.position);
    end
    if isfield(results.comparison.best_filters, 'velocity')
        fprintf('Velocity RMSE: %s\n', results.comparison.best_filters.velocity);
    end
    if isfield(results.comparison.best_filters, 'sog')
        fprintf('SOG RMSE: %s\n', results.comparison.best_filters.sog);
    end
    if isfield(results.comparison.best_filters, 'cog')
        fprintf('COG RMSE: %s\n', results.comparison.best_filters.cog);
    end
end

% Print detailed performance for each filter
fprintf('\n===== DETAILED FILTER PERFORMANCE =====\n');
for i = 1:length(filterNames)
    filterName = filterNames{i};
    if ~isnan(pos_means(i))
        fprintf('\n%s:\n', filterName);
        fprintf('  Position RMSE: %.3f ± %.3f m  (range: %.3f to %.3f m)\n', ...
            pos_means(i), pos_stds(i), ...
            pos_means(i) - pos_stds(i), pos_means(i) + pos_stds(i));
        
        if ~isnan(vel_means(i))
            fprintf('  Velocity RMSE: %.3f ± %.3f m/s (range: %.3f to %.3f m/s)\n', ...
                vel_means(i), vel_stds(i), ...
                vel_means(i) - vel_stds(i), vel_means(i) + vel_stds(i));
        end
        
        if ~isnan(sog_means(i))
            fprintf('  SOG RMSE:      %.3f ± %.3f m/s (range: %.3f to %.3f m/s)\n', ...
                sog_means(i), sog_stds(i), ...
                sog_means(i) - sog_stds(i), sog_means(i) + sog_stds(i));
        end
        
        if ~isnan(cog_means(i))
            fprintf('  COG RMSE:      %.3f ± %.3f°   (range: %.3f to %.3f°)\n', ...
                cog_means(i), cog_stds(i), ...
                cog_means(i) - cog_stds(i), cog_means(i) + cog_stds(i));
        end
        
        % Add runtime information
        if isfield(results.filterStats.(filterName), 'summary') && ...
           isfield(results.filterStats.(filterName).summary, 'runtime')
            runtime_mean = results.filterStats.(filterName).summary.runtime.mean;
            runtime_std = results.filterStats.(filterName).summary.runtime.std;
            fprintf('  Runtime:       %.4f ± %.4f s\n', runtime_mean, runtime_std);
        end
    end
end

% Print measurement quality statistics
if isfield(results, 'measurementStats') && ~isempty(results.measurementStats.position_rmse)
    fprintf('\n===== MEASUREMENT QUALITY BASELINE =====\n');
    fprintf('Position measurement RMSE: %.3f ± %.3f m  (range: %.3f to %.3f m)\n', ...
        mean(results.measurementStats.position_rmse), std(results.measurementStats.position_rmse), ...
        min(results.measurementStats.position_rmse), max(results.measurementStats.position_rmse));
    fprintf('Velocity measurement RMSE: %.3f ± %.3f m/s (range: %.3f to %.3f m/s)\n', ...
        mean(results.measurementStats.velocity_rmse), std(results.measurementStats.velocity_rmse), ...
        min(results.measurementStats.velocity_rmse), max(results.measurementStats.velocity_rmse));
    fprintf('SOG measurement RMSE:      %.3f ± %.3f m/s (range: %.3f to %.3f m/s)\n', ...
        mean(results.measurementStats.sog_rmse), std(results.measurementStats.sog_rmse), ...
        min(results.measurementStats.sog_rmse), max(results.measurementStats.sog_rmse));
    fprintf('COG measurement RMSE:      %.3f ± %.3f°   (range: %.3f to %.3f°)\n', ...
        mean(results.measurementStats.cog_rmse), std(results.measurementStats.cog_rmse), ...
        min(results.measurementStats.cog_rmse), max(results.measurementStats.cog_rmse));
    fprintf('Average observations per run: %.1f\n', mean(results.measurementStats.observation_counts));
end

% Print improvement analysis
if isfield(results, 'measurementStats') && ~isempty(results.measurementStats.position_rmse)
    fprintf('\n===== FILTER IMPROVEMENT OVER MEASUREMENTS =====\n');
    measurementRMSE = mean(results.measurementStats.position_rmse);
    measurementSOGRMSE = mean(results.measurementStats.sog_rmse);
    measurementCOGRMSE = mean(results.measurementStats.cog_rmse);
    
    for i = 1:length(filterNames)
        filterName = filterNames{i};
        if ~isnan(pos_means(i))
            pos_improvement = ((measurementRMSE - pos_means(i)) / measurementRMSE) * 100;
            fprintf('\n%s improvements:\n', filterName);
            fprintf('  Position: %.1f%% (%.3fm → %.3fm)\n', ...
                pos_improvement, measurementRMSE, pos_means(i));
            
            if ~isnan(sog_means(i))
                sog_improvement = ((measurementSOGRMSE - sog_means(i)) / measurementSOGRMSE) * 100;
                fprintf('  SOG:      %.1f%% (%.3fm/s → %.3fm/s)\n', ...
                    sog_improvement, measurementSOGRMSE, sog_means(i));
            end
            
            if ~isnan(cog_means(i))
                cog_improvement = ((measurementCOGRMSE - cog_means(i)) / measurementCOGRMSE) * 100;
                fprintf('  COG:      %.1f%% (%.3f° → %.3f°)\n', ...
                    cog_improvement, measurementCOGRMSE, cog_means(i));
            end
        end
    end
end

% Print overall ranking
fprintf('\n===== OVERALL FILTER RANKING =====\n');
validFilters = ~isnan(pos_means);
if any(validFilters)
    % Normalize all metrics and calculate combined score
    valid_pos = pos_means(validFilters);
    valid_vel = vel_means(validFilters);
    valid_sog = sog_means(validFilters);
    valid_cog = cog_means(validFilters);
    valid_names = filterNames(validFilters);
    
    % Normalize to [0,1] where 0 is best (lowest RMSE)
    norm_pos = (valid_pos - min(valid_pos)) / (max(valid_pos) - min(valid_pos) + eps);
    norm_vel = (valid_vel - min(valid_vel)) / (max(valid_vel) - min(valid_vel) + eps);
    norm_sog = (valid_sog - min(valid_sog)) / (max(valid_sog) - min(valid_sog) + eps);
    norm_cog = (valid_cog - min(valid_cog)) / (max(valid_cog) - min(valid_cog) + eps);
    
    % Combined score (equal weights for all metrics)
    combined_scores = (norm_pos + norm_vel + norm_sog + norm_cog) / 4;
    
    % Sort by combined score (lower is better)
    [sorted_scores, sort_idx] = sort(combined_scores);
    
    for i = 1:length(sort_idx)
        idx = sort_idx(i);
        fprintf('%d. %s (Combined Score: %.3f)\n', i, valid_names{idx}, sorted_scores(i));
    end
end

fprintf('\n================================================================================\n');
fprintf('                          END OF DETAILED SUMMARY\n');
fprintf('================================================================================\n\n');
end