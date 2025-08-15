function generateFilterAnalysisReport(results)
% Generate separate figures for each analysis group, including NEES/NIS

filterNames = results.comparison.filterTypes;
numFilters = numel(filterNames);

% Calculate measurement statistics for reference lines
measurementStats = results.measurementStats;
if ~isempty(measurementStats.position_rmse)
    meas_pos_mean = mean(measurementStats.position_rmse);
    meas_pos_std = std(measurementStats.position_rmse);
    meas_pos_n = length(measurementStats.position_rmse);
    meas_pos_sem = meas_pos_std / sqrt(meas_pos_n);
    meas_pos_ci = tinv(0.975, meas_pos_n-1) * meas_pos_sem;
    
    meas_vel_mean = mean(measurementStats.velocity_rmse);
    meas_vel_std = std(measurementStats.velocity_rmse);
    meas_vel_sem = meas_vel_std / sqrt(meas_pos_n);
    meas_vel_ci = tinv(0.975, meas_pos_n-1) * meas_vel_sem;
    
    meas_sog_mean = mean(measurementStats.sog_rmse);
    meas_sog_std = std(measurementStats.sog_rmse);
    meas_sog_sem = meas_sog_std / sqrt(meas_pos_n);
    meas_sog_ci = tinv(0.975, meas_pos_n-1) * meas_sog_sem;
    
    meas_cog_mean = mean(measurementStats.cog_rmse);
    meas_cog_std = std(measurementStats.cog_rmse);
    meas_cog_sem = meas_cog_std / sqrt(meas_pos_n);
    meas_cog_ci = tinv(0.975, meas_pos_n-1) * meas_cog_sem;
else
    meas_pos_mean = NaN; meas_pos_ci = NaN;
    meas_vel_mean = NaN; meas_vel_ci = NaN;
    meas_sog_mean = NaN; meas_sog_ci = NaN;
    meas_cog_mean = NaN; meas_cog_ci = NaN;
end

% --- 1. RMSE Comparison Figure with Measurement References ---
figure;

subplot(2,2,1);
pos_means = results.comparison.position_rmse.means;
pos_stds = results.comparison.position_rmse.stds;
pos_counts = results.comparison.position_rmse.counts;
validIdx = ~isnan(pos_means);

% Calculate 95% confidence intervals for the mean
pos_cis = zeros(2, length(pos_means));
for i = 1:length(pos_means)
    if validIdx(i) && pos_counts(i) > 1
        sem = pos_stds(i) / sqrt(pos_counts(i));  % Standard error of the mean
        t_val = tinv(0.975, pos_counts(i)-1);    % t-value for 95% CI
        ci_half = t_val * sem;
        pos_cis(:,i) = [pos_means(i) - ci_half; pos_means(i) + ci_half];
    else
        pos_cis(:,i) = [NaN; NaN];
    end
end

% Plot filter results
errorbar(find(validIdx), pos_means(validIdx), ...
         pos_means(validIdx) - pos_cis(1,validIdx), pos_cis(2,validIdx) - pos_means(validIdx), ...
         'o-', 'LineWidth', 2, 'MarkerSize', 8, 'Color', [0.2 0.4 0.8], 'DisplayName', 'Filters');
hold on;

% Add measurement reference line with CI
if ~isnan(meas_pos_mean)
    yline(meas_pos_mean, 'r-', 'LineWidth', 3, 'DisplayName', sprintf('Raw Measurements (%.2f±%.2f m)', meas_pos_mean, meas_pos_ci));
    fill([0.5, numFilters+0.5, numFilters+0.5, 0.5], ...
         [meas_pos_mean-meas_pos_ci, meas_pos_mean-meas_pos_ci, meas_pos_mean+meas_pos_ci, meas_pos_mean+meas_pos_ci], ...
         'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', '95% CI');
end

set(gca, 'XTick', find(validIdx), 'XTickLabel', filterNames(validIdx), 'XTickLabelRotation', 45);
ylabel('Position RMSE (m)');
title('Position RMSE: Filters vs Raw Measurements');
legend('Location', 'best');
grid on;

subplot(2,2,2);
vel_means = results.comparison.velocity_rmse.means;
vel_stds = results.comparison.velocity_rmse.stds;
vel_counts = results.comparison.velocity_rmse.counts;
validIdx = ~isnan(vel_means);

% Calculate 95% confidence intervals for velocity
vel_cis = zeros(2, length(vel_means));
for i = 1:length(vel_means)
    if validIdx(i) && vel_counts(i) > 1
        sem = vel_stds(i) / sqrt(vel_counts(i));
        t_val = tinv(0.975, vel_counts(i)-1);
        ci_half = t_val * sem;
        vel_cis(:,i) = [vel_means(i) - ci_half; vel_means(i) + ci_half];
    else
        vel_cis(:,i) = [NaN; NaN];
    end
end

% Plot filter results
errorbar(find(validIdx), vel_means(validIdx), ...
         vel_means(validIdx) - vel_cis(1,validIdx), vel_cis(2,validIdx) - vel_means(validIdx), ...
         's-', 'LineWidth', 2, 'MarkerSize', 8, 'Color', [0.8 0.4 0.2], 'DisplayName', 'Filters');
hold on;

% Add measurement reference line with CI
if ~isnan(meas_vel_mean)
    yline(meas_vel_mean, 'r-', 'LineWidth', 3, 'DisplayName', sprintf('Raw Measurements (%.2f±%.2f m/s)', meas_vel_mean, meas_vel_ci));
    fill([0.5, numFilters+0.5, numFilters+0.5, 0.5], ...
         [meas_vel_mean-meas_vel_ci, meas_vel_mean-meas_vel_ci, meas_vel_mean+meas_vel_ci, meas_vel_mean+meas_vel_ci], ...
         'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', '95% CI');
end

set(gca, 'XTick', find(validIdx), 'XTickLabel', filterNames(validIdx), 'XTickLabelRotation', 45);
ylabel('Velocity RMSE (m/s)');
title('Velocity RMSE: Filters vs Raw Measurements');
legend('Location', 'best');
grid on;

subplot(2,2,3);
sog_means = results.comparison.sog_rmse.means;
sog_stds = results.comparison.sog_rmse.stds;
sog_counts = results.comparison.sog_rmse.counts;
validIdx = ~isnan(sog_means);

% Calculate 95% confidence intervals for SOG
sog_cis = zeros(2, length(sog_means));
for i = 1:length(sog_means)
    if validIdx(i) && sog_counts(i) > 1
        sem = sog_stds(i) / sqrt(sog_counts(i));
        t_val = tinv(0.975, sog_counts(i)-1);
        ci_half = t_val * sem;
        sog_cis(:,i) = [sog_means(i) - ci_half; sog_means(i) + ci_half];
    else
        sog_cis(:,i) = [NaN; NaN];
    end
end

% Plot filter results
errorbar(find(validIdx), sog_means(validIdx), ...
         sog_means(validIdx) - sog_cis(1,validIdx), sog_cis(2,validIdx) - sog_means(validIdx), ...
         '^-', 'LineWidth', 2, 'MarkerSize', 8, 'Color', [0.2 0.6 0.2], 'DisplayName', 'Filters');
hold on;

% Add measurement reference line with CI
if ~isnan(meas_sog_mean)
    yline(meas_sog_mean, 'r-', 'LineWidth', 3, 'DisplayName', sprintf('Raw Measurements (%.2f±%.2f m/s)', meas_sog_mean, meas_sog_ci));
    fill([0.5, numFilters+0.5, numFilters+0.5, 0.5], ...
         [meas_sog_mean-meas_sog_ci, meas_sog_mean-meas_sog_ci, meas_sog_mean+meas_sog_ci, meas_sog_mean+meas_sog_ci], ...
         'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', '95% CI');
end

set(gca, 'XTick', find(validIdx), 'XTickLabel', filterNames(validIdx), 'XTickLabelRotation', 45);
ylabel('SOG RMSE (m/s)');
title('SOG RMSE: Filters vs Raw Measurements');
legend('Location', 'best');
grid on;

subplot(2,2,4);
cog_means = results.comparison.cog_rmse.means;
cog_stds = results.comparison.cog_rmse.stds;
cog_counts = results.comparison.cog_rmse.counts;
validIdx = ~isnan(cog_means);

% Calculate 95% confidence intervals for COG
cog_cis = zeros(2, length(cog_means));
for i = 1:length(cog_means)
    if validIdx(i) && cog_counts(i) > 1
        sem = cog_stds(i) / sqrt(cog_counts(i));
        t_val = tinv(0.975, cog_counts(i)-1);
        ci_half = t_val * sem;
        cog_cis(:,i) = [cog_means(i) - ci_half; cog_means(i) + ci_half];
    else
        cog_cis(:,i) = [NaN; NaN];
    end
end

% Plot filter results
errorbar(find(validIdx), cog_means(validIdx), ...
         cog_means(validIdx) - cog_cis(1,validIdx), cog_cis(2,validIdx) - cog_means(validIdx), ...
         'd-', 'LineWidth', 2, 'MarkerSize', 8, 'Color', [0.6 0.2 0.6], 'DisplayName', 'Filters');
hold on;

% Add measurement reference line with CI
if ~isnan(meas_cog_mean)
    yline(meas_cog_mean, 'r-', 'LineWidth', 3, 'DisplayName', sprintf('Raw Measurements (%.1f±%.1f°)', meas_cog_mean, meas_cog_ci));
    fill([0.5, numFilters+0.5, numFilters+0.5, 0.5], ...
         [meas_cog_mean-meas_cog_ci, meas_cog_mean-meas_cog_ci, meas_cog_mean+meas_cog_ci, meas_cog_mean+meas_cog_ci], ...
         'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', '95% CI');
end

set(gca, 'XTick', find(validIdx), 'XTickLabel', filterNames(validIdx), 'XTickLabelRotation', 45);
ylabel('COG RMSE (°)');
title('COG RMSE: Filters vs Raw Measurements');
legend('Location', 'best');
grid on;

sgtitle(sprintf('RMSE Comparison: Filters vs Raw Measurements (%d runs)', results.metadata.numRuns));
saveas(gcf, 'figures/rmse_comparison.png');

% --- 2. Improvement Analysis Figure ---
figure;

% Calculate improvement percentages
improvement_data = zeros(numFilters, 4);
improvement_labels = {'Position', 'Velocity', 'SOG', 'COG'};
colors = [0.2 0.4 0.8; 0.8 0.4 0.2; 0.2 0.6 0.2; 0.6 0.2 0.6];

if ~isnan(meas_pos_mean)
    for i = 1:numFilters
        if ~isnan(pos_means(i))
            improvement_data(i, 1) = ((meas_pos_mean - pos_means(i)) / meas_pos_mean) * 100;
        end
        if ~isnan(vel_means(i))
            improvement_data(i, 2) = ((meas_vel_mean - vel_means(i)) / meas_vel_mean) * 100;
        end
        if ~isnan(sog_means(i))
            improvement_data(i, 3) = ((meas_sog_mean - sog_means(i)) / meas_sog_mean) * 100;
        end
        if ~isnan(cog_means(i))
            improvement_data(i, 4) = ((meas_cog_mean - cog_means(i)) / meas_cog_mean) * 100;
        end
    end
    
    % Plot improvement percentages
    h = bar(improvement_data, 'grouped');
    for i = 1:4
        h(i).FaceColor = colors(i, :);
    end
    
    % Add zero line for reference
    yline(0, 'k--', 'LineWidth', 2, 'DisplayName', 'No Improvement');
    
    xlabel('Filter Type');
    ylabel('Improvement over Raw Measurements (%)');
    title('Filter Performance Improvement over Raw Measurements');
    set(gca, 'XTick', 1:numFilters, 'XTickLabel', filterNames, 'XTickLabelRotation', 45);
    legend([h, yline(0, 'k--', 'HandleVisibility', 'off')], [improvement_labels, 'No Improvement'], 'Location', 'best');
    grid on;
    
    % Add value labels on bars
    for i = 1:numFilters
        for j = 1:4
            if ~isnan(improvement_data(i, j)) && improvement_data(i, j) ~= 0
                text(i + (j-2.5)*0.2, improvement_data(i, j) + sign(improvement_data(i, j))*2, ...
                     sprintf('%.1f%%', improvement_data(i, j)), ...
                     'HorizontalAlignment', 'center', 'FontSize', 8);
            end
        end
    end
end

saveas(gcf, 'figures/filter_improvement_analysis.png');

% --- 3. Computational Performance Figure ---
figure;
runtime_means = [];
runtime_cis = zeros(2, length(filterNames));
for i = 1:length(filterNames)
    filterName = filterNames{i};
    if isfield(results.filterStats.(filterName), 'summary') && isfield(results.filterStats.(filterName).summary, 'runtime')
        runtime_means(i) = results.filterStats.(filterName).summary.runtime.mean;
        runtime_std = results.filterStats.(filterName).summary.runtime.std;
        n_samples = length(results.filterStats.(filterName).runtime);
        
        if n_samples > 1
            sem = runtime_std / sqrt(n_samples);
            t_val = tinv(0.975, n_samples-1);
            ci_half = t_val * sem;
            runtime_cis(:,i) = [runtime_means(i) - ci_half; runtime_means(i) + ci_half];
        else
            runtime_cis(:,i) = [NaN; NaN];
        end
    else
        runtime_means(i) = NaN;
        runtime_cis(:,i) = [NaN; NaN];
    end
end
validIdx = ~isnan(runtime_means);
errorbar(find(validIdx), runtime_means(validIdx), ...
         runtime_means(validIdx) - runtime_cis(1,validIdx), ...
         runtime_cis(2,validIdx) - runtime_means(validIdx), ...
         'p-', 'LineWidth', 2, 'MarkerSize', 8);
set(gca, 'XTick', find(validIdx), 'XTickLabel', filterNames(validIdx), 'XTickLabelRotation', 45);
ylabel('Runtime (s)');
title('Computational Performance (95% CI)');
grid on;
saveas(gcf, 'figures/computational_performance.png');


% --- 4. Filter Performance Summary ---
figure;
axis off;
% Enhanced Filter Performance Summary formatting - DOUBLE COLUMN VERSION
summaryText = {
    '╔═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════╗';
    '║                                                    FILTER PERFORMANCE ANALYSIS SUMMARY                                      ║';
    '╚═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════╝';
    '';
};

% Add measurement baselines with better formatting
if ~isnan(meas_pos_mean)
    summaryText{end+1} = 'RAW MEASUREMENT BASELINE PERFORMANCE                    TOP PERFORMING FILTERS BY METRIC';
    summaryText{end+1} = '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━                 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━';
    summaryText{end+1} = sprintf('   Position RMSE: %6.2f ± %4.2f m                       Best Position:  %s', ...
        meas_pos_mean, meas_pos_ci, getFieldOrDefault(results.comparison, 'best_filters.position', 'N/A'));
    summaryText{end+1} = sprintf('   Velocity RMSE: %6.2f ± %4.2f m/s                     Best Velocity:  %s', ...
        meas_vel_mean, meas_vel_ci, getFieldOrDefault(results.comparison, 'best_filters.velocity', 'N/A'));
    summaryText{end+1} = sprintf('   SOG RMSE:      %6.2f ± %4.2f m/s                     Best SOG:       %s', ...
        meas_sog_mean, meas_sog_ci, getFieldOrDefault(results.comparison, 'best_filters.sog', 'N/A'));
    summaryText{end+1} = sprintf('   COG RMSE:      %6.2f ± %4.2f°                        Best COG:       %s', ...
        meas_cog_mean, meas_cog_ci, getFieldOrDefault(results.comparison, 'best_filters.cog', 'N/A'));
    summaryText{end+1} = '';
end

% Add detailed filter performance in two columns
summaryText{end+1} = 'DETAILED FILTER PERFORMANCE BREAKDOWN';
summaryText{end+1} = '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━';

% Get filter performance data
pos_means = results.comparison.position_rmse.means;
pos_stds = results.comparison.position_rmse.stds;
pos_counts = results.comparison.position_rmse.counts;
vel_means = results.comparison.velocity_rmse.means;
vel_stds = results.comparison.velocity_rmse.stds;
vel_counts = results.comparison.velocity_rmse.counts;

% Organize filters into two columns
leftColumn = {};
rightColumn = {};
numFiltersPerColumn = ceil(length(filterNames) / 2);

for i = 1:length(filterNames)
    filterName = filterNames{i};
    displayName = strrep(filterName, '_', '-');
    
    if ~isnan(pos_means(i))
        % Filter header with emoji
        filterHeader = sprintf('%s %s', upper(displayName));
        filterSeparator = repmat('─', 1, length(displayName) + 4);
        
        % Position performance
        if pos_counts(i) > 1 && ~isnan(meas_pos_mean)
            sem = pos_stds(i) / sqrt(pos_counts(i));
            ci_half = tinv(0.975, pos_counts(i)-1) * sem;
            improvement_pct = ((meas_pos_mean - pos_means(i)) / meas_pos_mean) * 100;
            posLine = sprintf('   Position: %6.2f [%5.2f-%5.2f] m   %s %s%5.1f%%', ...
                pos_means(i), pos_means(i) - ci_half, pos_means(i) + ci_half, ...
                getSign(improvement_pct), abs(improvement_pct));
        else
            posLine = sprintf('   Position: %6.2f ± %5.2f m', pos_means(i), pos_stds(i));
        end
        
        % Velocity performance
        if ~isnan(vel_means(i)) && vel_counts(i) > 1 && ~isnan(meas_vel_mean)
            sem = vel_stds(i) / sqrt(vel_counts(i));
            ci_half = tinv(0.975, vel_counts(i)-1) * sem;
            improvement_pct = ((meas_vel_mean - vel_means(i)) / meas_vel_mean) * 100;
            velLine = sprintf('   Velocity: %6.2f [%5.2f-%5.2f] m/s %s %s%5.1f%%', ...
                vel_means(i), vel_means(i) - ci_half, vel_means(i) + ci_half, ...
                getSign(improvement_pct), abs(improvement_pct));
        elseif ~isnan(vel_means(i))
            velLine = sprintf('   Velocity: %6.2f ± %5.2f m/s', vel_means(i), vel_stds(i));
        else
            velLine = '   Velocity: N/A';
        end
        
        % Add to appropriate column
        if i <= numFiltersPerColumn
            leftColumn{end+1} = filterHeader;
            leftColumn{end+1} = filterSeparator;
            leftColumn{end+1} = posLine;
            leftColumn{end+1} = velLine;
            leftColumn{end+1} = '';
        else
            rightColumn{end+1} = filterHeader;
            rightColumn{end+1} = filterSeparator;
            rightColumn{end+1} = posLine;
            rightColumn{end+1} = velLine;
            rightColumn{end+1} = '';
        end
    end
end

% Combine columns side by side
maxLines = max(length(leftColumn), length(rightColumn));
for i = 1:maxLines
    leftText = '';
    rightText = '';
    
    if i <= length(leftColumn)
        leftText = leftColumn{i};
    end
    if i <= length(rightColumn)
        rightText = rightColumn{i};
    end
    
    % Pad left column to consistent width (65 characters)
    leftPadded = sprintf('%-65s', leftText);
    combinedLine = sprintf('%s │ %s', leftPadded, rightText);
    summaryText{end+1} = combinedLine;
end

% Add summary statistics at the bottom
summaryText{end+1} = '';
summaryText{end+1} = 'ANALYSIS SUMMARY';
summaryText{end+1} = '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━';
summaryText{end+1} = sprintf('Monte Carlo Runs: %d                              Valid Runs: %d/%d (%.1f%%)                              Best Overall: %s', ...
    results.metadata.numRuns, sum(~isnan(pos_means)), length(filterNames), ...
    100 * sum(~isnan(pos_means)) / length(filterNames), strrep(results.comparison.best_filter, '_', '-'));

% Helper function to safely get nested fields
function value = getFieldOrDefault(struct, fieldPath, defaultValue)
    try
        fields = strsplit(fieldPath, '.');
        current = struct;
        for i = 1:length(fields)
            if isfield(current, fields{i})
                current = current.(fields{i});
            else
                value = defaultValue;
                return;
            end
        end
        if ischar(current)
            value = strrep(current, '_', '-');
        else
            value = current;
        end
    catch
        value = defaultValue;
    end
end



function status = getPerformanceStatus(improvement_pct)
    if improvement_pct > 15
        status = '🟢'; % Excellent
    elseif improvement_pct > 5
        status = '🟡'; % Good
    elseif improvement_pct > 0
        status = '🟠'; % Fair
    else
        status = '🔴'; % Poor
    end
end

function sign_str = getSign(value)
    if value >= 0
        sign_str = '+';
    else
        sign_str = '';
    end
end

text(0.01, 0.99, summaryText, 'Units', 'normalized', 'VerticalAlignment', 'top', ...
     'FontName', 'Courier New', 'FontSize', 10, 'Interpreter', 'none');
sgtitle('Enhanced Filter Performance Summary with Measurement Baselines', 'FontSize', 16, 'FontWeight', 'bold');
% --- 7. NEES/NIS Statistics with Proper Chi-Square Confidence Intervals ---
nees_means = zeros(1, numFilters);
nees_cis = zeros(2, numFilters);
nis_means = zeros(1, numFilters);
nis_cis = zeros(2, numFilters);

for i = 1:numFilters
    f = filterNames{i};
    
    % NEES statistics with chi-square confidence intervals
    if isfield(results.filterStats.(f), 'nees_all') && ~isempty(results.filterStats.(f).nees_all)
        vals = results.filterStats.(f).nees_all;
        valid_vals = vals(~isnan(vals));
        if ~isempty(valid_vals)
            nees_means(i) = mean(valid_vals);
            n = length(valid_vals);
            dof = 2; % Position-only NEES (x, y)
            
            % Chi-square confidence intervals for NEES
            chi2_lower = chi2inv(0.025, dof * n) / n;
            chi2_upper = chi2inv(0.975, dof * n) / n;
            nees_cis(:,i) = [chi2_lower; chi2_upper];
        else
            nees_means(i) = NaN;
            nees_cis(:,i) = [NaN; NaN];
        end
    else
        nees_means(i) = NaN;
        nees_cis(:,i) = [NaN; NaN];
    end
    
    % NIS statistics with chi-square confidence intervals
    if isfield(results.filterStats.(f), 'nis_all') && ~isempty(results.filterStats.(f).nis_all)
        vals = results.filterStats.(f).nis_all;
        valid_vals = vals(~isnan(vals));
        if ~isempty(valid_vals)
            nis_means(i) = mean(valid_vals);
            n = length(valid_vals);
            dof = 2; % Measurement dimension (x, y positions)
            
            % Chi-square confidence intervals for NIS
            chi2_lower = chi2inv(0.025, dof * n) / n;
            chi2_upper = chi2inv(0.975, dof * n) / n;
            nis_cis(:,i) = [chi2_lower; chi2_upper];
        else
            nis_means(i) = NaN;
            nis_cis(:,i) = [NaN; NaN];
        end
    else
        nis_means(i) = NaN;
        nis_cis(:,i) = [NaN; NaN];
    end
end

figure;
subplot(1,2,1);
validIdx = ~isnan(nees_means);
if any(validIdx)
    errorbar(find(validIdx), nees_means(validIdx), ...
             nees_means(validIdx) - nees_cis(1,validIdx), ...
             nees_cis(2,validIdx) - nees_means(validIdx), ...
             'o-', 'LineWidth', 2, 'MarkerSize', 8);
    hold on;
    
    % Add theoretical expected value and bounds
    yline(2, 'g--', 'Expected (χ² = 2)', 'LineWidth', 2);
    yline(chi2inv(0.025, 2), 'r--', '95% Lower Bound', 'LineWidth', 1);
    yline(chi2inv(0.975, 2), 'r--', '95% Upper Bound', 'LineWidth', 1);
end
set(gca, 'XTick', find(validIdx), 'XTickLabel', filterNames(validIdx), 'XTickLabelRotation', 45);
ylabel('NEES');
title('NEES with χ² Confidence Intervals');
grid on;

subplot(1,2,2);
validIdx = ~isnan(nis_means);
if any(validIdx)
    errorbar(find(validIdx), nis_means(validIdx), ...
             nis_means(validIdx) - nis_cis(1,validIdx), ...
             nis_cis(2,validIdx) - nis_means(validIdx), ...
             'o-', 'LineWidth', 2, 'MarkerSize', 8);
    hold on;
    
    % Add theoretical expected value and bounds
    yline(2, 'g--', 'Expected (χ² = 2)', 'LineWidth', 2);
    yline(chi2inv(0.025, 2), 'r--', '95% Lower Bound', 'LineWidth', 1);
    yline(chi2inv(0.975, 2), 'r--', '95% Upper Bound', 'LineWidth', 1);
end
set(gca, 'XTick', find(validIdx), 'XTickLabel', filterNames(validIdx), 'XTickLabelRotation', 45);
ylabel('NIS');
title('NIS with χ² Confidence Intervals');
grid on;

sgtitle('NEES and NIS Statistics with Chi-Square Confidence Intervals');
saveas(gcf, 'figures/nees_nis_statistics.png');



end