function plotIMMResults(data, estimates, stats, title_prefix, model_names)
% PLOTIMMRESULTS - Create comprehensive IMM filter analysis plots with NEES/NIS
% 
% Inputs:
%   data - Table with ground truth and measurement data
%   estimates - IMM filter estimates with model probabilities
%   stats - Filter statistics (optional)
%   title_prefix - String for plot titles (e.g., 'IMM-2', 'IMM-3')
%   model_names - Cell array of model names (e.g., {'CV', 'CA', 'CTRV'} or {'CV', 'CTRV'})

% Handle input arguments and ensure model_names is a cell array
if nargin < 5 || isempty(model_names)
    % Try to determine number of models from estimates fields
    if isfield(estimates, 'model_probs_ctrv')
        if isfield(estimates, 'model_probs_ca')
            model_names = {'CV', 'CA', 'CTRV'};
        else
            model_names = {'CV', 'CTRV'};
        end
    else
        model_names = {'CV', 'CTRV'};
    end
elseif ~iscell(model_names)
    % Convert string array to cell array if needed
    if isstring(model_names)
        model_names = cellstr(model_names);
    elseif ischar(model_names)
        model_names = {model_names};
    end
end

if nargin < 4 || isempty(title_prefix)
    title_prefix = sprintf('IMM-%d', length(model_names));
end

num_models = length(model_names);
colors = [0.0000, 0.4470, 0.7410; 0.8500, 0.3250, 0.0980; 0.9290, 0.6940, 0.1250; 0.4940, 0.1840, 0.5560];

% Check which model probability fields exist in estimates
prob_fields = cell(num_models, 1);
for i = 1:num_models
    field_name = sprintf('model_probs_%s', lower(model_names{i}));
    if isfield(estimates, field_name)
        prob_fields{i} = field_name;
    else
        % Try alternative field names
        alt_field_name = sprintf('%s_prob', lower(model_names{i}));
        if isfield(estimates, alt_field_name)
            prob_fields{i} = alt_field_name;
        else
            warning('Could not find probability field for model %s', model_names{i});
            prob_fields{i} = '';
        end
    end
end

% Create figure with 6 subplots (2x3 layout)
figure;

% Determine layout - use 6 subplots if we have NEES/NIS data
has_nees_nis = isfield(estimates, 'nees') && isfield(estimates, 'nis');
if has_nees_nis
    subplot_rows = 2;
    subplot_cols = 3;
else
    subplot_rows = 2;
    subplot_cols = 2;
end

% Plot 1: Model Probabilities Over Time
subplot(subplot_rows, subplot_cols, 1);
t = (0:height(data)-1)';

for i = 1:num_models
    if ~isempty(prob_fields{i}) && isfield(estimates, prob_fields{i})
        plot(t, estimates.(prob_fields{i}), '-', 'Color', colors(i, :), ...
             'LineWidth', 2, 'DisplayName', sprintf('%s Model', model_names{i}));
        hold on;
    end
end

% Add segment boundaries if available
if isfield(data, 'segment') && length(unique(data.segment)) > 1
    segmentChanges = find(diff(data.segment) ~= 0);
    for i = 1:length(segmentChanges)
        xline(segmentChanges(i)+1, 'k--', 'Alpha', 0.3, 'HandleVisibility', 'off');
    end
end

xlabel('Sample Number', 'FontSize', 12);
ylabel('Model Probability', 'FontSize', 12);
title(sprintf('%s Filter - Model Probabilities', title_prefix), 'FontSize', 14);
legend('Location', 'best', 'FontSize', 10);
grid on;
ylim([0, 1]);

% Plot 2: Dominant Model Over Time
subplot(subplot_rows, subplot_cols, 2);

% Create matrix of all model probabilities
prob_matrix = [];
for i = 1:num_models
    if ~isempty(prob_fields{i}) && isfield(estimates, prob_fields{i})
        prob_matrix(:, i) = estimates.(prob_fields{i});
    else
        prob_matrix(:, i) = zeros(height(data), 1);
    end
end

if ~isempty(prob_matrix)
    [~, dominant_model] = max(prob_matrix, [], 2);
    
    % Create scatter plot with different colors for each dominant model
    for i = 1:num_models
        idx = (dominant_model == i);
        if any(idx)
            scatter(t(idx), dominant_model(idx), ...
                    20, colors(i, :), 'filled', 'DisplayName', sprintf('%s Dominant', model_names{i}));
            hold on;
        end
    end
end

xlabel('Sample Number', 'FontSize', 12);
ylabel('Dominant Model', 'FontSize', 12);
title('Dominant Model Selection', 'FontSize', 14);
yticks(1:num_models);
yticklabels(model_names);
ylim([0.5, num_models + 0.5]);
legend('Location', 'best', 'FontSize', 10);
grid on;

% Plot 3: Trajectory Comparison
if has_nees_nis
    subplot(subplot_rows, subplot_cols, 3);
else
    subplot(subplot_rows, subplot_cols, 3);
end

plot(data.x, data.y, 'k.', 'MarkerSize', 6, 'DisplayName', 'Measurements');
hold on;
plot(data.x_true, data.y_true, 'g-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
plot(estimates.x_est, estimates.y_est, 'r-', 'LineWidth', 2, 'DisplayName', sprintf('%s Estimate', title_prefix));

xlabel('X Position (m)', 'FontSize', 12);
ylabel('Y Position (m)', 'FontSize', 12);
title('Trajectory Comparison', 'FontSize', 14);
legend('Location', 'best', 'FontSize', 10);
grid on;
axis equal;

% Plot 4: Position Errors
if has_nees_nis
    subplot(subplot_rows, subplot_cols, 4);
else
    subplot(subplot_rows, subplot_cols, 4);
end

pos_errors = sqrt((estimates.x_est - data.x_true).^2 + (estimates.y_est - data.y_true).^2);
plot(t, pos_errors, 'b-', 'LineWidth', 1.5);
hold on;

% Add segment boundaries
if isfield(data, 'segment') && length(unique(data.segment)) > 1
    for i = 1:length(segmentChanges)
        xline(segmentChanges(i)+1, 'k--', 'Alpha', 0.3, 'HandleVisibility', 'off');
    end
end

xlabel('Sample Number', 'FontSize', 12);
ylabel('Position Error (m)', 'FontSize', 12);
title('Position Error Over Time', 'FontSize', 14);
grid on;

% Only add NEES and NIS plots if data is available
if has_nees_nis
    % Plot 5: NEES (Normalized Estimation Error Squared)
    subplot(subplot_rows, subplot_cols, 5);
    
    valid_nees = ~isnan(estimates.nees);
    if any(valid_nees)
        plot(t(valid_nees), estimates.nees(valid_nees), 'b-', 'LineWidth', 1.5, 'DisplayName', 'NEES');
        hold on;
        
        % Chi-squared bounds for 4-DOF system (position + velocity)
        alpha = 0.05;  % 95% confidence
        dof = 4;
        lower_bound = chi2inv(alpha/2, dof);
        upper_bound = chi2inv(1-alpha/2, dof);
        
        yline(lower_bound, 'r--', 'Lower Bound (95%)', 'HandleVisibility', 'off');
        yline(upper_bound, 'r--', 'Upper Bound (95%)', 'HandleVisibility', 'off');
        yline(dof, 'g--', 'Expected Value', 'HandleVisibility', 'off');
        
        % Add segment boundaries
        if isfield(data, 'segment') && length(unique(data.segment)) > 1
            for i = 1:length(segmentChanges)
                xline(segmentChanges(i)+1, 'k--', 'Alpha', 0.3, 'HandleVisibility', 'off');
            end
        end
        
        xlabel('Sample Number', 'FontSize', 12);
        ylabel('NEES', 'FontSize', 12);
        title('Normalized Estimation Error Squared', 'FontSize', 14);
        grid on;
        
        % Calculate percentage within bounds
        within_bounds = sum(estimates.nees(valid_nees) >= lower_bound & estimates.nees(valid_nees) <= upper_bound);
        pct_within = (within_bounds / sum(valid_nees)) * 100;
        text(0.02, 0.98, sprintf('%.1f%% within 95%% bounds', pct_within), ...
             'Units', 'normalized', 'VerticalAlignment', 'top', 'FontSize', 10, ...
             'BackgroundColor', 'white', 'EdgeColor', 'black');
    end
    
    % Plot 6: NIS (Normalized Innovation Squared)
    subplot(subplot_rows, subplot_cols, 6);
    
    valid_nis = ~isnan(estimates.nis) & estimates.nis ~= 0;
    if any(valid_nis)
        plot(t(valid_nis), estimates.nis(valid_nis), 'r-', 'LineWidth', 1.5, 'DisplayName', 'NIS');
        hold on;
        
        % Chi-squared bounds for 2-DOF system (position measurements)
        dof_meas = 2;
        lower_bound_nis = chi2inv(alpha/2, dof_meas);
        upper_bound_nis = chi2inv(1-alpha/2, dof_meas);
        
        yline(lower_bound_nis, 'r--', 'Lower Bound (95%)', 'HandleVisibility', 'off');
        yline(upper_bound_nis, 'r--', 'Upper Bound (95%)', 'HandleVisibility', 'off');
        yline(dof_meas, 'g--', 'Expected Value', 'HandleVisibility', 'off');
        
        % Add segment boundaries
        if isfield(data, 'segment') && length(unique(data.segment)) > 1
            for i = 1:length(segmentChanges)
                xline(segmentChanges(i)+1, 'k--', 'Alpha', 0.3, 'HandleVisibility', 'off');
            end
        end
        
        xlabel('Sample Number', 'FontSize', 12);
        ylabel('NIS', 'FontSize', 12);
        title('Normalized Innovation Squared', 'FontSize', 14);
        grid on;
        
        % Calculate percentage within bounds
        within_bounds_nis = sum(estimates.nis(valid_nis) >= lower_bound_nis & estimates.nis(valid_nis) <= upper_bound_nis);
        pct_within_nis = (within_bounds_nis / sum(valid_nis)) * 100;
        text(0.02, 0.98, sprintf('%.1f%% within 95%% bounds', pct_within_nis), ...
             'Units', 'normalized', 'VerticalAlignment', 'top', 'FontSize', 10, ...
             'BackgroundColor', 'white', 'EdgeColor', 'black');
    end
end

% Add overall title
sgtitle(sprintf('%s Filter Analysis', title_prefix), 'FontSize', 16, 'FontWeight', 'bold');

% Calculate and display summary statistics
if num_models > 0 && ~isempty(prob_matrix)
    fprintf('\n=== %s Model Selection Summary ===\n', title_prefix);
    dominance_pct = zeros(num_models, 1);
    for i = 1:num_models
        avg_prob = mean(prob_matrix(:, i));
        dominance = sum(dominant_model == i) / length(dominant_model) * 100;
        dominance_pct(i) = dominance;
        fprintf('%s Model: Average Probability = %.3f, Dominant %.1f%% of time\n', ...
            model_names{i}, avg_prob, dominance);
    end
end

% Save the figure
if ~exist('figures', 'dir')
    mkdir('figures');
end

% Create filename
if isfield(data, 'segment')
    filename_base = sprintf('imm_analysis_%s_segment', lower(strrep(title_prefix, '-', '_')));
else
    filename_base = sprintf('imm_analysis_%s', lower(strrep(title_prefix, '-', '_')));
end

% Add timestamp for uniqueness
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
filename_png = fullfile('figures', sprintf('%s_%s.png', filename_base, timestamp));
filename_fig = fullfile('figures', sprintf('%s_%s.fig', filename_base, timestamp));

saveas(gcf, filename_png);
savefig(filename_fig);

fprintf('IMM analysis plots saved to:\n');
fprintf('  %s\n', filename_png);
fprintf('  %s\n', filename_fig);

% Display statistics summary if available
if nargin >= 3 && ~isempty(stats) && isstruct(stats)
    fprintf('\n=== %s Performance Statistics ===\n', title_prefix);
    if isfield(stats, 'position_rmse')
        fprintf('Position RMSE: %.2f m\n', stats.position_rmse);
    end
    if isfield(stats, 'velocity_rmse')
        fprintf('Velocity RMSE: %.2f m/s\n', stats.velocity_rmse);
    end
    if isfield(stats, 'sog_rmse')
        fprintf('SOG RMSE: %.2f m/s\n', stats.sog_rmse);
    end
    if isfield(stats, 'cog_rmse')
        fprintf('COG RMSE: %.2f deg\n', stats.cog_rmse);
    end
    
    % Display NEES/NIS statistics if available
    if isfield(stats, 'nees_mean')
        fprintf('NEES: Mean = %.3f, Median = %.3f\n', stats.nees_mean, ...
                stats.nees_median);
        fprintf('NIS:  Mean = %.3f, Median = %.3f\n', stats.nis_mean, ...
                stats.nis_median);
    end
end

end