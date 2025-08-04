function plotIMMResults(data, estimates, title_prefix, model_names)
% PLOTIMMRESULTS - Plot IMM filter results and model probabilities
%
% Inputs:
%   data - Table with ground truth data
%   estimates - Struct with filter estimates including model probabilities
%   title_prefix - String prefix for plot titles (e.g., 'IMM-3', 'IMM-2')
%   model_names - Cell array of model names (e.g., {'CV', 'CA', 'CTRV'} or {'CV', 'CTRV'})
%
% The estimates struct should contain:
%   - Standard estimate fields: x_est, y_est, vx_est, vy_est, sog_est, cog_est
%   - Model probability fields: model_probs_<modelname> (e.g., model_probs_cv, model_probs_ca)

if nargin < 4
    % Default model names based on number of probability fields
    prob_fields = fieldnames(estimates);
    model_prob_fields = prob_fields(startsWith(prob_fields, 'model_probs_'));
    num_models = length(model_prob_fields);
    
    if num_models == 2
        model_names = {'CV', 'CTRV'};
    elseif num_models == 3
        model_names = {'CV', 'CA', 'CTRV'};
    else
        error('Unsupported number of models: %d', num_models);
    end
end

if nargin < 3
    title_prefix = sprintf('IMM-%d', length(model_names));
end

num_models = length(model_names);

% Extract model probabilities dynamically
model_probs = zeros(height(data), num_models);
for i = 1:num_models
    field_name = sprintf('model_probs_%s', lower(model_names{i}));
    if isfield(estimates, field_name)
        model_probs(:, i) = estimates.(field_name);
    else
        warning('Model probability field %s not found', field_name);
        model_probs(:, i) = zeros(height(data), 1);
    end
end

% Define colors for models (colorblind-friendly)
model_colors = [
    0.0000, 0.4470, 0.7410;  % Blue (CV)
    0.8500, 0.3250, 0.0980;  % Orange (CA)
    0.4660, 0.6740, 0.1880;  % Green (CTRV)
];

% Use subset of colors based on number of models
colors = model_colors(1:num_models, :);

% Create figure with model probabilities and dominance analysis
figure;

% Plot 1: Model probabilities over time
subplot(2, 2, 1);
hold on;
for i = 1:num_models
    plot(data.timestamp, model_probs(:, i), '-', 'Color', colors(i, :), ...
         'LineWidth', 2, 'DisplayName', sprintf('%s Model', model_names{i}));
end
xlabel('Time');
ylabel('Model Probability');
title(sprintf('%s Model Probabilities Over Time', title_prefix));
legend('Location', 'best');
grid on;
ylim([0, 1]);

% Plot 2: Dominant model over time
subplot(2, 2, 2);
[~, dominant_model] = max(model_probs, [], 2);
hold on;
for i = 1:num_models
    model_mask = dominant_model == i;
    if any(model_mask)
        scatter(data.timestamp(model_mask), i * ones(sum(model_mask), 1), ...
                20, colors(i, :), 'filled', 'DisplayName', sprintf('%s Dominant', model_names{i}));
    end
end
xlabel('Time');
ylabel('Dominant Model');
title(sprintf('%s Dominant Model Over Time', title_prefix));
yticks(1:num_models);
yticklabels(model_names);
ylim([0.5, num_models + 0.5]);
legend('Location', 'best');
grid on;

% Plot 3: Model probability distribution (box plot or histogram)
subplot(2, 2, 3);
if exist('boxplot', 'file') == 2
    % Use boxplot if available
    all_probs = [];
    group_labels = {};
    for i = 1:num_models
        all_probs = [all_probs; model_probs(:, i)];
        group_labels = [group_labels; repmat(model_names(i), height(data), 1)];
    end
    boxplot(all_probs, group_labels);
    ylabel('Model Probability');
    title(sprintf('%s Model Probability Distributions', title_prefix));
else
    % Fallback to histogram
    hold on;
    for i = 1:num_models
        histogram(model_probs(:, i), 20, 'FaceColor', colors(i, :), ...
                 'FaceAlpha', 0.7, 'DisplayName', sprintf('%s Model', model_names{i}));
    end
    xlabel('Model Probability');
    ylabel('Frequency');
    title(sprintf('%s Model Probability Distributions', title_prefix));
    legend('Location', 'best');
end
grid on;

% Plot 4: Model dominance statistics (pie chart or bar chart)
subplot(2, 2, 4);
dominance_pct = zeros(num_models, 1);
for i = 1:num_models
    dominance_pct(i) = sum(dominant_model == i) / length(dominant_model) * 100;
end

if exist('pie', 'file') == 2
    % Use pie chart if available
    pie(dominance_pct);
    title(sprintf('%s Model Dominance', title_prefix));
    % Create legend labels with percentages
    legend_labels = cell(num_models, 1);
    for i = 1:num_models
        legend_labels{i} = sprintf('%s: %.1f%%', model_names{i}, dominance_pct(i));
    end
    legend(legend_labels, 'Location', 'eastoutside');
else
    % Fallback to bar chart
    bar(1:num_models, dominance_pct, 'FaceColor', 'flat');
    colormap(colors);
    xlabel('Model');
    ylabel('Dominance (%)');
    title(sprintf('%s Model Dominance', title_prefix));
    set(gca, 'XTickLabel', model_names);
    grid on;
    
    % Add percentage labels on bars
    for i = 1:num_models
        text(i, dominance_pct(i) + 1, sprintf('%.1f%%', dominance_pct(i)), ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    end
end

% Add overall title
sgtitle(sprintf('%s Filter: Model Analysis', title_prefix), 'FontSize', 16, 'FontWeight', 'bold');

% Print summary statistics to console
fprintf('\n--- %s Model Selection Summary ---\n', title_prefix);
for i = 1:num_models
    avg_prob = mean(model_probs(:, i)) * 100;
    dominance = dominance_pct(i);
    fprintf('%s Model:   %.1f%% average probability, %.1f%% dominance\n', ...
            model_names{i}, avg_prob, dominance);
end

% Save figure
if ~exist('figures', 'dir')
    mkdir('figures');
end

filename_base = sprintf('figures/%s_analysis', lower(strrep(title_prefix, '-', '_')));
saveas(gcf, [filename_base '.png']);
saveas(gcf, [filename_base '.fig']);

fprintf('IMM analysis plots saved to %s.png/.fig\n', filename_base);

end