function plotIMMAnalysis(estimates, data, stats)
    % PLOTIMMANALYSIS - Visualize IMM filter model probabilities and switches
    
    % Protect against plot.mlx conflicts
    plot_func = @(varargin) builtin('plot', varargin{:});
    
    time_hours = hours(estimates.time - estimates.time(1));
    n = length(estimates.time);
    
    % Create figure with multiple subplots
    figure('Name', 'IMM-EKF Analysis');
    
    % 1. Model Probabilities Over Time
    subplot(3, 2, 1);
    plot_func(time_hours, estimates.model_probs_cv * 100, 'b-', 'LineWidth', 2, 'DisplayName', 'EKF-CV');
    hold on;
    plot_func(time_hours, estimates.model_probs_ca * 100, 'r-', 'LineWidth', 2, 'DisplayName', 'EKF-CA');
    plot_func(time_hours, estimates.model_probs_ctrv * 100, 'g-', 'LineWidth', 2, 'DisplayName', 'EKF-CTRV');
    xlabel('Time (hours)');
    ylabel('Model Probability (%)');
    title('Model Probabilities Over Time');
    legend('Location', 'best');
    grid on;
    ylim([0, 100]);
    
    % 2. Dominant Model Over Time
    subplot(3, 2, 2);
    [~, dominant_models] = max([estimates.model_probs_cv, estimates.model_probs_ca, estimates.model_probs_ctrv], [], 2);
    
    % Create colored regions for dominant models
    model_colors = [0, 0.4470, 0.7410;     % Blue for CV
                   0.8500, 0.3250, 0.0980;  % Red for CA
                   0.4660, 0.6740, 0.1880]; % Green for CTRV
    
    for i = 1:n-1
        x_patch = [time_hours(i), time_hours(i+1), time_hours(i+1), time_hours(i)];
        y_patch = [0, 0, 1, 1];
        patch(x_patch, y_patch, model_colors(dominant_models(i), :), 'EdgeColor', 'none', 'FaceAlpha', 0.7);
        hold on;
    end
    
    xlabel('Time (hours)');
    ylabel('Dominant Model');
    title('Dominant Model Over Time');
    ylim([0, 1]);
    yticks([0.17, 0.5, 0.83]);
    yticklabels({'EKF-CV', 'EKF-CA', 'EKF-CTRV'});
    grid on;
    
    % Add legend
    legend_handles = [];
    legend_labels = {};
    if any(dominant_models == 1)
        legend_handles(end+1) = patch(NaN, NaN, model_colors(1, :), 'FaceAlpha', 0.7);
        legend_labels{end+1} = 'EKF-CV Dominant';
    end
    if any(dominant_models == 2)
        legend_handles(end+1) = patch(NaN, NaN, model_colors(2, :), 'FaceAlpha', 0.7);
        legend_labels{end+1} = 'EKF-CA Dominant';
    end
    if any(dominant_models == 3)
        legend_handles(end+1) = patch(NaN, NaN, model_colors(3, :), 'FaceAlpha', 0.7);
        legend_labels{end+1} = 'EKF-CTRV Dominant';
    end
    legend(legend_handles, legend_labels, 'Location', 'best');
    
    % 3. Model Switch Analysis
    subplot(3, 2, 3);
    switches = diff(dominant_models) ~= 0;
    switch_times = time_hours(2:end);
    switch_times = switch_times(switches);
    
    if ~isempty(switch_times)
        stem(switch_times, ones(size(switch_times)), 'filled', 'LineWidth', 2, 'MarkerSize', 8);
        xlabel('Time (hours)');
        ylabel('Model Switch');
        title(sprintf('Model Switches (Total: %d)', sum(switches)));
        grid on;
        ylim([0, 1.2]);
    else
        text(0.5, 0.5, 'No Model Switches', 'HorizontalAlignment', 'center', 'Units', 'normalized');
        title('Model Switches (Total: 0)');
    end
    
    % 4. Position Error with Model Indication
    subplot(3, 2, 4);
    pos_errors = sqrt((estimates.x - data.x_true).^2 + (estimates.y - data.y_true).^2);
    
    % Plot error with color coding by dominant model
    for model = 1:3
        mask = dominant_models == model;
        if model == 1
            model_n = 'CV';
        elseif model == 2
            model_n = 'CV';
        else
            model_n = 'CTRV';
        end
        
        if any(mask)
            plot_func(time_hours(mask), pos_errors(mask), 'o', 'Color', model_colors(model, :), ...
                     'MarkerSize', 4, 'DisplayName', model_n);
            hold on;
        end
    end
    
    xlabel('Time (hours)');
    ylabel('Position Error (m)');
    title(sprintf('Position Error by Dominant Model (RMSE: %.2fm)', stats.position_rmse));
    legend('Location', 'best');
    grid on;
    
    % 5. Model Probability Distribution
    subplot(3, 2, 5);
    model_names = {'EKF-CV', 'EKF-CA', 'EKF-CTRV'};
    mean_probs = [mean(estimates.model_probs_cv), mean(estimates.model_probs_ca), mean(estimates.model_probs_ctrv)] * 100;
    
    bar_colors = model_colors;
    bar_handle = bar(mean_probs, 'FaceColor', 'flat');
    bar_handle.CData = bar_colors;
    
    xlabel('Model');
    ylabel('Average Probability (%)');
    title('Average Model Probabilities');
    xticklabels(model_names);
    grid on;
    
    % Add percentage labels on bars
    for i = 1:3
        text(i, mean_probs(i) + 1, sprintf('%.1f%%', mean_probs(i)), ...
             'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    end
    
    % 6. Model Dominance Timeline
    subplot(3, 2, 6);
    
    % Calculate time spent in each model
    cv_time = sum(dominant_models == 1) / n * 100;
    ca_time = sum(dominant_models == 2) / n * 100;
    ctrv_time = sum(dominant_models == 3) / n * 100;
    
    dominance_data = [cv_time, ca_time, ctrv_time];
    pie_handle = pie(dominance_data);
    
    % Color the pie slices
    for i = 1:2:length(pie_handle)
        pie_handle(i).FaceColor = model_colors((i+1)/2, :);
    end
    
    title('Model Dominance Distribution');
    legend(model_names, 'Location', 'bestoutside');
    
    % Add overall statistics as text - FIXED THE SYNTAX ERROR HERE
    title_text = sprintf('IMM-EKF Performance Summary\nPosition RMSE: %.2fm | Velocity RMSE: %.2fm/s | SOG RMSE: %.2fm/s | COG RMSE: %.2f°', ...
                        stats.position_rmse, stats.velocity_rmse, stats.sog_rmse, stats.cog_rmse);
    sgtitle(title_text, 'FontSize', 14, 'FontWeight', 'bold');
    
    % Save the figure
    try
        saveas(gcf, 'figures/imm_ekf_analysis.png');
        saveas(gcf, 'figures/imm_ekf_analysis.fig');
        fprintf('IMM analysis plots saved to figures/\n');
    catch
        fprintf('Warning: Could not save figures\n');
    end
    
    % Print detailed model switching analysis
    fprintf('\n=== IMM-EKF Model Analysis ===\n');
    fprintf('Total simulation time: %.2f hours\n', time_hours(end));
    fprintf('Number of model switches: %d\n', sum(switches));
    if sum(switches) > 0
        fprintf('Average time between switches: %.2f minutes\n', ...
                (time_hours(end) * 60) / sum(switches));
    end
    
    fprintf('\nModel Usage Statistics:\n');
    fprintf('  EKF-CV:   %.1f%% average probability, %.1f%% dominant time\n', mean_probs(1), cv_time);
    fprintf('  EKF-CA:   %.1f%% average probability, %.1f%% dominant time\n', mean_probs(2), ca_time);
    fprintf('  EKF-CTRV: %.1f%% average probability, %.1f%% dominant time\n', mean_probs(3), ctrv_time);
    
    % Analyze model performance during dominance
    fprintf('\nPerformance During Model Dominance:\n');
    for model = 1:3
        mask = dominant_models == model;
        if any(mask)
            model_pos_rmse = sqrt(mean(pos_errors(mask).^2));
            fprintf('  %s dominant periods: %.2fm position RMSE\n', ...
                   model_names{model}, model_pos_rmse);
        end
    end
    
    % Model transition analysis
    if sum(switches) > 0
        fprintf('\nModel Transition Analysis:\n');
        transitions = zeros(3, 3);
        for i = 2:n
            if switches(i-1)
                from_model = dominant_models(i-1);
                to_model = dominant_models(i);
                transitions(from_model, to_model) = transitions(from_model, to_model) + 1;
            end
        end
        
        for from = 1:3
            for to = 1:3
                if transitions(from, to) > 0
                    fprintf('  %s -> %s: %d transitions\n', ...
                           model_names{from}, model_names{to}, transitions(from, to));
                end
            end
        end
    end
end