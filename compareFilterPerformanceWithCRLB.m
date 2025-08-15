function compareFilterPerformanceWithCRLB()
    % Compare filter performance against theoretical CRLB bounds
    % with comprehensive statistical analysis and visualization
    
    % Load CRLB results
    fprintf('Loading CRLB data...\n');
    crlb_file = 'output/crlb_results.mat';
    if exist(crlb_file, 'file')
        load(crlb_file, 'results_crlb');
        fprintf('Successfully loaded CRLB data from %s\n', crlb_file);
    else
        error('CRLB results file not found at %s. Run CRLB calculation first.', crlb_file);
    end
    
    % Load filter results from the most recent best_filters_results file
    fprintf('\nSearching for filter results...\n');
    output_files = dir('output/best_filters_results_*.mat');
    if isempty(output_files)
        error('No filter results files found in output/ folder');
    end
    
    % Get the most recent file
    [~, idx] = max([output_files.datenum]);
    filter_file = fullfile('output', output_files(idx).name);
    fprintf('Loading filter results from: %s\n', filter_file);
    
    loaded_data = load(filter_file);
    if ~isfield(loaded_data, 'results') || ~isfield(loaded_data.results, 'filterStats')
        error('Expected results.filterStats structure not found in filter results file');
    end
    
    results = loaded_data.results;
    fprintf('Successfully loaded filter data with %d filters\n', length(fieldnames(results.filterStats)));
    
    % Extract filter statistics
    fprintf('\nExtracting filter statistics...\n');
    filter_pos_stats = struct();
    filter_vel_stats = struct();
    
    filter_names = fieldnames(results.filterStats);
    for i = 1:length(filter_names)
        filter_name = filter_names{i};
        filter_data = results.filterStats.(filter_name);
        
        % Extract position statistics
        if isfield(filter_data, 'position_rmse') && ~isempty(filter_data.position_rmse)
            pos_data = filter_data.position_rmse;
            filter_pos_stats.(filter_name).count = length(pos_data);
            filter_pos_stats.(filter_name).mean = mean(pos_data);
            filter_pos_stats.(filter_name).std = std(pos_data);
        else
            filter_pos_stats.(filter_name).count = 0;
            filter_pos_stats.(filter_name).mean = NaN;
            filter_pos_stats.(filter_name).std = NaN;
        end
        
        % Extract velocity statistics
        if isfield(filter_data, 'velocity_rmse') && ~isempty(filter_data.velocity_rmse)
            vel_data = filter_data.velocity_rmse;
            filter_vel_stats.(filter_name).count = length(vel_data);
            filter_vel_stats.(filter_name).mean = mean(vel_data);
            filter_vel_stats.(filter_name).std = std(vel_data);
        else
            filter_vel_stats.(filter_name).count = 0;
            filter_vel_stats.(filter_name).mean = NaN;
            filter_vel_stats.(filter_name).std = NaN;
        end
    end
    
    % Define filter to CRLB model mapping
    filter_to_crlb = containers.Map();
    filter_to_crlb('KF') = 'cv';
    filter_to_crlb('EKF_CV') = 'cv';
    filter_to_crlb('EKF_CA') = 'ca';
    filter_to_crlb('EKF_CTRV') = 'ctrv';
    filter_to_crlb('UKF_CV') = 'cv';
    filter_to_crlb('UKF_CTRV') = 'ctrv';
    filter_to_crlb('IMM_2') = 'cv';
    filter_to_crlb('IMM_3') = 'cv';
    
    % Create much clearer visualization
    figure;
    
    % Define consistent colors
    crlb_color = [0.7, 0.7, 0.7];  % Light gray for CRLB bounds
    filter_colors = [
        0.2, 0.4, 0.8;   % Blue
        0.8, 0.2, 0.2;   % Red  
        0.2, 0.7, 0.3;   % Green
        1.0, 0.6, 0.0;   % Orange
        0.8, 0.2, 0.8;   % Magenta
        0.6, 0.4, 0.2;   % Brown
        0.0, 0.6, 0.6;   % Cyan
        0.4, 0.4, 0.4;   % Dark gray
    ];
    
    % Position RMSE comparison (subplot 1)
    subplot(2, 2, 1);
    hold on;
    
    x_pos = 1:length(filter_names);
    bar_width = 0.35;
    
    % Collect CRLB and filter data
    crlb_pos_means = zeros(size(filter_names));
    crlb_pos_stds = zeros(size(filter_names));
    filter_pos_means = zeros(size(filter_names));
    filter_pos_stds = zeros(size(filter_names));
    
    for i = 1:length(filter_names)
        filter_name = filter_names{i};
        
        % Get CRLB data
        if isKey(filter_to_crlb, filter_name)
            crlb_model = filter_to_crlb(filter_name);
            crlb_data = results_crlb.(crlb_model)(:, 1);
            crlb_pos_means(i) = mean(crlb_data);
            crlb_pos_stds(i) = std(crlb_data);
        end
        
        % Get filter data
        filter_pos_means(i) = filter_pos_stats.(filter_name).mean;
        filter_pos_stds(i) = filter_pos_stats.(filter_name).std;
    end
    
    % Plot CRLB bounds (left bars)
    b1 = bar(x_pos - bar_width/2, crlb_pos_means, bar_width, 'FaceColor', crlb_color, ...
        'EdgeColor', 'k', 'LineWidth', 1);
    
    % Plot filter results (right bars)
    b2 = bar(x_pos + bar_width/2, filter_pos_means, bar_width, 'FaceColor', 'flat', ...
        'EdgeColor', 'k', 'LineWidth', 1);
    
    % Color each filter bar differently
    for i = 1:length(filter_names)
        b2.CData(i,:) = filter_colors(mod(i-1, size(filter_colors,1))+1, :);
    end
    
    % Add error bars
    for i = 1:length(filter_names)
        if crlb_pos_means(i) > 0
            errorbar(i - bar_width/2, crlb_pos_means(i), crlb_pos_stds(i), 'k', 'LineWidth', 1);
        end
        if ~isnan(filter_pos_stds(i)) && filter_pos_stats.(filter_names{i}).count > 1
            n = filter_pos_stats.(filter_names{i}).count;
            t_val = tinv(0.975, n-1);
            margin = t_val * filter_pos_stds(i) / sqrt(n);
            errorbar(i + bar_width/2, filter_pos_means(i), margin, 'k', 'LineWidth', 1);
        end
    end
    
    xlabel('Filter');
    ylabel('Position RMSE (m)');
    title('Position RMSE: Filter Performance vs CRLB Theoretical Bounds');
    set(gca, 'XTick', x_pos, 'XTickLabel', filter_names, 'XTickLabelRotation', 45);
    legend([b1, b2], {'CRLB Bound', 'Filter Performance'}, 'Location', 'best');
    grid on;
    
    % Velocity RMSE comparison (subplot 2)
    subplot(2, 2, 2);
    hold on;
    
    % Collect CRLB and filter data for velocity
    crlb_vel_means = zeros(size(filter_names));
    crlb_vel_stds = zeros(size(filter_names));
    filter_vel_means = zeros(size(filter_names));
    filter_vel_stds = zeros(size(filter_names));
    
    for i = 1:length(filter_names)
        filter_name = filter_names{i};
        
        % Get CRLB data
        if isKey(filter_to_crlb, filter_name)
            crlb_model = filter_to_crlb(filter_name);
            crlb_data = results_crlb.(crlb_model)(:, 2);
            crlb_vel_means(i) = mean(crlb_data);
            crlb_vel_stds(i) = std(crlb_data);
        end
        
        % Get filter data
        filter_vel_means(i) = filter_vel_stats.(filter_name).mean;
        filter_vel_stds(i) = filter_vel_stats.(filter_name).std;
    end
    
    % Plot CRLB bounds (left bars)
    b3 = bar(x_pos - bar_width/2, crlb_vel_means, bar_width, 'FaceColor', crlb_color, ...
        'EdgeColor', 'k', 'LineWidth', 1);
    
    % Plot filter results (right bars)
    b4 = bar(x_pos + bar_width/2, filter_vel_means, bar_width, 'FaceColor', 'flat', ...
        'EdgeColor', 'k', 'LineWidth', 1);
    
    % Color each filter bar differently
    for i = 1:length(filter_names)
        b4.CData(i,:) = filter_colors(mod(i-1, size(filter_colors,1))+1, :);
    end
    
    % Add error bars
    for i = 1:length(filter_names)
        if crlb_vel_means(i) > 0
            errorbar(i - bar_width/2, crlb_vel_means(i), crlb_vel_stds(i), 'k', 'LineWidth', 1);
        end
        if ~isnan(filter_vel_stds(i)) && filter_vel_stats.(filter_names{i}).count > 1
            n = filter_vel_stats.(filter_names{i}).count;
            t_val = tinv(0.975, n-1);
            margin = t_val * filter_vel_stds(i) / sqrt(n);
            errorbar(i + bar_width/2, filter_vel_means(i), margin, 'k', 'LineWidth', 1);
        end
    end
    
    xlabel('Filter');
    ylabel('Velocity RMSE (m/s)');
    title('Velocity RMSE: Filter Performance vs CRLB Theoretical Bounds');
    set(gca, 'XTick', x_pos, 'XTickLabel', filter_names, 'XTickLabelRotation', 45);
    legend([b3, b4], {'CRLB Bound', 'Filter Performance'}, 'Location', 'best');
    grid on;
    
    % Performance ratio analysis (subplot 3)
    subplot(2, 2, 3);
    hold on;
    
    pos_ratios = filter_pos_means ./ crlb_pos_means;
    vel_ratios = filter_vel_means ./ crlb_vel_means;
    
    % Remove NaN and Inf ratios
    valid_pos = ~isnan(pos_ratios) & ~isinf(pos_ratios) & (crlb_pos_means > 0);
    valid_vel = ~isnan(vel_ratios) & ~isinf(vel_ratios) & (crlb_vel_means > 0);
    
    % Only plot if we have valid data
    if any(valid_pos)
        b5 = bar(x_pos(valid_pos) - bar_width/2, pos_ratios(valid_pos), bar_width, ...
            'FaceColor', [0.3, 0.6, 0.9], 'EdgeColor', 'k');
    end
    
    if any(valid_vel)
        b6 = bar(x_pos(valid_vel) + bar_width/2, vel_ratios(valid_vel), bar_width, ...
            'FaceColor', [0.9, 0.4, 0.3], 'EdgeColor', 'k');
    end
    
    % Add reference line at ratio = 1 (theoretical minimum)
    line([0.5, length(filter_names)+0.5], [1, 1], 'Color', 'k', 'LineStyle', '--', 'LineWidth', 2);
    text(length(filter_names)/2, 1.05, 'CRLB Theoretical Minimum', ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    
    xlabel('Filter');
    ylabel('Performance Ratio (Filter RMSE / CRLB)');
    title('Filter Efficiency: How Close to Theoretical Optimum?');
    set(gca, 'XTick', x_pos, 'XTickLabel', filter_names, 'XTickLabelRotation', 45);
    
    % Set up legend based on what was plotted
    legend_handles = [];
    legend_labels = {};
    if any(valid_pos) && exist('b5', 'var')
        legend_handles(end+1) = b5;
        legend_labels{end+1} = 'Position Ratio';
    end
    if any(valid_vel) && exist('b6', 'var')
        legend_handles(end+1) = b6;
        legend_labels{end+1} = 'Velocity Ratio';
    end
    if ~isempty(legend_handles)
        legend(legend_handles, legend_labels, 'Location', 'best');
    end
    
    grid on;
    
    % Calculate ylim safely - collect all valid ratio values as scalars
    fprintf('Debug: Calculating ylim for ratios...\n');
    all_valid_ratios = [];
    
    if any(valid_pos)
        valid_pos_ratios = pos_ratios(valid_pos);
        fprintf('Valid position ratios: min=%.3f, max=%.3f\n', min(valid_pos_ratios), max(valid_pos_ratios));
        all_valid_ratios = [all_valid_ratios; valid_pos_ratios(:)];
    end
    
    if any(valid_vel)
        valid_vel_ratios = vel_ratios(valid_vel);
        fprintf('Valid velocity ratios: min=%.3f, max=%.3f\n', min(valid_vel_ratios), max(valid_vel_ratios));
        all_valid_ratios = [all_valid_ratios; valid_vel_ratios(:)];
    end
    
    if ~isempty(all_valid_ratios)
        min_ratio = min(all_valid_ratios);
        max_ratio = max(all_valid_ratios);
        fprintf('Overall: min_ratio=%.3f, max_ratio=%.3f\n', min_ratio, max_ratio);
        
        % Calculate safe limits
        y_min = min(0.5, min_ratio * 0.9);
        y_max = max_ratio * 1.1;
        
        fprintf('Calculated ylim: [%.3f, %.3f]\n', y_min, y_max);
        
        % Validate the limits are sensible
        if y_min < y_max && isfinite(y_min) && isfinite(y_max) && y_min >= 0
            ylim([y_min, y_max]);
            fprintf('Applied ylim successfully\n');
        else
            fprintf('Invalid ylim values, using default\n');
            ylim([0.5, 2.5]);
        end
    else
        fprintf('No valid ratios found, using default ylim\n');
        ylim([0.5, 2.5]);
    end
    
    % Summary table (subplot 4)
    subplot(2, 2, 4);
    axis off;
    
    % Create summary text
    summary_text = {'=== CRLB VALIDATION SUMMARY ==='; ''};
    
    % Add CRLB bounds
    models = {'cv', 'ca', 'ctrv'};
    model_names = {'CV (Constant Velocity)', 'CA (Constant Acceleration)', 'CTRV (Constant Turn Rate)'};
    
    summary_text{end+1} = 'THEORETICAL BOUNDS (Mean ± 95% CI):';
    for i = 1:length(models)
        model = models{i};
        model_name = model_names{i};
        
        pos_data = results_crlb.(model)(:, 1);
        vel_data = results_crlb.(model)(:, 2);
        
        n = length(pos_data);
        t_val = tinv(0.975, n-1);
        
        pos_mean = mean(pos_data);
        pos_ci = t_val * std(pos_data) / sqrt(n);
        vel_mean = mean(vel_data);
        vel_ci = t_val * std(vel_data) / sqrt(n);
        
        summary_text{end+1} = sprintf('  %s:', model_name);
        summary_text{end+1} = sprintf('    Pos: %.3f±%.3f m, Vel: %.3f±%.3f m/s', pos_mean, pos_ci, vel_mean, vel_ci);
    end
    
    summary_text{end+1} = '';
    summary_text{end+1} = 'FILTER PERFORMANCE RATIOS:';
    for i = 1:length(filter_names)
        filter_name = filter_names{i};
        if isKey(filter_to_crlb, filter_name) && valid_pos(i) && valid_vel(i)
            crlb_model = filter_to_crlb(filter_name);
            summary_text{end+1} = sprintf('  %s (vs %s):', filter_name, upper(crlb_model));
            summary_text{end+1} = sprintf('    Pos: %.2fx, Vel: %.2fx', pos_ratios(i), vel_ratios(i));
            
            if pos_ratios(i) < 1 || vel_ratios(i) < 1
                summary_text{end+1} = '    *** POTENTIAL VIOLATION ***';
            end
        end
    end
    
    % Display summary
    text(0.05, 0.95, summary_text, 'VerticalAlignment', 'top', 'FontName', 'monospace', ...
        'FontSize', 9, 'Units', 'normalized');
    
    % Save results
    save('figures/comprehensive_crlb_validation.mat', 'filter_pos_stats', 'filter_vel_stats', ...
         'results_crlb', 'filter_to_crlb', 'pos_ratios', 'vel_ratios');
    saveas(gcf, 'figures/comprehensive_crlb_validation.png');
    savefig('figures/comprehensive_crlb_validation.fig');
    
    fprintf('\nValidation complete! Results saved to figures/\n');
end