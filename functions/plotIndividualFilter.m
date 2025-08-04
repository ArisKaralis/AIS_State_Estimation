function plotIndividualFilter(data, estimates, filterTitle)
% PLOTINDIVIDUALFILTER - Enhanced visualization for individual filter results
% 
% Creates high-quality plots for a single filter with improved styling
% and better error visualization

figure;

% Sample indices
t = 1:height(data);

% Enhanced color scheme
measurement_color = [0.7, 0.7, 0.7];
truth_color = [0.0, 0.0, 0.0];
estimate_color = [0.8500, 0.3250, 0.0980];  % Orange
error_color = [0.6350, 0.0780, 0.1840];     % Dark Red

% 1. Trajectory plot
subplot(2, 3, 1);
scatter(data.x, data.y, 20, measurement_color, 'filled', 'MarkerFaceAlpha', 0.4, 'DisplayName', 'Measurements');
hold on;
plot(data.x_true, data.y_true, '-', 'Color', truth_color, 'LineWidth', 3, 'DisplayName', 'Ground Truth');
plot(estimates.x_est, estimates.y_est, '-', 'Color', estimate_color, 'LineWidth', 2, 'DisplayName', filterTitle);

xlabel('X Position (m)', 'FontSize', 12);
ylabel('Y Position (m)', 'FontSize', 12);
title('Trajectory', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
grid on;
axis equal;
set(gca, 'FontSize', 10);

% 2. Speed comparison
subplot(2, 3, 2);
plot(t, data.SOG, '.', 'Color', measurement_color, 'MarkerSize', 8, 'DisplayName', 'Measurements');
hold on;
plot(t, data.sog_true, '-', 'Color', truth_color, 'LineWidth', 3, 'DisplayName', 'Ground Truth');
plot(t, estimates.sog_est, '-', 'Color', estimate_color, 'LineWidth', 2, 'DisplayName', filterTitle);

% Add segment boundaries if available
if isfield(data, 'segment')
    segmentChanges = find(diff(data.segment) ~= 0);
    for i = 1:length(segmentChanges)
        xline(segmentChanges(i)+1, 'k--', 'Alpha', 0.5, 'HandleVisibility', 'off');
    end
end

xlabel('Sample Number', 'FontSize', 12);
ylabel('Speed (m/s)', 'FontSize', 12);
title('Speed Over Ground', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
grid on;
set(gca, 'FontSize', 10);

% 3. Course comparison with proper unwrapping
subplot(2, 3, 3);

% --- Fix for COG wrapping using MATLAB's unwrap function ---
% Convert all course data to radians first
cog_meas_rad = deg2rad(data.COG);
cog_true_rad = deg2rad(data.cog_true);
cog_est_rad = deg2rad(estimates.cog_est);

% Unwrap all angles to remove discontinuities
cog_meas_unwrapped = rad2deg(unwrap(cog_meas_rad));
cog_true_unwrapped = rad2deg(unwrap(cog_true_rad));
cog_est_unwrapped = rad2deg(unwrap(cog_est_rad));

% Plot with all unwrapped angles
plot(t, cog_meas_unwrapped, '.', 'Color', measurement_color, 'MarkerSize', 8, 'DisplayName', 'Measurements');
hold on;
plot(t, cog_true_unwrapped, '-', 'Color', truth_color, 'LineWidth', 3, 'DisplayName', 'Ground Truth');
plot(t, cog_est_unwrapped, '-', 'Color', estimate_color, 'LineWidth', 2, 'DisplayName', filterTitle);
% --- End of fix ---

% Add segment boundaries
if isfield(data, 'segment')
    for i = 1:length(segmentChanges)
        xline(segmentChanges(i)+1, 'k--', 'Alpha', 0.5, 'HandleVisibility', 'off');
    end
end

xlabel('Sample Number', 'FontSize', 12);
ylabel('Course (degrees)', 'FontSize', 12);
title('Course Over Ground', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
grid on;
set(gca, 'FontSize', 10);

% 4. Position error with log scale
subplot(2, 3, 4);
posError = sqrt((estimates.x_est - data.x_true).^2 + (estimates.y_est - data.y_true).^2);
semilogy(t, posError, '-', 'Color', error_color, 'LineWidth', 2);

% Add segment boundaries
if isfield(data, 'segment')
    for i = 1:length(segmentChanges)
        xline(segmentChanges(i)+1, 'k--', 'Alpha', 0.5, 'HandleVisibility', 'off');
    end
end

xlabel('Sample Number', 'FontSize', 12);
ylabel('Position Error (m)', 'FontSize', 12);
title('Position Error (Log Scale)', 'FontSize', 14, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 10);

% Add RMSE text
pos_rmse = sqrt(mean(posError.^2));
text(0.05, 0.95, sprintf('RMSE: %.2f m', pos_rmse), ...
     'Units', 'normalized', 'BackgroundColor', 'white', ...
     'EdgeColor', 'black', 'FontSize', 11, 'FontWeight', 'bold');

% 5. Speed error
subplot(2, 3, 5);
sogError = abs(estimates.sog_est - data.sog_true);
plot(t, sogError, '-', 'Color', error_color, 'LineWidth', 2);

% Add segment boundaries
if isfield(data, 'segment')
    for i = 1:length(segmentChanges)
        xline(segmentChanges(i)+1, 'k--', 'Alpha', 0.5, 'HandleVisibility', 'off');
    end
end

xlabel('Sample Number', 'FontSize', 12);
ylabel('Speed Error (m/s)', 'FontSize', 12);
title('Speed Error', 'FontSize', 14, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 10);

% Add RMSE text
sog_rmse = sqrt(mean(sogError.^2));
text(0.05, 0.95, sprintf('RMSE: %.3f m/s', sog_rmse), ...
     'Units', 'normalized', 'BackgroundColor', 'white', ...
     'EdgeColor', 'black', 'FontSize', 11, 'FontWeight', 'bold');

% 6. Course error
subplot(2, 3, 6);
cogError = abs(angdiff(deg2rad(estimates.cog_est), deg2rad(data.cog_true))) * 180/pi;
plot(t, cogError, '-', 'Color', error_color, 'LineWidth', 2);

% Add segment boundaries
if isfield(data, 'segment')
    for i = 1:length(segmentChanges)
        xline(segmentChanges(i)+1, 'k--', 'Alpha', 0.5, 'HandleVisibility', 'off');
    end
end

xlabel('Sample Number', 'FontSize', 12);
ylabel('Course Error (degrees)', 'FontSize', 12);
title('Course Error', 'FontSize', 14, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 10);

% Add RMSE text
cog_rmse = sqrt(mean(cogError.^2));
text(0.05, 0.95, sprintf('RMSE: %.2f deg', cog_rmse), ...
     'Units', 'normalized', 'BackgroundColor', 'white', ...
     'EdgeColor', 'black', 'FontSize', 11, 'FontWeight', 'bold');

% Overall title
sgtitle(sprintf('%s - Performance Analysis', filterTitle), 'FontSize', 16, 'FontWeight', 'bold');

% Create output directory if needed
if ~exist('figures', 'dir')
    mkdir('figures');
end

% Save figure
filter_name_clean = strrep(lower(filterTitle), ' ', '_');
filter_name_clean = strrep(filter_name_clean, '-', '_');
saveas(gcf, sprintf('figures/enhanced_%s_results.png', filter_name_clean));
saveas(gcf, sprintf('figures/enhanced_%s_results.fig', filter_name_clean));

fprintf('\n%s results saved to figures/%s_results.png\n', filterTitle, filter_name_clean);

end