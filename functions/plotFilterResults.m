function plotFilterResults(data, estimates, filterTitle)
% PLOTFILTERRESULTS - Create visualization of filter results

figure;

% Plot trajectory
subplot(2, 2, 1);
builtin('plot', data.x, data.y, 'k.', 'MarkerSize', 10, 'DisplayName', 'Measurements');
hold on;
builtin('plot', data.x_true, data.y_true, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth');
builtin('plot', estimates.x_est, estimates.y_est, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Filter');

xlabel('X (meters)');
ylabel('Y (meters)');
title('Trajectory');
legend('Location', 'best');
grid on;
axis equal;

% Plot speed
subplot(2, 2, 2);
% Sample indices
t = 1:height(data);

builtin('plot', t, data.SOG, 'k.', 'MarkerSize', 5, 'DisplayName', 'Measurements');
hold on;
builtin('plot', t, data.sog_true, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth');
builtin('plot', t, estimates.sog_est, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Filter');

% Add segment boundaries
segmentChanges = find(diff(data.segment) ~= 0);
for i = 1:length(segmentChanges)
    xline(segmentChanges(i)+1, 'k--', 'HandleVisibility', 'off');
end

xlabel('Sample number');
ylabel('Speed (m/s)');
title('Speed Over Ground');
legend('Location', 'best');
grid on;

% Plot course
subplot(2, 2, 3);
builtin('plot', t, data.COG, 'k.', 'MarkerSize', 5, 'DisplayName', 'Measurements');
hold on;
builtin('plot', t, data.cog_true, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth');
builtin('plot', t, estimates.cog_est, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Filter');

% Add segment boundaries
for i = 1:length(segmentChanges)
    xline(segmentChanges(i)+1, 'k--', 'HandleVisibility', 'off');
end

xlabel('Sample number');
ylabel('Course (degrees)');
title('Course Over Ground');
legend('Location', 'best');
grid on;

% Plot position error
subplot(2, 2, 4);
posError = sqrt((estimates.x_est - data.x_true).^2 + (estimates.y_est - data.y_true).^2);
builtin('plot', t, posError, 'b-', 'LineWidth', 1.5);

% Add segment boundaries
for i = 1:length(segmentChanges)
    xline(segmentChanges(i)+1, 'k--', 'HandleVisibility', 'off');
end


xlabel('Sample number');
ylabel('Error (m)');
title('Position Error');
grid on;

% Add overall title
sgtitle(filterTitle);

% Create output directory if needed
if ~exist('figures', 'dir')
    mkdir('figures');
end

% Save figure
filterName = strrep(filterTitle, ' ', '_');
filterName = strrep(filterName, '(', '');
filterName = strrep(filterName, ')', '');
filterName = strrep(filterName, '+', 'and');
filterName = lower(filterName);

saveas(gcf, sprintf('figures/%s_results.png', filterName));
saveas(gcf, sprintf('figures/%s_results.fig', filterName));

fprintf('Plots saved to figures/%s_results.png/.fig\n', filterName);

end