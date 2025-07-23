function compareFilterPerformance(data, kf_est, ekf_est_cv, ekf_est_ca, ukf_est, imm_est)
% COMPAREFILTERPERFORMANCE - Compare and visualize performance across filters

% Create figure for comparison
figure;

% Sample indices
t = 1:height(data);

% Plot trajectory comparison
subplot(2, 2, 1);
builtin('plot', data.x, data.y, 'k.', 'MarkerSize', 4, 'DisplayName', 'Measurements');
hold on;
builtin('plot', data.x_true, data.y_true, 'k-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
builtin('plot', kf_est.x_est, kf_est.y_est, 'b-', 'LineWidth', 1.5, 'DisplayName', 'KF');
builtin('plot', ekf_est_cv.x_est, ekf_est_cv.y_est, 'r-', 'LineWidth', 1.5, 'DisplayName', 'EKF-CV');
builtin('plot', ekf_est_ca.x_est, ekf_est_ca.y_est, 'r-', 'LineWidth', 1.5, 'DisplayName', 'EKF-CA');
builtin('plot', ukf_est.x_est, ukf_est.y_est, 'g-', 'LineWidth', 1.5, 'DisplayName', 'UKF');
builtin('plot', imm_est.x_est, imm_est.y_est, 'm-', 'LineWidth', 1.5, 'DisplayName', 'IMM');

xlabel('X (meters)');
ylabel('Y (meters)');
title('Trajectory Comparison');
legend('Location', 'best');
grid on;
axis equal;

% Plot position error comparison
subplot(2, 2, 2);
pos_err_kf = sqrt((kf_est.x_est - data.x_true).^2 + (kf_est.y_est - data.y_true).^2);
pos_err_ekf_cv = sqrt((ekf_est_cv.x_est - data.x_true).^2 + (ekf_est_cv.y_est - data.y_true).^2);
pos_err_ekf_ca = sqrt((ekf_est_ca.x_est - data.x_true).^2 + (ekf_est_ca.y_est - data.y_true).^2);
pos_err_ukf = sqrt((ukf_est.x_est - data.x_true).^2 + (ukf_est.y_est - data.y_true).^2);
pos_err_imm = sqrt((imm_est.x_est - data.x_true).^2 + (imm_est.y_est - data.y_true).^2);

builtin('plot', t, pos_err_kf, 'b-', 'LineWidth', 1.5, 'DisplayName', 'KF');
hold on;
builtin('plot', t, pos_err_ekf_cv, 'r-', 'LineWidth', 1.5, 'DisplayName', 'EKF-CV');
builtin('plot', t, pos_err_ekf_ca, 'r-', 'LineWidth', 1.5, 'DisplayName', 'EKF-CA');
builtin('plot', t, pos_err_ukf, 'g-', 'LineWidth', 1.5, 'DisplayName', 'UKF');
builtin('plot', t, pos_err_imm, 'm-', 'LineWidth', 1.5, 'DisplayName', 'IMM');

% Add segment boundaries
segmentChanges = find(diff(data.segment) ~= 0);
for i = 1:length(segmentChanges)
    xline(t(segmentChanges(i)+1), 'k--', 'HandleVisibility', 'off');
end

xlabel('Samples');
ylabel('Error (m)');
title('Position Error Comparison');
legend('Location', 'best');
grid on;

% Plot speed comparison
subplot(2, 2, 3);
builtin('plot', t, data.SOG, 'k.', 'MarkerSize', 4, 'DisplayName', 'Measurements');
hold on;
builtin('plot', t, data.sog_true, 'k-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
builtin('plot', t, kf_est.sog_est, 'b-', 'LineWidth', 1.5, 'DisplayName', 'KF');
builtin('plot', t, ekf_est_cv.sog_est, 'r-', 'LineWidth', 1.5, 'DisplayName', 'EKF-CV');
builtin('plot', t, ekf_est_ca.sog_est, 'r-', 'LineWidth', 1.5, 'DisplayName', 'EKF-CA');
builtin('plot', t, ukf_est.sog_est, 'g-', 'LineWidth', 1.5, 'DisplayName', 'UKF');
builtin('plot', t, imm_est.sog_est, 'm-', 'LineWidth', 1.5, 'DisplayName', 'IMM');

% Add segment boundaries
for i = 1:length(segmentChanges)
    xline(t(segmentChanges(i)+1), 'k--', 'HandleVisibility', 'off');
end

xlabel('Samples');
ylabel('Speed (m/s)');
title('Speed Over Ground Comparison');
legend('Location', 'best');
grid on;

% Plot course comparison
subplot(2, 2, 4);
builtin('plot', t, data.COG, 'k.', 'MarkerSize', 4, 'DisplayName', 'Measurements');
hold on;
builtin('plot', t, data.cog_true, 'k-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
builtin('plot', t, kf_est.cog_est, 'b-', 'LineWidth', 1.5, 'DisplayName', 'KF');
builtin('plot', t, ekf_est_cv.cog_est, 'r-', 'LineWidth', 1.5, 'DisplayName', 'EKF-CV');
builtin('plot', t, ekf_est_ca.cog_est, 'r-', 'LineWidth', 1.5, 'DisplayName', 'EKF-CA');
builtin('plot', t, ukf_est.cog_est, 'g-', 'LineWidth', 1.5, 'DisplayName', 'UKF');
builtin('plot', t, imm_est.cog_est, 'm-', 'LineWidth', 1.5, 'DisplayName', 'IMM');

% Add segment boundaries
for i = 1:length(segmentChanges)
    xline(t(segmentChanges(i)+1), 'k--', 'HandleVisibility', 'off');
end

xlabel('Samples');
ylabel('Course (degrees)');
title('Course Over Ground Comparison');
legend('Location', 'best');
grid on;

sgtitle('Filter Performance Comparison');

% Create output directory if needed
if ~exist('figures', 'dir')
    mkdir('figures');
end

% Save figure
saveas(gcf, 'figures/filter_comparison.png');
saveas(gcf, 'figures/filter_comparison.fig');

% Create a summary table for overall performance
fprintf('\n===== Overall Filter Performance Summary =====\n');
fprintf('%-10s | %-10s | %-10s | %-10s | %-10s\n', ...
        'Filter', 'Pos RMSE', 'Vel RMSE', 'SOG RMSE', 'COG RMSE');
fprintf('-------------------------------------------------------------\n');

% Calculate overall RMSE for each filter
kf_pos_rmse = sqrt(mean(pos_err_kf.^2));
ekf_cv_pos_rmse = sqrt(mean(pos_err_ekf_cv.^2));
ekf_ca_pos_rmse = sqrt(mean(pos_err_ekf_ca.^2));
ukf_pos_rmse = sqrt(mean(pos_err_ukf.^2));
imm_pos_rmse = sqrt(mean(pos_err_imm.^2));

kf_vel_rmse = sqrt(mean((sqrt((kf_est.vx_est - data.vx_true).^2 + ...
                             (kf_est.vy_est - data.vy_true).^2)).^2));
ekf_cv_vel_rmse = sqrt(mean((sqrt((ekf_est_cv.vx_est - data.vx_true).^2 + ...
                              (ekf_est_cv.vy_est - data.vy_true).^2)).^2));
ekf_ca_vel_rmse = sqrt(mean((sqrt((ekf_est_ca.vx_est - data.vx_true).^2 + ...
                              (ekf_est_ca.vy_est - data.vy_true).^2)).^2));
ukf_vel_rmse = sqrt(mean((sqrt((ukf_est.vx_est - data.vx_true).^2 + ...
                              (ukf_est.vy_est - data.vy_true).^2)).^2));
imm_vel_rmse = sqrt(mean((sqrt((imm_est.vx_est - data.vx_true).^2 + ...
                              (imm_est.vy_est - data.vy_true).^2)).^2));

kf_sog_rmse = sqrt(mean((kf_est.sog_est - data.sog_true).^2));
ekf_cv_sog_rmse = sqrt(mean((ekf_est_cv.sog_est - data.sog_true).^2));
ekf_ca_sog_rmse = sqrt(mean((ekf_est_ca.sog_est - data.sog_true).^2));
ukf_sog_rmse = sqrt(mean((ukf_est.sog_est - data.sog_true).^2));
imm_sog_rmse = sqrt(mean((imm_est.sog_est - data.sog_true).^2));

kf_cog_rmse = sqrt(mean((angdiff(deg2rad(kf_est.cog_est), deg2rad(data.cog_true)) * 180/pi).^2));
ekf_cv_cog_rmse = sqrt(mean((angdiff(deg2rad(ekf_est_cv.cog_est), deg2rad(data.cog_true)) * 180/pi).^2));
ekf_ca_cog_rmse = sqrt(mean((angdiff(deg2rad(ekf_est_ca.cog_est), deg2rad(data.cog_true)) * 180/pi).^2));
ukf_cog_rmse = sqrt(mean((angdiff(deg2rad(ukf_est.cog_est), deg2rad(data.cog_true)) * 180/pi).^2));
imm_cog_rmse = sqrt(mean((angdiff(deg2rad(imm_est.cog_est), deg2rad(data.cog_true)) * 180/pi).^2));

% Display results
fprintf('%-10s | %-10.2f | %-10.2f | %-10.2f | %-10.2f\n', ...
        'KF', kf_pos_rmse, kf_vel_rmse, kf_sog_rmse, kf_cog_rmse);
fprintf('%-10s | %-10.2f | %-10.2f | %-10.2f | %-10.2f\n', ...
        'EKF-CV', ekf_cv_pos_rmse, ekf_cv_vel_rmse, ekf_cv_sog_rmse, ekf_cv_cog_rmse);
fprintf('%-10s | %-10.2f | %-10.2f | %-10.2f | %-10.2f\n', ...
        'EKF-CA', ekf_ca_pos_rmse, ekf_ca_vel_rmse, ekf_ca_sog_rmse, ekf_ca_cog_rmse);
fprintf('%-10s | %-10.2f | %-10.2f | %-10.2f | %-10.2f\n', ...
        'UKF', ukf_pos_rmse, ukf_vel_rmse, ukf_sog_rmse, ukf_cog_rmse);
fprintf('%-10s | %-10.2f | %-10.2f | %-10.2f | %-10.2f\n', ...
        'IMM', imm_pos_rmse, imm_vel_rmse, imm_sog_rmse, imm_cog_rmse);

% ===== OVERALL BEST FILTER ANALYSIS =====
fprintf('\n===== Overall Best Filter Analysis =====\n');

% Filter names and corresponding RMSE values
filter_names = {'KF', 'EKF-CV', 'EKF-CA', 'UKF', 'IMM'};
pos_rmse_values = [kf_pos_rmse, ekf_cv_pos_rmse, ekf_ca_pos_rmse, ukf_pos_rmse, imm_pos_rmse];
vel_rmse_values = [kf_vel_rmse, ekf_cv_vel_rmse, ekf_ca_vel_rmse, ukf_vel_rmse, imm_vel_rmse];
sog_rmse_values = [kf_sog_rmse, ekf_cv_sog_rmse, ekf_ca_sog_rmse, ukf_sog_rmse, imm_sog_rmse];
cog_rmse_values = [kf_cog_rmse, ekf_cv_cog_rmse, ekf_ca_cog_rmse, ukf_cog_rmse, imm_cog_rmse];

% Method 1: Best in each category
[~, best_pos_idx] = min(pos_rmse_values);
[~, best_vel_idx] = min(vel_rmse_values);
[~, best_sog_idx] = min(sog_rmse_values);
[~, best_cog_idx] = min(cog_rmse_values);

fprintf('\n--- Best Filter by Individual Metric ---\n');
fprintf('Best Position RMSE: %s (%.2f m)\n', filter_names{best_pos_idx}, pos_rmse_values(best_pos_idx));
fprintf('Best Velocity RMSE: %s (%.2f m/s)\n', filter_names{best_vel_idx}, vel_rmse_values(best_vel_idx));
fprintf('Best SOG RMSE: %s (%.2f m/s)\n', filter_names{best_sog_idx}, sog_rmse_values(best_sog_idx));
fprintf('Best COG RMSE: %s (%.2f deg)\n', filter_names{best_cog_idx}, cog_rmse_values(best_cog_idx));

% Method 2: Weighted composite score
% Define weights (you can adjust these based on importance)
w_pos = 0.4;  % Position is most important for navigation
w_vel = 0.2;  % Velocity estimation
w_sog = 0.2;  % Speed over ground  
w_cog = 0.2;  % Course over ground

% Normalize each metric (0-1 scale where 0 is best, 1 is worst)
pos_norm = (pos_rmse_values - min(pos_rmse_values)) ./ (max(pos_rmse_values) - min(pos_rmse_values));
vel_norm = (vel_rmse_values - min(vel_rmse_values)) ./ (max(vel_rmse_values) - min(vel_rmse_values));
sog_norm = (sog_rmse_values - min(sog_rmse_values)) ./ (max(sog_rmse_values) - min(sog_rmse_values));
cog_norm = (cog_rmse_values - min(cog_rmse_values)) ./ (max(cog_rmse_values) - min(cog_rmse_values));

% Handle edge cases where min = max (all filters perform identically)
pos_norm(isnan(pos_norm)) = 0;
vel_norm(isnan(vel_norm)) = 0;
sog_norm(isnan(sog_norm)) = 0;
cog_norm(isnan(cog_norm)) = 0;

% Calculate weighted composite scores (lower is better)
composite_scores = w_pos * pos_norm + w_vel * vel_norm + w_sog * sog_norm + w_cog * cog_norm;
[~, best_overall_idx] = min(composite_scores);

fprintf('\n--- Weighted Composite Score Analysis ---\n');
fprintf('Weights: Position=%.1f, Velocity=%.1f, SOG=%.1f, COG=%.1f\n', w_pos, w_vel, w_sog, w_cog);
fprintf('Composite Scores (lower is better):\n');
for i = 1:length(filter_names)
    fprintf('  %s: %.3f\n', filter_names{i}, composite_scores(i));
end

% Method 3: Simple average rank
pos_ranks = tiedrank(pos_rmse_values);
vel_ranks = tiedrank(vel_rmse_values);
sog_ranks = tiedrank(sog_rmse_values);
cog_ranks = tiedrank(cog_rmse_values);

avg_ranks = (pos_ranks + vel_ranks + sog_ranks + cog_ranks) / 4;
[~, best_rank_idx] = min(avg_ranks);

fprintf('\n--- Average Rank Analysis ---\n');
fprintf('Average Ranks (lower is better):\n');
for i = 1:length(filter_names)
    fprintf('  %s: %.2f (Pos:%d, Vel:%d, SOG:%d, COG:%d)\n', ...
        filter_names{i}, avg_ranks(i), pos_ranks(i), vel_ranks(i), sog_ranks(i), cog_ranks(i));
end

% Method 4: Count of "wins" (best performance in individual categories)
win_counts = zeros(1, length(filter_names));
win_counts(best_pos_idx) = win_counts(best_pos_idx) + 1;
win_counts(best_vel_idx) = win_counts(best_vel_idx) + 1;
win_counts(best_sog_idx) = win_counts(best_sog_idx) + 1;
win_counts(best_cog_idx) = win_counts(best_cog_idx) + 1;

[max_wins, most_wins_idx] = max(win_counts);

fprintf('\n--- Category Wins Analysis ---\n');
fprintf('Number of categories won by each filter:\n');
for i = 1:length(filter_names)
    fprintf('  %s: %d wins\n', filter_names{i}, win_counts(i));
end

% Final recommendation
fprintf('\n===== OVERALL BEST FILTER RECOMMENDATION =====\n');
fprintf('Based on Weighted Composite Score: %s (Score: %.3f)\n', ...
    filter_names{best_overall_idx}, composite_scores(best_overall_idx));
fprintf('Based on Average Ranking: %s (Avg Rank: %.2f)\n', ...
    filter_names{best_rank_idx}, avg_ranks(best_rank_idx));
fprintf('Based on Category Wins: %s (%d out of 4 categories)\n', ...
    filter_names{most_wins_idx}, max_wins);

% Determine consensus recommendation
recommendations = [best_overall_idx, best_rank_idx, most_wins_idx];
recommendation_counts = histcounts(recommendations, 1:length(filter_names)+1);
[~, consensus_idx] = max(recommendation_counts);

if recommendation_counts(consensus_idx) >= 2
    fprintf('\nCONSENSUS RECOMMENDATION: %s\n', filter_names{consensus_idx});
    fprintf('   (Recommended by %d out of 3 methods)\n', recommendation_counts(consensus_idx));
else
    fprintf('\nPRIMARY RECOMMENDATION: %s (Weighted Composite)\n', filter_names{best_overall_idx});
    fprintf('   (No clear consensus - defaulting to weighted composite score)\n');
end

% Performance summary for the best filter
best_filter = filter_names{best_overall_idx};
fprintf('\n--- Performance Summary for %s ---\n', best_filter);
fprintf('Position RMSE: %.2f m (Rank: %d/5)\n', pos_rmse_values(best_overall_idx), pos_ranks(best_overall_idx));
fprintf('Velocity RMSE: %.2f m/s (Rank: %d/5)\n', vel_rmse_values(best_overall_idx), vel_ranks(best_overall_idx));
fprintf('SOG RMSE: %.2f m/s (Rank: %d/5)\n', sog_rmse_values(best_overall_idx), sog_ranks(best_overall_idx));
fprintf('COG RMSE: %.2f deg (Rank: %d/5)\n', cog_rmse_values(best_overall_idx), cog_ranks(best_overall_idx));

end