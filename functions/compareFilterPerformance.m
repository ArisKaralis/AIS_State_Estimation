function compareFilterPerformance(data, kf_est, ekf_est_cv, ekf_est_ca, ukf_est, imm_est)
% COMPAREFILTERPERFORMANCE - Compare and visualize performance across filters

% Create figure for comparison
figure;

% Sample indices
t = 1:height(data);

% Plot trajectory comparison
subplot(2, 2, 1);
plot(data.x, data.y, 'k.', 'MarkerSize', 4, 'DisplayName', 'Measurements');
hold on;
plot(data.x_true, data.y_true, 'k-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
plot(kf_est.x_est, kf_est.y_est, 'b-', 'LineWidth', 1.5, 'DisplayName', 'KF');
plot(ekf_est_cv.x_est, ekf_est_cv.y_est, 'r-', 'LineWidth', 1.5, 'DisplayName', 'EKF-CV');
plot(ekf_est_ca.x_est, ekf_est_ca.y_est, 'r-', 'LineWidth', 1.5, 'DisplayName', 'EKF-CA');
plot(ukf_est.x_est, ukf_est.y_est, 'g-', 'LineWidth', 1.5, 'DisplayName', 'UKF');
plot(imm_est.x_est, imm_est.y_est, 'm-', 'LineWidth', 1.5, 'DisplayName', 'IMM');

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

plot(t, pos_err_kf, 'b-', 'LineWidth', 1.5, 'DisplayName', 'KF');
hold on;
plot(t, pos_err_ekf_cv, 'r-', 'LineWidth', 1.5, 'DisplayName', 'EKF-CV');
plot(t, pos_err_ekf_ca, 'r-', 'LineWidth', 1.5, 'DisplayName', 'EKF-CA');
plot(t, pos_err_ukf, 'g-', 'LineWidth', 1.5, 'DisplayName', 'UKF');
plot(t, pos_err_imm, 'm-', 'LineWidth', 1.5, 'DisplayName', 'IMM');

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
plot(t, data.SOG, 'k.', 'MarkerSize', 4, 'DisplayName', 'Measurements');
hold on;
plot(t, data.sog_true, 'k-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
plot(t, kf_est.sog_est, 'b-', 'LineWidth', 1.5, 'DisplayName', 'KF');
plot(t, ekf_est_cv.sog_est, 'r-', 'LineWidth', 1.5, 'DisplayName', 'EKF-CV');
plot(t, ekf_est_ca.sog_est, 'r-', 'LineWidth', 1.5, 'DisplayName', 'EKF-CA');
plot(t, ukf_est.sog_est, 'g-', 'LineWidth', 1.5, 'DisplayName', 'UKF');
plot(t, imm_est.sog_est, 'm-', 'LineWidth', 1.5, 'DisplayName', 'IMM');

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
plot(t, data.COG, 'k.', 'MarkerSize', 4, 'DisplayName', 'Measurements');
hold on;
plot(t, data.cog_true, 'k-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
plot(t, kf_est.cog_est, 'b-', 'LineWidth', 1.5, 'DisplayName', 'KF');
plot(t, ekf_est_cv.cog_est, 'r-', 'LineWidth', 1.5, 'DisplayName', 'EKF-CV');
plot(t, ekf_est_ca.cog_est, 'r-', 'LineWidth', 1.5, 'DisplayName', 'EKF-CA');
plot(t, ukf_est.cog_est, 'g-', 'LineWidth', 1.5, 'DisplayName', 'UKF');
plot(t, imm_est.cog_est, 'm-', 'LineWidth', 1.5, 'DisplayName', 'IMM');

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
end