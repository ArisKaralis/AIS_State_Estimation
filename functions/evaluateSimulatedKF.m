function evaluateSimulatedKF(data, x_est, P_est, R, H)
% EVALUATESIMULATEDKF - Evaluate KF results against known ground truth from synthetic AIS
% Inputs:
%   data: table with ground truth columns: x, y, vx, vy
%   x_est: estimated states [4 x N]
%   P_est: state covariance matrices [4 x 4 x N]
%   R, H: measurement model and noise

% === Ground truth
x_gt = [data.x'; data.vx'; data.y'; data.vy'];
n = size(x_est, 2);

% === RMSE
rmse_pos = computeRMSE(x_est([1 3],:), x_gt([1 3],:));
rmse_vel = computeRMSE(x_est([2 4],:), x_gt([2 4],:));

% === NEES
nees_vals = computeNEES(x_est, x_gt, P_est);
avg_nees = mean(nees_vals);

% === NIS
z_meas = x_gt([1 3], :);  % true position
nis_vals = computeNIS(x_est, z_meas, P_est, H, R);
avg_nis = mean(nis_vals);

% === Print summary
fprintf('==== KF Evaluation (Simulated Data) ====\n');
fprintf('Position RMSE: %.2f m\n', rmse_pos);
fprintf('Velocity RMSE: %.2f m/s\n', rmse_vel);
fprintf('Average NEES: %.2f\n', avg_nees);
fprintf('Average NIS: %.2f\n', avg_nis);

% === Plots ===
plotTrajectoryComparison(data, x_gt, x_est);
plotStateErrors(data.timestamp, x_est, x_gt);

% === 1. Visual Check ===
figure;
plot(x_gt(1,:), x_gt(3,:), 'k--', 'DisplayName','Ground Truth');
hold on;
plot(x_est(1,:), x_est(3,:), 'b', 'DisplayName','KF Estimate');
legend;
title('Trajectory Comparison');

% === 2. RMSE Computation Debug ===
pos_error = x_gt([1 3], :) - x_est([1 3], :);
disp('Max position error in x:'); disp(max(abs(pos_error(1,:))));
disp('Max position error in y:'); disp(max(abs(pos_error(2,:))));

% === 3. Measurement comparison (was z = ground truth?) ===
figure;
plot(x_gt(1,:), 'k--', 'DisplayName','x GT'); hold on;
plot(data.x, 'ro', 'DisplayName','x Measurement');
legend; title('Check if x measurements match GT');

% === 4. Plot RMSE sensitivity to q_pos ===
q_vals = logspace(-2, 2, 20);
rmse_vals = zeros(size(q_vals));
for i = 1:length(q_vals)
    q = q_vals(i);
    Q = makeCVProcessNoise4D(dt, q, 0.5);
    [kf, ~, ~] = predictCorrectKF4D(kf, dt, Q, [data.x(k); data.y(k)]);
    err = x_gt([1 3], k) - kf.State([1 3]);
    rmse_vals(i) = sqrt(mean(err.^2));
end

figure; semilogx(q_vals, rmse_vals, '-o');
xlabel('q_{pos}'); ylabel('Position RMSE');
title('RMSE vs q_{pos}'); grid on;

end

function rmse = computeRMSE(est, gt)
    err = est - gt;
    rmse = sqrt(mean(sum(err.^2, 1)));
end

function nees_vals = computeNEES(x_est, x_gt, P_est)
    n = size(x_est, 2);
    nees_vals = zeros(1, n);
    for k = 1:n
        e = x_est(:,k) - x_gt(:,k);
        nees_vals(k) = e' / P_est(:,:,k) * e;
    end
end

function nis_vals = computeNIS(x_est, z, P_est, H, R)
    n = size(x_est, 2);
    nis_vals = zeros(1, n);
    for k = 1:n
        innov = z(:,k) - H * x_est(:,k);
        S = H * P_est(:,:,k) * H' + R;
        nis_vals(k) = innov' / S * innov;
    end
end

function plotStateErrors(time, x_est, x_gt)
    e_pos = vecnorm(x_est([1 3],:) - x_gt([1 3],:));
    e_vel = vecnorm(x_est([2 4],:) - x_gt([2 4],:));

    figure;
    subplot(2,1,1);
    plot(time, e_pos); title('Position Error (RMSE per step)');
    ylabel('meters'); grid on;

    subplot(2,1,2);
    plot(time, e_vel); title('Velocity Error (RMSE per step)');
    ylabel('m/s'); xlabel('Time'); grid on;
end

function plotTrajectoryComparison(data, x_gt, x_est)
    figure;
    plot(x_gt(1,:), x_gt(3,:), 'k-', 'LineWidth', 1.5, 'DisplayName', 'True Trajectory'); hold on;
    plot(data.x, data.y, 'r--', 'DisplayName', 'Measured (AIS)');
    plot(x_est(1,:), x_est(3,:), 'b', 'LineWidth', 1.5, 'DisplayName', 'KF Estimate');
    legend('Location', 'best');
    xlabel('Easting (m)'); ylabel('Northing (m)');
    title('Trajectory Comparison: True vs Measured vs KF Estimate');
    axis equal; grid on;
end
