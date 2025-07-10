function evaluateFilter(data, x_est, P_est, R, H)
    % Ground truth from data
    x_gt = [data.x'; data.vx'; data.y'; data.vy'];
    x_kf = x_est;

    % === RMSE ===
    rmse_pos = computeRMSE(x_kf([1 3], :), x_gt([1 3], :));
    rmse_vel = computeRMSE(x_kf([2 4], :), x_gt([2 4], :));

    % === NEES ===
    nees_vals = computeNEES(x_kf, x_gt, P_est);
    avg_nees = mean(nees_vals);

    % === NIS === (2D position measurement only)
    z_meas = x_gt([1 3], :);
    nis_vals = computeNIS(x_kf, z_meas, P_est, H, R);
    avg_nis = mean(nis_vals);

    % === Display ===
    fprintf('==== KF Evaluation ====\n');
    fprintf('Position RMSE: %.2f meters\n', rmse_pos);
    fprintf('Velocity RMSE: %.2f m/s\n', rmse_vel);
    fprintf('Average NEES: %.2f\n', avg_nees);
    fprintf('Average NIS: %.2f\n', avg_nis);

    % === Plot Trajectory ===
    figure;
    plot(x_gt(1,:), x_gt(3,:), 'k--', 'DisplayName', 'Measured (UTM)'); hold on;
    plot(x_kf(1,:), x_kf(3,:), 'b', 'DisplayName', 'Kalman Estimate');
    xlabel('Easting (m)'); ylabel('Northing (m)');
    title('Trajectory: Kalman Filter Estimate vs Measurement');
    legend('show'); grid on;

    % === Plot Velocity Diagnostics ===
    plotVelocityDiagnostics(data, x_est);
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

function nis_vals = computeNIS(x_est, z_meas, P_est, H, R)
    n = size(x_est, 2);
    nis_vals = zeros(1, n);
    for k = 1:n
        innov = z_meas(:,k) - H * x_est(:,k);
        S = H * P_est(:,:,k) * H' + R;
        nis_vals(k) = innov' / S * innov;
    end
end
function plotVelocityDiagnostics(data, x_est)
    time = data.timestamp;

    vx_gt = data.vx;
    vy_gt = data.vy;

    vx_kf = x_est(2, :)';
    vy_kf = x_est(4, :)';

    % Plot vx
    figure;
    subplot(2,1,1);
    plot(time, vx_gt, 'k--', 'DisplayName', 'Measured vx');
    hold on;
    plot(time, vx_kf, 'b', 'DisplayName', 'Estimated vx');
    ylabel('vx (m/s)');
    title('Velocity X over Time');
    legend; grid on;

    % Plot vy
    subplot(2,1,2);
    plot(time, vy_gt, 'k--', 'DisplayName', 'Measured vy');
    hold on;
    plot(time, vy_kf, 'b', 'DisplayName', 'Estimated vy');
    xlabel('Time'); ylabel('vy (m/s)');
    title('Velocity Y over Time');
    legend; grid on;
end
