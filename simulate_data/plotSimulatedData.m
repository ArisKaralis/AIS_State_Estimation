function plotSimulatedData(x_true, z_meas, sog_meas, cog_meas, sog_true, cog_true)
    figure;
    plot(x_true(1,:), x_true(3,:), 'k--', 'LineWidth', 1.2); hold on;
    plot(z_meas(1,:), z_meas(2,:), 'ro');
    legend('Ground Truth', 'Noisy AIS Measurements');
    xlabel('Easting (m)'); ylabel('Northing (m)');
    title('Simulated Vessel Trajectory (Truth vs AIS)');
    grid on;

    vx = x_true(2, :);
    vy = x_true(4, :);
    
    figure;
    
    % --- SOG plot ---
    subplot(2,1,1);
    plot(sog_true, 'b', 'DisplayName', 'True SOG'); 
    hold on;
    plot(sog_meas, 'r--', 'DisplayName', 'Measured SOG');
    title('Speed Over Ground (SOG)');
    ylabel('m/s');
    legend('Location', 'best');
    grid on;
    
    % --- COG plot ---
    subplot(2,1,2);
    plot(cog_true, 'b', 'DisplayName', 'True COG'); hold on;
    plot(cog_meas, 'r--', 'DisplayName', 'Measured COG');
    title('Course Over Ground (COG)');
    ylabel('deg');
    xlabel('Time Step');
    legend('Location', 'best');
    grid on;

end
