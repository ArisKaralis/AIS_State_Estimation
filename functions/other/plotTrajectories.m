function plotTrajectories(data_raw, data_resampled, data_segmented, mmsi)
    figure;
    plot(data_raw.LON, data_raw.LAT, 'k--', 'DisplayName', 'Raw', 'LineWidth', 1);
    hold on;
    plot(data_resampled.LON, data_resampled.LAT, 'r-', 'DisplayName', 'Resampled', 'LineWidth', 0.5);
    plot(data_segmented.LON, data_segmented.LAT, 'b--', 'DisplayName', 'Segmented', 'LineWidth', 1);
    xlabel('Longitude'); ylabel('Latitude');
    title(['AIS Trajectory Comparison (MMSI: ', num2str(mmsi), ')']);
    legend('Location', 'best'); grid on;
end
