function compareSOGCOG(data_raw, data_resampled, data_segmented, mmsi)
    t_raw = datetime(data_raw.BaseDateTime);
    t_resampled = datetime(data_resampled.BaseDateTime);
    t_segmented = datetime(data_segmented.BaseDateTime);

    figure;
    plot(t_raw, data_raw.SOG, 'k-', 'DisplayName', 'Raw SOG'); hold on;
    plot(t_resampled, data_resampled.SOG, 'r--', 'DisplayName', 'Resampled SOG');
    plot(t_segmented, data_segmented.SOG, 'b-', 'DisplayName', 'Segmented SOG');
    xlabel('Time'); ylabel('Speed (m/s)');
    title(['SOG Comparison (MMSI: ', num2str(mmsi), ')']);
    legend('Location', 'best'); grid on;

    figure;
    plot(t_raw, data_raw.COG, 'k-', 'DisplayName', 'Raw COG'); hold on;
    plot(t_resampled, data_resampled.COG, 'r--', 'DisplayName', 'Resampled COG');
    plot(t_segmented, data_segmented.COG, 'b-', 'DisplayName', 'Segmented COG');
    xlabel('Time'); ylabel('Course over Ground (°)');
    title(['COG Comparison (MMSI: ', num2str(mmsi), ')']);
    legend('Location', 'best'); grid on;
end
