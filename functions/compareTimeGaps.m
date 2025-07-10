function compareTimeGaps(data_raw, data_resampled, data_segmented, mmsi)
    dt_raw = seconds(diff(data_raw.BaseDateTime));
    dt_resampled = seconds(diff(data_resampled.BaseDateTime));
    dt_segmented = seconds(diff(data_segmented.BaseDateTime));
    figure;
    plot(dt_raw, 'k', 'DisplayName', 'Original Δt'); hold on;
    plot(dt_resampled, 'r', 'DisplayName', 'Resampled Δt');
    plot(dt_segmented, 'b', 'DisplayName', 'Segmented Δt');
    xlabel('Sample Index'); ylabel('Δt (seconds)');
    title(['AIS Inter-Message Time Gaps (MMSI: ', num2str(mmsi), ')']);
    legend('Location', 'northeast'); grid on;
end
