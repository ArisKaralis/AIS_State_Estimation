function plotInterMessageGaps(data_raw, data_resampled, data_segment, mmsi)
    % === Compute dt vectors ===
    dt_raw = seconds(diff(data_raw.BaseDateTime));
    dt_resampled = seconds(diff(data_resampled.BaseDateTime));
    dt_segment = seconds(diff(data_segment.BaseDateTime));
    

    % === Plot ===
    figure;
    plot(dt_raw, 'k', 'DisplayName', 'Original Δt'); hold on;
    plot(dt_resampled, 'r', 'DisplayName', 'Resampled Δt', 'LineWidth', 1);
    plot(dt_segment, 'b-', 'DisplayName', 'Segment Δt', 'LineWidth', 4);
    xlabel('Sample Index'); ylabel('\Deltat (seconds)');
    title(['AIS Inter-Message Time Gaps (MMSI: ', num2str(mmsi), ')']);
    legend('show'); grid on;
end
