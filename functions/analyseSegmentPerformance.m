function analyseSegmentPerformance(data, kf_est, ekf_est_cv, ekf_est_ca, ukf_est, imm_est)
% ANALYZESEGMENTPERFORMANCE - Analyze filter performance by motion segment

% Get unique segments
segments = unique(data.segment);
numSegments = length(segments);
segment_names = cell(numSegments, 1);

% Filter names for indexing
filter_names = {'KF', 'EKF-CV', 'EKF-CA', 'UKF', 'IMM'};

% Pre-allocate arrays for RMSE by segment
pos_rmse = zeros(5, numSegments);  % [KF; EKF-CV; EKF-CA; UKF; IMM] x numSegments
vel_rmse = zeros(5, numSegments);
sog_rmse = zeros(5, numSegments);
cog_rmse = zeros(5, numSegments);

% Calculate performance metrics for each segment
for i = 1:numSegments
    seg = segments(i);
    segIdx = data.segment == seg;
    segment_names{i} = char(data.segment_name(find(segIdx, 1)));
    
    % Position RMSE
    pos_rmse(1, i) = sqrt(mean((sqrt((kf_est.x_est(segIdx) - data.x_true(segIdx)).^2 + ...
                                    (kf_est.y_est(segIdx) - data.y_true(segIdx)).^2)).^2));
    pos_rmse(2, i) = sqrt(mean((sqrt((ekf_est_cv.x_est(segIdx) - data.x_true(segIdx)).^2 + ...
                                     (ekf_est_cv.y_est(segIdx) - data.y_true(segIdx)).^2)).^2));
    pos_rmse(3, i) = sqrt(mean((sqrt((ekf_est_ca.x_est(segIdx) - data.x_true(segIdx)).^2 + ...
                                     (ekf_est_ca.y_est(segIdx) - data.y_true(segIdx)).^2)).^2));
    pos_rmse(4, i) = sqrt(mean((sqrt((ukf_est.x_est(segIdx) - data.x_true(segIdx)).^2 + ...
                                     (ukf_est.y_est(segIdx) - data.y_true(segIdx)).^2)).^2));
    pos_rmse(5, i) = sqrt(mean((sqrt((imm_est.x_est(segIdx) - data.x_true(segIdx)).^2 + ...
                                     (imm_est.y_est(segIdx) - data.y_true(segIdx)).^2)).^2));
    
    % Velocity RMSE
    vel_rmse(1, i) = sqrt(mean((sqrt((kf_est.vx_est(segIdx) - data.vx_true(segIdx)).^2 + ...
                                    (kf_est.vy_est(segIdx) - data.vy_true(segIdx)).^2)).^2));
    vel_rmse(2, i) = sqrt(mean((sqrt((ekf_est_cv.vx_est(segIdx) - data.vx_true(segIdx)).^2 + ...
                                     (ekf_est_cv.vy_est(segIdx) - data.vy_true(segIdx)).^2)).^2));
    vel_rmse(3, i) = sqrt(mean((sqrt((ekf_est_ca.vx_est(segIdx) - data.vx_true(segIdx)).^2 + ...
                                     (ekf_est_ca.vy_est(segIdx) - data.vy_true(segIdx)).^2)).^2));
    vel_rmse(4, i) = sqrt(mean((sqrt((ukf_est.vx_est(segIdx) - data.vx_true(segIdx)).^2 + ...
                                     (ukf_est.vy_est(segIdx) - data.vy_true(segIdx)).^2)).^2));
    vel_rmse(5, i) = sqrt(mean((sqrt((imm_est.vx_est(segIdx) - data.vx_true(segIdx)).^2 + ...
                                     (imm_est.vy_est(segIdx) - data.vy_true(segIdx)).^2)).^2));
    
    % SOG RMSE
    sog_rmse(1, i) = sqrt(mean((kf_est.sog_est(segIdx) - data.sog_true(segIdx)).^2));
    sog_rmse(2, i) = sqrt(mean((ekf_est_cv.sog_est(segIdx) - data.sog_true(segIdx)).^2));
    sog_rmse(3, i) = sqrt(mean((ekf_est_ca.sog_est(segIdx) - data.sog_true(segIdx)).^2));
    sog_rmse(4, i) = sqrt(mean((ukf_est.sog_est(segIdx) - data.sog_true(segIdx)).^2));
    sog_rmse(5, i) = sqrt(mean((imm_est.sog_est(segIdx) - data.sog_true(segIdx)).^2));
    
    % COG RMSE
    cog_rmse(1, i) = sqrt(mean((angdiff(deg2rad(kf_est.cog_est(segIdx)), ...
                                       deg2rad(data.cog_true(segIdx))) * 180/pi).^2));
    cog_rmse(2, i) = sqrt(mean((angdiff(deg2rad(ekf_est_cv.cog_est(segIdx)), ...
                                       deg2rad(data.cog_true(segIdx))) * 180/pi).^2));
    cog_rmse(3, i) = sqrt(mean((angdiff(deg2rad(ekf_est_ca.cog_est(segIdx)), ...
                                       deg2rad(data.cog_true(segIdx))) * 180/pi).^2));
    cog_rmse(4, i) = sqrt(mean((angdiff(deg2rad(ukf_est.cog_est(segIdx)), ...
                                       deg2rad(data.cog_true(segIdx))) * 180/pi).^2));
    cog_rmse(5, i) = sqrt(mean((angdiff(deg2rad(imm_est.cog_est(segIdx)), ...
                                       deg2rad(data.cog_true(segIdx))) * 180/pi).^2));
end

% Create bar charts for visualization
figure;

% Position RMSE by segment
subplot(2, 2, 1);
bar(pos_rmse');
xlabel('Segment');
ylabel('RMSE (m)');
title('Position Error by Segment');
set(gca, 'XTickLabel', segment_names);
legend('KF', 'EKF-CV', 'EKF-CA', 'UKF', 'IMM', 'Location', 'best');
grid on;

% Velocity RMSE by segment
subplot(2, 2, 2);
bar(vel_rmse');
xlabel('Segment');
ylabel('RMSE (m/s)');
title('Velocity Error by Segment');
set(gca, 'XTickLabel', segment_names);
legend('KF', 'EKF-CV', 'EKF-CA', 'UKF', 'IMM', 'Location', 'best');
grid on;

% SOG RMSE by segment
subplot(2, 2, 3);
bar(sog_rmse');
xlabel('Segment');
ylabel('RMSE (m/s)');
title('Speed Over Ground Error by Segment');
set(gca, 'XTickLabel', segment_names);
legend('KF', 'EKF-CV', 'EKF-CA', 'UKF', 'IMM', 'Location', 'best');
grid on;

% COG RMSE by segment
subplot(2, 2, 4);
bar(cog_rmse');
xlabel('Segment');
ylabel('RMSE (degrees)');
title('Course Over Ground Error by Segment');
set(gca, 'XTickLabel', segment_names);
legend('KF', 'EKF-CV', 'EKF-CA', 'UKF', 'IMM', 'Location', 'best');
grid on;

sgtitle('Filter Performance by Motion Segment');

% Create output directory if needed
if ~exist('figures', 'dir')
    mkdir('figures');
end

% Save figure
saveas(gcf, 'figures/segment_performance.png');
saveas(gcf, 'figures/segment_performance.fig');

% Print segment analysis
fprintf('\n===== Filter Performance by Segment =====\n');
for i = 1:numSegments
    fprintf('\nSegment %d: %s\n', segments(i), segment_names{i});
    fprintf('%-10s | %-10s | %-10s | %-10s | %-10s\n', ...
            'Filter', 'Pos RMSE', 'Vel RMSE', 'SOG RMSE', 'COG RMSE');
    fprintf('-------------------------------------------------------------\n');
    fprintf('%-10s | %-10.2f | %-10.2f | %-10.2f | %-10.2f\n', ...
            'KF', pos_rmse(1,i), vel_rmse(1,i), sog_rmse(1,i), cog_rmse(1,i));
    fprintf('%-10s | %-10.2f | %-10.2f | %-10.2f | %-10.2f\n', ...
            'EKF-CV', pos_rmse(2,i), vel_rmse(2,i), sog_rmse(2,i), cog_rmse(2,i));
    fprintf('%-10s | %-10.2f | %-10.2f | %-10.2f | %-10.2f\n', ...
            'EKF-CA', pos_rmse(3,i), vel_rmse(3,i), sog_rmse(3,i), cog_rmse(3,i));
    fprintf('%-10s | %-10.2f | %-10.2f | %-10.2f | %-10.2f\n', ...
            'UKF', pos_rmse(4,i), vel_rmse(4,i), sog_rmse(4,i), cog_rmse(4,i));
    fprintf('%-10s | %-10.2f | %-10.2f | %-10.2f | %-10.2f\n', ...
            'IMM', pos_rmse(5,i), vel_rmse(5,i), sog_rmse(5,i), cog_rmse(5,i));
    
    % Find best filter for each metric in this segment
    [~, bestPosIdx] = min(pos_rmse(:,i));
    [~, bestVelIdx] = min(vel_rmse(:,i));
    [~, bestSogIdx] = min(sog_rmse(:,i));
    [~, bestCogIdx] = min(cog_rmse(:,i));
    
    fprintf('\nBest filters for this segment:\n');
    fprintf('Position: %s (%.2f m)\n', ...
            filter_names{bestPosIdx}, pos_rmse(bestPosIdx,i));
    fprintf('Velocity: %s (%.2f m/s)\n', ...
            filter_names{bestVelIdx}, vel_rmse(bestVelIdx,i));
    fprintf('SOG: %s (%.2f m/s)\n', ...
            filter_names{bestSogIdx}, sog_rmse(bestSogIdx,i));
    fprintf('COG: %s (%.2f degrees)\n', ...
            filter_names{bestCogIdx}, cog_rmse(bestCogIdx,i));
end

% Calculate and print overall best filter for each segment
fprintf('\n===== Best Overall Filter by Segment =====\n');
for i = 1:numSegments
    % Calculate normalized scores (lower is better)
    % Normalize each error by the maximum error across filters
    norm_pos = pos_rmse(:,i) / max(pos_rmse(:,i));
    norm_vel = vel_rmse(:,i) / max(vel_rmse(:,i));
    norm_sog = sog_rmse(:,i) / max(sog_rmse(:,i));
    norm_cog = cog_rmse(:,i) / max(cog_rmse(:,i));
    
    % Overall score (equal weighting)
    overall_score = norm_pos + norm_vel + norm_sog + norm_cog;
    
    % Find best filter (lowest score)
    [~, bestIdx] = min(overall_score);
    
    fprintf('Segment %d (%s): Best filter is %s\n', ...
            segments(i), segment_names{i}, filter_names{bestIdx});
end

end