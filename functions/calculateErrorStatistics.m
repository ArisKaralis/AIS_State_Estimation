function stats = calculateErrorStatistics(data, estimates, filterName)
% CALCULATEERRORSTATISTICS - Calculate error metrics for filter estimates

% Check if ground truth is available
if ~all(ismember({'x_true', 'y_true', 'vx_true', 'vy_true'}, data.Properties.VariableNames))
    warning('Ground truth not available, cannot calculate error statistics.');
    stats = struct('filterName', filterName);
    return;
end

% Calculate position error
posError = sqrt((estimates.x_est - data.x_true).^2 + ...
                (estimates.y_est - data.y_true).^2);

% Calculate velocity error
velError = sqrt((estimates.vx_est - data.vx_true).^2 + ...
                (estimates.vy_est - data.vy_true).^2);

% Calculate SOG and COG errors
sogError = abs(estimates.sog_est - data.sog_true);
cogError = abs(angdiff(deg2rad(estimates.cog_est), deg2rad(data.cog_true))) * 180/pi;

% Calculate statistics
stats = struct();
stats.filterName = filterName;
stats.posRMSE = sqrt(mean(posError.^2));
stats.posMAE = mean(posError);
stats.posMax = max(posError);
stats.pos95Pct = prctile(posError, 95);

stats.velRMSE = sqrt(mean(velError.^2));
stats.velMAE = mean(velError);
stats.velMax = max(velError);
stats.vel95Pct = prctile(velError, 95);

stats.sogRMSE = sqrt(mean(sogError.^2));
stats.sogMAE = mean(sogError);

stats.cogRMSE = sqrt(mean(cogError.^2));
stats.cogMAE = mean(cogError);

% Display statistics
fprintf('\n===== %s Performance Statistics =====\n', filterName);
fprintf('Position RMSE: %.2f m\n', stats.posRMSE);
fprintf('Velocity RMSE: %.2f m/s\n', stats.velRMSE);
fprintf('SOG RMSE: %.2f m/s\n', stats.sogRMSE);
fprintf('COG RMSE: %.2f degrees\n', stats.cogRMSE);

% Calculate error by segment
segments = unique(data.segment);
fprintf('\nPerformance by segment:\n');
fprintf('%-15s | %-15s | %-15s | %-15s\n', 'Segment', 'Pos RMSE (m)', 'Vel RMSE (m/s)', 'Course RMSE (deg)');
fprintf('---------------------------------------------------------------------\n');

segStats = struct();
for i = 1:length(segments)
    seg = segments(i);
    segIdx = data.segment == seg;
    segName = char(data.segment_name(find(segIdx, 1)));
    
    segPosError = posError(segIdx);
    segVelError = velError(segIdx);
    segCogError = cogError(segIdx);
    
    segPosRMSE = sqrt(mean(segPosError.^2));
    segVelRMSE = sqrt(mean(segVelError.^2));
    segCogRMSE = sqrt(mean(segCogError.^2));
    
    fprintf('%-15s | %-15.2f | %-15.2f | %-15.2f\n', ...
        segName, segPosRMSE, segVelRMSE, segCogRMSE);
    
    % Store stats for each segment
    segField = ['segment', num2str(seg)];
    segStats.(segField).name = segName;
    segStats.(segField).posRMSE = segPosRMSE;
    segStats.(segField).velRMSE = segVelRMSE;
    segStats.(segField).cogRMSE = segCogRMSE;
end

% Add segment stats to overall stats
stats.segmentStats = segStats;
end

