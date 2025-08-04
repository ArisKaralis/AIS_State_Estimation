function estimates = createEstimatesTable(timestamps, x_est, state_type)
    % CREATEESTIMATESTABLE - Create standardized estimates table
    % state_type: 'cv' (4-state), 'ca' (6-state), 'ctrv' (5-state)
    
    switch lower(state_type)
        case 'cv'
            % State: [x, vx, y, vy]
            estimates = table(timestamps, x_est(1,:)', x_est(3,:)', ...
                             x_est(2,:)', x_est(4,:)', ...
                             sqrt(x_est(2,:).^2 + x_est(4,:).^2)', ...
                             mod(atan2(x_est(4,:), x_est(2,:)) * 180/pi, 360)', ...
                             'VariableNames', {'timestamp', 'x_est', 'y_est', 'vx_est', 'vy_est', 'sog_est', 'cog_est'});
            
        case 'ca'
            % State: [x, vx, ax, y, vy, ay]
            estimates = table(timestamps, x_est(1,:)', x_est(4,:)', ...
                             x_est(2,:)', x_est(5,:)', x_est(3,:)', x_est(6,:)', ...
                             sqrt(x_est(2,:).^2 + x_est(5,:).^2)', ...
                             mod(atan2(x_est(5,:), x_est(2,:)) * 180/pi, 360)', ...
                             'VariableNames', {'timestamp', 'x_est', 'y_est', 'vx_est', 'vy_est', 'ax_est', 'ay_est', 'sog_est', 'cog_est'});
            
        case 'ctrv'
            % State: [x, y, v, yaw, yaw_rate]
            vx_est = x_est(3,:) .* cos(x_est(4,:));
            vy_est = x_est(3,:) .* sin(x_est(4,:));
            sog_est = x_est(3,:);
            cog_est = mod(x_est(4,:) * 180/pi, 360);
            
            estimates = table(timestamps, x_est(1,:)', x_est(2,:)', ...
                             vx_est', vy_est', sog_est', cog_est', x_est(5,:)', ...
                             'VariableNames', {'timestamp', 'x_est', 'y_est', 'vx_est', 'vy_est', 'sog_est', 'cog_est', 'yaw_rate_est'});
            
        otherwise
            error('Unknown state type: %s', state_type);
    end
end