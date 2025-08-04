function stats = calculateFilterStats(estimates, data, filterName)
    % CALCULATEFILTERSTATS - Calculate standardized filter statistics
    
    stats = struct();
    stats.filterName = filterName;
    
    if all(ismember({'x_true', 'y_true'}, data.Properties.VariableNames))
        pos_error = sqrt((estimates.x_est - data.x_true).^2 + (estimates.y_est - data.y_true).^2);
        stats.position_rmse = sqrt(mean(pos_error.^2));
        stats.position_mean = mean(pos_error);
        stats.position_max = max(pos_error);
        stats.position_std = std(pos_error);
        stats.position_errors = pos_error;
        
        if all(ismember({'vx_true', 'vy_true'}, data.Properties.VariableNames))
            vel_error = sqrt((estimates.vx_est - data.vx_true).^2 + (estimates.vy_est - data.vy_true).^2);
            stats.velocity_rmse = sqrt(mean(vel_error.^2));
            stats.velocity_mean = mean(vel_error);
            stats.velocity_max = max(vel_error);
        else
            stats.velocity_rmse = NaN;
            stats.velocity_mean = NaN;
            stats.velocity_max = NaN;
        end
        
        if all(ismember({'sog_true', 'cog_true'}, data.Properties.VariableNames))
            stats.sog_rmse = sqrt(mean((estimates.sog_est - data.sog_true).^2));
            stats.cog_rmse = sqrt(mean((angdiff(deg2rad(estimates.cog_est), deg2rad(data.cog_true)) * 180/pi).^2));
        else
            stats.sog_rmse = NaN;
            stats.cog_rmse = NaN;
        end
    else
        fprintf('No ground truth available for error calculation.\n');
        stats.position_rmse = NaN;
        stats.position_mean = NaN;
        stats.position_max = NaN;
        stats.velocity_rmse = NaN;
        stats.velocity_mean = NaN;
        stats.velocity_max = NaN;
        stats.sog_rmse = NaN;
        stats.cog_rmse = NaN;
    end
end