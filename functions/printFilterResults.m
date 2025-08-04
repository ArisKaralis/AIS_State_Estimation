function printFilterResults(stats)
    % PRINTFILTERRESULTS - Print standardized filter results
    
    if ~isnan(stats.position_rmse)
        fprintf('\n===== %s Performance Statistics =====\n', stats.filterName);
        fprintf('Position RMSE: %.2f m\n', stats.position_rmse);
        fprintf('Position Mean Error: %.2f m\n', stats.position_mean);
        fprintf('Position Max Error: %.2f m\n', stats.position_max);
        
        if ~isnan(stats.velocity_rmse)
            fprintf('Velocity RMSE: %.2f m/s\n', stats.velocity_rmse);
        end
        if ~isnan(stats.sog_rmse)
            fprintf('SOG RMSE: %.2f m/s\n', stats.sog_rmse);
            fprintf('COG RMSE: %.2f degrees\n', stats.cog_rmse);
        end
    end
end