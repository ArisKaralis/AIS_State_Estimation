function printFilterResultsEnhanced(stats)
    % PRINTFILTERRESULTSENHANCED - Print results with position-only NEES
    
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
        
        % Print Position NEES statistics
        if ~isnan(stats.nees_mean)
            fprintf('\n--- Position Consistency Metrics ---\n');
            fprintf('Position NEES Mean: %.2f (Expected: 2.0 for 2D position)\n', stats.nees_mean);
            
            % ADD MEDIAN DISPLAY
            if isfield(stats, 'nees_median') && ~isnan(stats.nees_median)
                fprintf('Position NEES Median: %.2f\n', stats.nees_median);
            end
            
            if isfield(stats, 'nees_within_bounds')
                if stats.nees_within_bounds
                    fprintf('Position NEES Consistency: PASSED (within 95%% chi-squared bounds)\n');
                else
                    fprintf('Position NEES Consistency: FAILED (outside 95%% chi-squared bounds)\n');
                end
                fprintf('Position NEES Bounds: [%.2f, %.2f]\n', stats.nees_chi2_lower, stats.nees_chi2_upper);
            end
            
            % Provide tuning guidance based on mean (but also show median for context)
            if stats.nees_mean < 1.0
                fprintf('SEVERELY OVERCONFIDENT: Position uncertainty too small\n');
                fprintf('   Recommendation: Increase pos_std and process noise significantly\n');
            elseif stats.nees_mean < 1.5
                fprintf('OVERCONFIDENT: Position uncertainty too small\n');
                fprintf('   Recommendation: Increase pos_std and process noise moderately\n');
            elseif stats.nees_mean > 3.0
                fprintf('UNDERCONFIDENT: Position uncertainty too large\n');
                fprintf('   Recommendation: Decrease pos_std and process noise\n');
            else
                fprintf('WELL CALIBRATED: Position uncertainty appropriate\n');
            end
        end
        
        % Print NIS statistics (unchanged)
        if ~isnan(stats.nis_mean)
            fprintf('\n--- Innovation Consistency Metrics ---\n');
            fprintf('NIS Mean: %.2f (Expected: ~2 for position measurements)\n', stats.nis_mean);
            
            % ADD MEDIAN DISPLAY
            if isfield(stats, 'nis_median') && ~isnan(stats.nis_median)
                fprintf('NIS Median: %.2f\n', stats.nis_median);
            end
            
            if isfield(stats, 'nis_within_bounds')
                if stats.nis_within_bounds
                    fprintf('NIS Consistency: PASSED (within 95%% chi-squared bounds)\n');
                else
                    fprintf('NIS Consistency: FAILED (outside 95%% chi-squared bounds)\n');
                end
                fprintf('NIS Bounds: [%.2f, %.2f]\n', stats.nis_chi2_lower, stats.nis_chi2_upper);
            end
        end
    end
end