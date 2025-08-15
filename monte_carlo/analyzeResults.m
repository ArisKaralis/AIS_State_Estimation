function results = analyzeResults(results, measurementStats, config)
% ANALYZERESULTS - Analyze and compare filter results
%
% Inputs:
%   results - Results structure with filter statistics
%   measurementStats - Measurement statistics
%   config - Configuration structure
%
% Outputs:
%   results - Updated results structure with comparison analysis

% Generate comparison analysis
results.comparison = generateFilterComparison(results.filterStats, config.filterTypes);

% Display measurement statistics
if config.verbose
    displayMeasurementStatistics(measurementStats);
end

% Generate comprehensive report
if config.verbose
    generateFilterAnalysisReport(results);
end

% Save results if requested
if config.saveResults
    saveFilterAnalysisResults(results, config.verbose);
end

% Display final summary
if config.verbose
    displayFinalSummary(results, measurementStats);
end

end

function displayMeasurementStatistics(measurementStats)
% Display measurement quality statistics
if ~isempty(measurementStats.position_rmse)
    fprintf('\n===== Measurement Quality Statistics =====\n');
    fprintf('Position measurement RMSE: %.2f ± %.2f m (%.2f to %.2f m)\n', ...
        mean(measurementStats.position_rmse), std(measurementStats.position_rmse), ...
        min(measurementStats.position_rmse), max(measurementStats.position_rmse));
    fprintf('Velocity measurement RMSE: %.2f ± %.2f m/s (%.2f to %.2f m/s)\n', ...
        mean(measurementStats.velocity_rmse), std(measurementStats.velocity_rmse), ...
        min(measurementStats.velocity_rmse), max(measurementStats.velocity_rmse));
    fprintf('SOG measurement RMSE: %.2f ± %.2f m/s (%.2f to %.2f m/s)\n', ...
        mean(measurementStats.sog_rmse), std(measurementStats.sog_rmse), ...
        min(measurementStats.sog_rmse), max(measurementStats.sog_rmse));
    fprintf('COG measurement RMSE: %.2f ± %.2f° (%.2f to %.2f°)\n', ...
        mean(measurementStats.cog_rmse), std(measurementStats.cog_rmse), ...
        min(measurementStats.cog_rmse), max(measurementStats.cog_rmse));
    fprintf('Average observations per run: %.1f\n', mean(measurementStats.observation_counts));
end
end

function displayFinalSummary(results, measurementStats)
% Display final summary
fprintf('Best performing filter: %s\n', results.comparison.best_filter);
if ~isnan(results.comparison.best_rmse)
    fprintf('Best RMSE: %.2f meters\n', results.comparison.best_rmse);
    
    % Display improvement over measurements
    if ~isempty(measurementStats.position_rmse)
        measurementMean = mean(measurementStats.position_rmse);
        improvement = ((measurementMean - results.comparison.best_rmse) / measurementMean) * 100;
        if improvement > 0
            fprintf('Improvement over raw measurements: %.1f%%\n', improvement);
        else
            fprintf('Performance vs raw measurements: %.1f%% (worse)\n', abs(improvement));
        end
    end
end
end