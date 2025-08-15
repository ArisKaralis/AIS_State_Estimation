function saveFilterAnalysisResults(results, verbose)
% SAVEFILTERANALYSISRESULTS - Save filter analysis results to files
% 
% Inputs:
%   results - Results structure from runMonteCarloFilterAnalysis
%   verbose - Boolean for verbose output

% Generate timestamp
timestamp = datestr(now, 'yyyymmdd_HHMMSS');

% Save complete results to MAT file
if verbose
    fprintf('Saving results to MAT file...\n');
end
filename = sprintf('output/best_filters_results_%s.mat', timestamp);
save(filename, 'results', '-v7.3');

% Create summary CSV table with separate metrics
if verbose
    fprintf('Creating summary CSV table...\n');
end
csvFilename = sprintf('output/best_filters_separate_metrics_%s.csv', timestamp);

filterNames = results.comparison.filterTypes;

% Build summary table
summaryTable = table();
summaryTable.Filter = filterNames';

% Position metrics
summaryTable.Position_RMSE_Mean = results.comparison.position_rmse.means';
summaryTable.Position_RMSE_Std = results.comparison.position_rmse.stds';
% Velocity metrics  
summaryTable.Velocity_RMSE_Mean = results.comparison.velocity_rmse.means';
summaryTable.Velocity_RMSE_Std = results.comparison.velocity_rmse.stds';

% SOG metrics
summaryTable.SOG_RMSE_Mean = results.comparison.sog_rmse.means';
summaryTable.SOG_RMSE_Std = results.comparison.sog_rmse.stds';

% COG metrics
summaryTable.COG_RMSE_Mean = results.comparison.cog_rmse.means';
summaryTable.COG_RMSE_Std = results.comparison.cog_rmse.stds';

% Runtime statistics
runtime_means = NaN(length(filterNames), 1);
runtime_stds = NaN(length(filterNames), 1);

for i = 1:length(filterNames)
    filterName = filterNames{i};
    if isfield(results.filterStats.(filterName), 'summary') && ...
       isfield(results.filterStats.(filterName).summary, 'runtime')
        runtime_means(i) = results.filterStats.(filterName).summary.runtime.mean;
        runtime_stds(i) = results.filterStats.(filterName).summary.runtime.std;
    end
end

summaryTable.Runtime_Mean = runtime_means;
summaryTable.Runtime_Std = runtime_stds;

% Calculate relative performance metric based on normalized position RMSE
pos_means = results.comparison.position_rmse.means;
valid_pos = ~isnan(pos_means);
if any(valid_pos)
    % Calculate relative performance: lower RMSE = better performance (higher score)
    max_rmse = max(pos_means(valid_pos));
    min_rmse = min(pos_means(valid_pos));
    relative_performance = NaN(length(pos_means), 1);
    if max_rmse > min_rmse
        % Normalize to 0-100 scale (100 = best performance)
        relative_performance(valid_pos) = 100 * (max_rmse - pos_means(valid_pos)) / (max_rmse - min_rmse);
    else
        % All filters have same performance
        relative_performance(valid_pos) = 100;
    end
    summaryTable.Relative_Performance = relative_performance;
else
    % No valid position data
    summaryTable.Relative_Performance = NaN(length(filterNames), 1);
end

% Add measurement baselines if available
if isfield(results, 'measurementStats') && ~isempty(results.measurementStats.position_rmse)
    summaryTable.Measurement_Pos_RMSE = repmat(mean(results.measurementStats.position_rmse), length(filterNames), 1);
    summaryTable.Measurement_Vel_RMSE = repmat(mean(results.measurementStats.velocity_rmse), length(filterNames), 1);
    summaryTable.Measurement_SOG_RMSE = repmat(mean(results.measurementStats.sog_rmse), length(filterNames), 1);
    summaryTable.Measurement_COG_RMSE = repmat(mean(results.measurementStats.cog_rmse), length(filterNames), 1);
end

writetable(summaryTable, csvFilename);

if verbose
    fprintf('Summary saved to: %s\n', csvFilename);
    fprintf('Complete results saved to: %s\n', filename);
end

end