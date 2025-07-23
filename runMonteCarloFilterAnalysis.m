function results = runMonteCarloFilterAnalysis(monteCarloResults, varargin)

p = inputParser;
addParameter(p, 'FilterTypes', {'KF', 'EKF_CV', 'EKF_CA', 'UKF', 'IMM'}, @iscell);
addParameter(p, 'QValues', struct(), @isstruct);
addParameter(p, 'SaveResults', true, @islogical);
parse(p, varargin{:});

filterTypes = p.Results.FilterTypes;
qValues = p.Results.QValues;
saveResults = p.Results.SaveResults;

if isempty(fieldnames(qValues))
    qValues.KF = 0.1;
    qValues.EKF_CV = 0.1;
    qValues.EKF_CA = 0.5;
    qValues.UKF = 0.1;
    qValues.IMM_CV = 0.1;
    qValues.IMM_CA = 0.5;
    qValues.IMM_CTRV = 0.1;
end

numRuns = length(monteCarloResults.runs);
numFilters = length(filterTypes);

fprintf('=== Monte Carlo Filter Analysis ===\n');
fprintf('Analyzing %d Monte Carlo runs with %d filter types\n', numRuns, numFilters);

results = struct();
results.filterStats = struct();
results.rawResults = cell(numRuns, numFilters);
results.comparison = struct();
results.metadata = struct();
results.metadata.timestamp = datetime('now');
results.metadata.numRuns = numRuns;
results.metadata.filterTypes = filterTypes;
results.metadata.qValues = qValues;

for i = 1:numFilters
    filterName = filterTypes{i};
    results.filterStats.(filterName) = struct();
    results.filterStats.(filterName).position_rmse = [];
    results.filterStats.(filterName).position_mean = [];
    results.filterStats.(filterName).position_std = [];
    results.filterStats.(filterName).position_max = [];
    results.filterStats.(filterName).position_min = [];
    results.filterStats.(filterName).velocity_rmse = [];
    results.filterStats.(filterName).velocity_mean = [];
    results.filterStats.(filterName).velocity_std = [];
    results.filterStats.(filterName).velocity_max = [];
    results.filterStats.(filterName).velocity_min = [];
    results.filterStats.(filterName).runtime = [];
end

for run = 1:numRuns
    fprintf('Processing run %d/%d\n', run, numRuns);
    
    groundTruth = monteCarloResults.runs{run}.groundTruth;
    observations = monteCarloResults.runs{run}.observations;
    metadata = monteCarloResults.runs{run}.metadata;
    
    try
        aisData = generateAISDataset(groundTruth, observations, metadata);
        
        if height(aisData) < 10
            fprintf('  Skipping run %d: Insufficient data (%d points)\n', run, height(aisData));
            continue;
        end
        
        for f = 1:numFilters
            filterName = filterTypes{f};
            
            try
                tic;
                
                switch filterName
                    case 'KF'
                        [estimates, stats] = runKalmanFilter(aisData, qValues.KF);
                        
                    case 'EKF_CV'
                        [estimates, stats] = runExtendedKalmanFilterCV(aisData, qValues.EKF_CV);
                        
                    case 'EKF_CA'
                        [estimates, stats] = runExtendedKalmanFilterCA(aisData, qValues.EKF_CA);
                        
                    case 'UKF'
                        [estimates, stats] = runUnscentedKalmanFilter(aisData, qValues.UKF);
                        
                    case 'IMM'
                        [estimates, stats] = runIMMFilterWithExistingModels(aisData, ...
                            qValues.IMM_CV, qValues.IMM_CA, qValues.IMM_CTRV);
                        
                    otherwise
                        error('Unknown filter type: %s', filterName);
                end
                
                runtime = toc;
                
                runMetrics = calculateFilterPerformance(aisData, estimates, stats);
                runMetrics.runtime = runtime;
                
                results.rawResults{run, f} = runMetrics;
                
                results.filterStats.(filterName).position_rmse(end+1) = runMetrics.position_rmse;
                results.filterStats.(filterName).position_mean(end+1) = runMetrics.position_mean;
                results.filterStats.(filterName).position_std(end+1) = runMetrics.position_std;
                results.filterStats.(filterName).position_max(end+1) = runMetrics.position_max;
                results.filterStats.(filterName).position_min(end+1) = runMetrics.position_min;
                
                if isfield(runMetrics, 'velocity_rmse')
                    results.filterStats.(filterName).velocity_rmse(end+1) = runMetrics.velocity_rmse;
                    results.filterStats.(filterName).velocity_mean(end+1) = runMetrics.velocity_mean;
                    results.filterStats.(filterName).velocity_std(end+1) = runMetrics.velocity_std;
                    results.filterStats.(filterName).velocity_max(end+1) = runMetrics.velocity_max;
                    results.filterStats.(filterName).velocity_min(end+1) = runMetrics.velocity_min;
                end
                
                results.filterStats.(filterName).runtime(end+1) = runtime;
                
                fprintf('    %s: RMSE=%.2fm, Runtime=%.3fs\n', filterName, runMetrics.position_rmse, runtime);
                
            catch ME
                fprintf('    Error in %s filter: %s\n', filterName, ME.message);
                continue;
            end
        end
        
    catch ME
        fprintf('  Error processing run %d: %s\n', run, ME.message);
        continue;
    end
end

fprintf('\n=== Calculating Summary Statistics ===\n');
for i = 1:numFilters
    filterName = filterTypes{i};
    
    posRMSE = results.filterStats.(filterName).position_rmse;
    if ~isempty(posRMSE)
        results.filterStats.(filterName).summary = struct();
        results.filterStats.(filterName).summary.position = struct();
        results.filterStats.(filterName).summary.position.rmse_mean = mean(posRMSE);
        results.filterStats.(filterName).summary.position.rmse_std = std(posRMSE);
        results.filterStats.(filterName).summary.position.rmse_min = min(posRMSE);
        results.filterStats.(filterName).summary.position.rmse_max = max(posRMSE);
        
        allPosErrors = [];
        for run = 1:numRuns
            if ~isempty(results.rawResults{run, i}) && isfield(results.rawResults{run, i}, 'position_errors')
                allPosErrors = [allPosErrors; results.rawResults{run, i}.position_errors];
            end
        end
        
        if ~isempty(allPosErrors)
            results.filterStats.(filterName).summary.position.overall_rmse = sqrt(mean(allPosErrors.^2));
            results.filterStats.(filterName).summary.position.overall_mean = mean(allPosErrors);
            results.filterStats.(filterName).summary.position.overall_std = std(allPosErrors);
        end
    end
    
    velRMSE = results.filterStats.(filterName).velocity_rmse;
    if ~isempty(velRMSE)
        results.filterStats.(filterName).summary.velocity = struct();
        results.filterStats.(filterName).summary.velocity.rmse_mean = mean(velRMSE);
        results.filterStats.(filterName).summary.velocity.rmse_std = std(velRMSE);
        results.filterStats.(filterName).summary.velocity.rmse_min = min(velRMSE);
        results.filterStats.(filterName).summary.velocity.rmse_max = max(velRMSE);
    end
    
    runtimes = results.filterStats.(filterName).runtime;
    if ~isempty(runtimes)
        results.filterStats.(filterName).summary.runtime = struct();
        results.filterStats.(filterName).summary.runtime.mean = mean(runtimes);
        results.filterStats.(filterName).summary.runtime.std = std(runtimes);
        results.filterStats.(filterName).summary.runtime.min = min(runtimes);
        results.filterStats.(filterName).summary.runtime.max = max(runtimes);
    end
    
    if isfield(results.filterStats.(filterName), 'summary') && isfield(results.filterStats.(filterName).summary, 'position')
        fprintf('%s Summary:\n', filterName);
        fprintf('  Position RMSE: %.2f ± %.2f m (%.2f to %.2f m)\n', ...
            results.filterStats.(filterName).summary.position.rmse_mean, ...
            results.filterStats.(filterName).summary.position.rmse_std, ...
            results.filterStats.(filterName).summary.position.rmse_min, ...
            results.filterStats.(filterName).summary.position.rmse_max);
        if isfield(results.filterStats.(filterName).summary, 'runtime')
            fprintf('  Runtime: %.3f ± %.3f s\n', ...
                results.filterStats.(filterName).summary.runtime.mean, ...
                results.filterStats.(filterName).summary.runtime.std);
        end
    end
end

results.comparison = generateFilterComparison(results.filterStats, filterTypes);

generateFilterAnalysisReport(results);

if saveResults
    saveFilterAnalysisResults(results);
end

fprintf('\n=== Filter Analysis Complete ===\n');

end

function metrics = calculateFilterPerformance(aisData, estimates, stats)
    metrics = struct();
    
    if ismember('x_true', aisData.Properties.VariableNames) && ismember('y_true', aisData.Properties.VariableNames) && isfield(estimates, 'x_est') && isfield(estimates, 'y_est')
        pos_errors = sqrt((estimates.x_est - aisData.x_true).^2 + (estimates.y_est - aisData.y_true).^2);
        metrics.position_errors = pos_errors;
        metrics.position_rmse = sqrt(mean(pos_errors.^2));
        metrics.position_mean = mean(pos_errors);
        metrics.position_std = std(pos_errors);
        metrics.position_max = max(pos_errors);
        metrics.position_min = min(pos_errors);
    end
    
    if ismember('sog_true', aisData.Properties.VariableNames) && isfield(estimates, 'velocity_est')
        vel_errors = abs(estimates.velocity_est - aisData.sog_true);
        metrics.velocity_errors = vel_errors;
        metrics.velocity_rmse = sqrt(mean(vel_errors.^2));
        metrics.velocity_mean = mean(vel_errors);
        metrics.velocity_std = std(vel_errors);
        metrics.velocity_max = max(vel_errors);
        metrics.velocity_min = min(vel_errors);
    end
    
    if ismember('cog_true', aisData.Properties.VariableNames) && isfield(estimates, 'course_est')
        course_errors = abs(angdiff(deg2rad(estimates.course_est), deg2rad(aisData.cog_true)));
        metrics.course_errors = course_errors;
        metrics.course_rmse = sqrt(mean(course_errors.^2));
        metrics.course_mean = mean(course_errors);
        metrics.course_std = std(course_errors);
    end
    
    if isfield(metrics, 'position_errors') && length(metrics.position_errors) > 10
        finalAvg = mean(metrics.position_errors(end-min(10, floor(length(metrics.position_errors)/3)):end));
        convergenceThreshold = 2 * finalAvg;
        convergenceIdx = find(metrics.position_errors < convergenceThreshold, 1);
        if ~isempty(convergenceIdx)
            metrics.convergence_time = convergenceIdx;
        end
    end
end

function comparison = generateFilterComparison(filterStats, filterTypes)
    comparison = struct();
    
    rmse_means = [];
    rmse_stds = [];
    for i = 1:length(filterTypes)
        filterName = filterTypes{i};
        if isfield(filterStats.(filterName), 'summary') && isfield(filterStats.(filterName).summary, 'position')
            rmse_means(i) = filterStats.(filterName).summary.position.rmse_mean;
            rmse_stds(i) = filterStats.(filterName).summary.position.rmse_std;
        else
            rmse_means(i) = NaN;
            rmse_stds(i) = NaN;
        end
    end
    
    comparison.position_rmse.means = rmse_means;
    comparison.position_rmse.stds = rmse_stds;
    comparison.position_rmse.filterTypes = filterTypes;
    
    validIdx = ~isnan(rmse_means);
    if any(validIdx)
        [~, bestIdx] = min(rmse_means(validIdx));
        validTypes = filterTypes(validIdx);
        comparison.best_filter = validTypes{bestIdx};
        comparison.best_rmse = min(rmse_means(validIdx));
        comparison.relative_performance = rmse_means / min(rmse_means(validIdx));
    else
        comparison.best_filter = 'None';
        comparison.best_rmse = NaN;
        comparison.relative_performance = rmse_means;
    end
end

function generateFilterAnalysisReport(results)
    figure;
    
    subplot(2, 3, 1);
    filterNames = results.comparison.position_rmse.filterTypes;
    rmse_means = results.comparison.position_rmse.means;
    rmse_stds = results.comparison.position_rmse.stds;
    
    validIdx = ~isnan(rmse_means);
    if any(validIdx)
        errorbar(find(validIdx), rmse_means(validIdx), rmse_stds(validIdx), 'o-', 'LineWidth', 2, 'MarkerSize', 8);
        set(gca, 'XTick', find(validIdx), 'XTickLabel', filterNames(validIdx));
    end
    ylabel('Position RMSE (m)');
    title('Filter Performance Comparison');
    grid on;
    
    subplot(2, 3, 2);
    allRMSE = [];
    groupLabels = [];
    for i = 1:length(filterNames)
        filterName = filterNames{i};
        if isfield(results.filterStats.(filterName), 'position_rmse')
            rmseData = results.filterStats.(filterName).position_rmse;
            if ~isempty(rmseData)
                allRMSE = [allRMSE; rmseData(:)];
                groupLabels = [groupLabels; repmat({filterName}, length(rmseData), 1)];
            end
        end
    end
    if ~isempty(allRMSE)
        boxplot(allRMSE, groupLabels);
        ylabel('Position RMSE (m)');
        title('RMSE Distribution by Filter');
    end
    
    subplot(2, 3, 3);
    runtime_means = [];
    runtime_stds = [];
    for i = 1:length(filterNames)
        filterName = filterNames{i};
        if isfield(results.filterStats.(filterName), 'summary') && isfield(results.filterStats.(filterName).summary, 'runtime')
            runtime_means(i) = results.filterStats.(filterName).summary.runtime.mean;
            runtime_stds(i) = results.filterStats.(filterName).summary.runtime.std;
        else
            runtime_means(i) = NaN;
            runtime_stds(i) = NaN;
        end
    end
    
    validIdx = ~isnan(runtime_means);
    if any(validIdx)
        errorbar(find(validIdx), runtime_means(validIdx), runtime_stds(validIdx), 's-', 'LineWidth', 2, 'MarkerSize', 8);
        set(gca, 'XTick', find(validIdx), 'XTickLabel', filterNames(validIdx));
    end
    ylabel('Runtime (s)');
    title('Computational Performance');
    grid on;
    
    subplot(2, 3, 4);
    if ~strcmp(results.comparison.best_filter, 'None')
        bestFilter = results.comparison.best_filter;
        runsToShow = min(3, size(results.rawResults, 1));
        plotCount = 0;
        for run = 1:size(results.rawResults, 1)
            for f = 1:length(filterNames)
                if strcmp(filterNames{f}, bestFilter) && ~isempty(results.rawResults{run, f})
                    if isfield(results.rawResults{run, f}, 'position_errors')
                        plot(results.rawResults{run, f}.position_errors, 'DisplayName', sprintf('Run %d', run));
                        hold on;
                        plotCount = plotCount + 1;
                        if plotCount >= runsToShow
                            break;
                        end
                    end
                    break;
                end
            end
            if plotCount >= runsToShow
                break;
            end
        end
        if plotCount > 0
            xlabel('Time Step');
            ylabel('Position Error (m)');
            title(sprintf('Error Evolution - %s Filter', bestFilter));
            legend('Location', 'best');
        end
    end
    grid on;
    
    subplot(2, 3, 5);
    validIdx = ~isnan(results.comparison.relative_performance);
    if any(validIdx)
        bar(find(validIdx), results.comparison.relative_performance(validIdx));
        set(gca, 'XTick', find(validIdx), 'XTickLabel', filterNames(validIdx));
    end
    ylabel('Relative RMSE (normalized)');
    title('Relative Performance');
    grid on;
    
    subplot(2, 3, 6);
    axis off;
    
    summaryText = {'Filter Performance Summary:', ''};
    for i = 1:length(filterNames)
        filterName = filterNames{i};
        if ~isnan(rmse_means(i))
            summaryText{end+1} = sprintf('%s:', filterName);
            summaryText{end+1} = sprintf('  RMSE: %.2f ± %.2f m', rmse_means(i), rmse_stds(i));
            if ~isnan(runtime_means(i))
                summaryText{end+1} = sprintf('  Runtime: %.3f ± %.3f s', runtime_means(i), runtime_stds(i));
            end
            summaryText{end+1} = '';
        end
    end
    summaryText{end+1} = sprintf('Best Filter: %s', results.comparison.best_filter);
    if ~isnan(results.comparison.best_rmse)
        summaryText{end+1} = sprintf('Best RMSE: %.2f m', results.comparison.best_rmse);
    end
    
    text(0.1, 0.9, summaryText, 'Units', 'normalized', 'VerticalAlignment', 'top', ...
         'FontName', 'FixedWidth', 'FontSize', 10);
    
    sgtitle(sprintf('Monte Carlo Filter Analysis (%d runs)', results.metadata.numRuns));
    
    if ~exist('output', 'dir')
        mkdir('output');
    end
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    filename = sprintf('output/filter_analysis_%s.png', timestamp);
    saveas(gcf, filename);
    fprintf('Filter analysis report saved to: %s\n', filename);
end

function saveFilterAnalysisResults(results)
    if ~exist('output', 'dir')
        mkdir('output');
    end
    
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    filename = sprintf('output/filter_analysis_results_%s.mat', timestamp);
    save(filename, 'results', '-v7.3');
    
    fprintf('Filter analysis results saved to: %s\n', filename);
    
    csvFilename = sprintf('output/filter_summary_%s.csv', timestamp);
    
    filterNames = results.comparison.position_rmse.filterTypes;
    
    summaryTable = table();
    summaryTable.Filter = filterNames';
    summaryTable.Position_RMSE_Mean = results.comparison.position_rmse.means';
    summaryTable.Position_RMSE_Std = results.comparison.position_rmse.stds';
    
    runtime_means = [];
    runtime_stds = [];
    for i = 1:length(filterNames)
        filterName = filterNames{i};
        if isfield(results.filterStats.(filterName), 'summary') && isfield(results.filterStats.(filterName).summary, 'runtime')
            runtime_means(i) = results.filterStats.(filterName).summary.runtime.mean;
            runtime_stds(i) = results.filterStats.(filterName).summary.runtime.std;
        else
            runtime_means(i) = NaN;
            runtime_stds(i) = NaN;
        end
    end
    
    summaryTable.Runtime_Mean = runtime_means';
    summaryTable.Runtime_Std = runtime_stds';
    summaryTable.Relative_Performance = results.comparison.relative_performance';
    
    writetable(summaryTable, csvFilename);
    fprintf('Summary statistics saved to: %s\n', csvFilename);
end