function results = calculateFilterStatistics(results, config)
% CALCULATEFILTERSTATISTICS - Calculate summary statistics for all filters
%
% Inputs:
%   results - Results structure with raw filter statistics
%   config - Configuration structure
%
% Outputs:
%   results - Updated results structure with summary statistics

if config.verbose
    fprintf('\n===== Filter Analysis Results =====\n');
    fprintf('Valid runs processed: %d/%d\n', results.metadata.validRunCount, config.numRuns);
end

% Calculate summary statistics for all filters
for i = 1:length(config.filterTypes)
    filterName = config.filterTypes{i};
    results.filterStats.(filterName).summary = struct();
    
    % Calculate position statistics
    results = calculateMetricSummary(results, filterName, 'position');
    
    % Calculate velocity statistics
    results = calculateMetricSummary(results, filterName, 'velocity');
    
    % Calculate SOG statistics
    results = calculateMetricSummary(results, filterName, 'sog');
    
    % Calculate COG statistics
    results = calculateMetricSummary(results, filterName, 'cog');
    
    % Calculate runtime statistics
    results = calculateRuntimeStatistics(results, filterName);
    
    % Calculate NEES/NIS statistics
    results = calculateNeesNisStatistics(results, filterName);
    
    % Display summary if verbose
    if config.verbose
        displayFilterSummary(results, filterName);
    end
end

end

function results = calculateMetricSummary(results, filterName, metricType)
% Calculate summary statistics for a specific metric type
rmseData = results.filterStats.(filterName).([metricType '_rmse']);

if ~isempty(rmseData)
    results.filterStats.(filterName).summary.(metricType) = struct();
    results.filterStats.(filterName).summary.(metricType).rmse_mean = mean(rmseData);
    results.filterStats.(filterName).summary.(metricType).rmse_std = std(rmseData);
    results.filterStats.(filterName).summary.(metricType).rmse_min = min(rmseData);
    results.filterStats.(filterName).summary.(metricType).rmse_max = max(rmseData);
end
end

function results = calculateRuntimeStatistics(results, filterName)
% Calculate runtime statistics
runtimes = results.filterStats.(filterName).runtime;
if ~isempty(runtimes)
    results.filterStats.(filterName).summary.runtime = struct();
    results.filterStats.(filterName).summary.runtime.mean = mean(runtimes);
    results.filterStats.(filterName).summary.runtime.std = std(runtimes);
    results.filterStats.(filterName).summary.runtime.min = min(runtimes);
    results.filterStats.(filterName).summary.runtime.max = max(runtimes);
end
end

function results = calculateNeesNisStatistics(results, filterName)
% Calculate NEES and NIS summary statistics
filterData = results.filterStats.(filterName);

% NEES statistics
if isfield(filterData, 'nees_all') && ~isempty(filterData.nees_all)
    validNees = ~isnan(filterData.nees_all);
    if any(validNees)
        neesValues = filterData.nees_all(validNees);
        
        results.filterStats.(filterName).summary.nees = struct();
        results.filterStats.(filterName).summary.nees.mean = mean(neesValues);
        results.filterStats.(filterName).summary.nees.median = median(neesValues);
        results.filterStats.(filterName).summary.nees.std = std(neesValues);
        results.filterStats.(filterName).summary.nees.min = min(neesValues);
        results.filterStats.(filterName).summary.nees.max = max(neesValues);
        results.filterStats.(filterName).summary.nees.count = length(neesValues);
        
        % Calculate consistency (percentage within 95% bounds)
        alpha = 0.05;
        dof = 2; % Position only (x, y)
        lower_bound = chi2inv(alpha/2, dof);
        upper_bound = chi2inv(1-alpha/2, dof);
        within_bounds = sum(neesValues >= lower_bound & neesValues <= upper_bound);
        results.filterStats.(filterName).summary.nees.consistency = (within_bounds / length(neesValues)) * 100;
        
        % Expected value for 2D position
        results.filterStats.(filterName).summary.nees.expected = 2.0;
    end
end

% NIS statistics
if isfield(filterData, 'nis_all') && ~isempty(filterData.nis_all)
    validNis = ~isnan(filterData.nis_all);
    if any(validNis)
        nisValues = filterData.nis_all(validNis);
        
        results.filterStats.(filterName).summary.nis = struct();
        results.filterStats.(filterName).summary.nis.mean = mean(nisValues);
        results.filterStats.(filterName).summary.nis.median = median(nisValues);
        results.filterStats.(filterName).summary.nis.std = std(nisValues);
        results.filterStats.(filterName).summary.nis.min = min(nisValues);
        results.filterStats.(filterName).summary.nis.max = max(nisValues);
        results.filterStats.(filterName).summary.nis.count = length(nisValues);
        
        % Calculate consistency (percentage within 95% bounds)
        alpha = 0.05;
        dof = 2; % Measurement dimension (x, y positions)
        lower_bound = chi2inv(alpha/2, dof);
        upper_bound = chi2inv(1-alpha/2, dof);
        within_bounds = sum(nisValues >= lower_bound & nisValues <= upper_bound);
        results.filterStats.(filterName).summary.nis.consistency = (within_bounds / length(nisValues)) * 100;
        
        % Expected value for 2D measurements
        results.filterStats.(filterName).summary.nis.expected = 2.0;
    end
end
end

function displayFilterSummary(results, filterName)
% Display summary statistics for a filter
if ~isfield(results.filterStats.(filterName), 'summary')
    return;
end

fprintf('\n%s Summary:\n', filterName);
summary = results.filterStats.(filterName).summary;

if isfield(summary, 'position')
    fprintf('  Position RMSE: %.2f ± %.2f m (%.2f to %.2f m)\n', ...
        summary.position.rmse_mean, summary.position.rmse_std, ...
        summary.position.rmse_min, summary.position.rmse_max);
end

if isfield(summary, 'velocity')
    fprintf('  Velocity RMSE: %.2f ± %.2f m/s (%.2f to %.2f m/s)\n', ...
        summary.velocity.rmse_mean, summary.velocity.rmse_std, ...
        summary.velocity.rmse_min, summary.velocity.rmse_max);
end

if isfield(summary, 'sog')
    fprintf('  SOG RMSE: %.2f ± %.2f m/s (%.2f to %.2f m/s)\n', ...
        summary.sog.rmse_mean, summary.sog.rmse_std, ...
        summary.sog.rmse_min, summary.sog.rmse_max);
end

if isfield(summary, 'cog')
    fprintf('  COG RMSE: %.2f ± %.2f° (%.2f to %.2f°)\n', ...
        summary.cog.rmse_mean, summary.cog.rmse_std, ...
        summary.cog.rmse_min, summary.cog.rmse_max);
end

if isfield(summary, 'nees')
    fprintf('  NEES: %.2f ± %.2f (expected: %.1f, consistency: %.1f%%, %d values)\n', ...
        summary.nees.mean, summary.nees.std, summary.nees.expected, ...
        summary.nees.consistency, summary.nees.count);
end

if isfield(summary, 'nis')
    fprintf('  NIS: %.2f ± %.2f (expected: %.1f, consistency: %.1f%%, %d values)\n', ...
        summary.nis.mean, summary.nis.std, summary.nis.expected, ...
        summary.nis.consistency, summary.nis.count);
end

if isfield(summary, 'runtime')
    fprintf('  Runtime: %.3f ± %.3f s\n', ...
        summary.runtime.mean, summary.runtime.std);
end
end