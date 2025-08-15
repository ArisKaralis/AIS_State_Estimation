function comparison = generateFilterComparison(filterStats, filterTypes)
% GENERATEFILTERCOMPARISON - Generate comprehensive comparison statistics
% with proper confidence intervals

comparison = struct();
comparison.filterTypes = filterTypes;

% Initialize comparison structure with count tracking
comparison.position_rmse = struct('means', [], 'stds', [], 'counts', []);
comparison.velocity_rmse = struct('means', [], 'stds', [], 'counts', []);
comparison.sog_rmse = struct('means', [], 'stds', [], 'counts', []);
comparison.cog_rmse = struct('means', [], 'stds', [], 'counts', []);

comparison.runtime = struct('means', [], 'stds', [], 'counts', []);

% Extract statistics for each filter
for i = 1:length(filterTypes)
    filterName = filterTypes{i};
    
    % Position RMSE statistics
    if isfield(filterStats, filterName) && isfield(filterStats.(filterName), 'position_rmse')
        values = filterStats.(filterName).position_rmse;
        valid_values = values(~isnan(values));
        if ~isempty(valid_values)
            comparison.position_rmse.means(i) = mean(valid_values);
            comparison.position_rmse.stds(i) = std(valid_values);
            comparison.position_rmse.counts(i) = length(valid_values);
        else
            comparison.position_rmse.means(i) = NaN;
            comparison.position_rmse.stds(i) = NaN;
            comparison.position_rmse.counts(i) = 0;
        end
    else
        comparison.position_rmse.means(i) = NaN;
        comparison.position_rmse.stds(i) = NaN;
        comparison.position_rmse.counts(i) = 0;
    end
    
    % Velocity RMSE statistics
    if isfield(filterStats, filterName) && isfield(filterStats.(filterName), 'velocity_rmse')
        values = filterStats.(filterName).velocity_rmse;
        valid_values = values(~isnan(values));
        if ~isempty(valid_values)
            comparison.velocity_rmse.means(i) = mean(valid_values);
            comparison.velocity_rmse.stds(i) = std(valid_values);
            comparison.velocity_rmse.counts(i) = length(valid_values);
        else
            comparison.velocity_rmse.means(i) = NaN;
            comparison.velocity_rmse.stds(i) = NaN;
            comparison.velocity_rmse.counts(i) = 0;
        end
    else
        comparison.velocity_rmse.means(i) = NaN;
        comparison.velocity_rmse.stds(i) = NaN;
        comparison.velocity_rmse.counts(i) = 0;
    end
    
    % SOG RMSE statistics
    if isfield(filterStats, filterName) && isfield(filterStats.(filterName), 'sog_rmse')
        values = filterStats.(filterName).sog_rmse;
        valid_values = values(~isnan(values));
        if ~isempty(valid_values)
            comparison.sog_rmse.means(i) = mean(valid_values);
            comparison.sog_rmse.stds(i) = std(valid_values);
            comparison.sog_rmse.counts(i) = length(valid_values);
        else
            comparison.sog_rmse.means(i) = NaN;
            comparison.sog_rmse.stds(i) = NaN;
            comparison.sog_rmse.counts(i) = 0;
        end
    else
        comparison.sog_rmse.means(i) = NaN;
        comparison.sog_rmse.stds(i) = NaN;
        comparison.sog_rmse.counts(i) = 0;
    end
    
    % COG RMSE statistics
    if isfield(filterStats, filterName) && isfield(filterStats.(filterName), 'cog_rmse')
        values = filterStats.(filterName).cog_rmse;
        valid_values = values(~isnan(values));
        if ~isempty(valid_values)
            comparison.cog_rmse.means(i) = mean(valid_values);
            comparison.cog_rmse.stds(i) = std(valid_values);
            comparison.cog_rmse.counts(i) = length(valid_values);
        else
            comparison.cog_rmse.means(i) = NaN;
            comparison.cog_rmse.stds(i) = NaN;
            comparison.cog_rmse.counts(i) = 0;
        end
    else
        comparison.cog_rmse.means(i) = NaN;
        comparison.cog_rmse.stds(i) = NaN;
        comparison.cog_rmse.counts(i) = 0;
    end
    
    % Runtime statistics
    if isfield(filterStats, filterName) && isfield(filterStats.(filterName), 'runtime')
        values = filterStats.(filterName).runtime;
        valid_values = values(~isnan(values));
        if ~isempty(valid_values)
            comparison.runtime.means(i) = mean(valid_values);
            comparison.runtime.stds(i) = std(valid_values);
            comparison.runtime.counts(i) = length(valid_values);
        else
            comparison.runtime.means(i) = NaN;
            comparison.runtime.stds(i) = NaN;
            comparison.runtime.counts(i) = 0;
        end
    else
        comparison.runtime.means(i) = NaN;
        comparison.runtime.stds(i) = NaN;
        comparison.runtime.counts(i) = 0;
    end
end

% Find best performers (minimum RMSE)
[~, best_pos_idx] = min(comparison.position_rmse.means);
[~, best_vel_idx] = min(comparison.velocity_rmse.means);
[~, best_sog_idx] = min(comparison.sog_rmse.means);
[~, best_cog_idx] = min(comparison.cog_rmse.means);

comparison.best_filters = struct();
if ~isempty(best_pos_idx) && ~isnan(comparison.position_rmse.means(best_pos_idx))
    comparison.best_filters.position = filterTypes{best_pos_idx};
end
if ~isempty(best_vel_idx) && ~isnan(comparison.velocity_rmse.means(best_vel_idx))
    comparison.best_filters.velocity = filterTypes{best_vel_idx};
end
if ~isempty(best_sog_idx) && ~isnan(comparison.sog_rmse.means(best_sog_idx))
    comparison.best_filters.sog = filterTypes{best_sog_idx};
end
if ~isempty(best_cog_idx) && ~isnan(comparison.cog_rmse.means(best_cog_idx))
    comparison.best_filters.cog = filterTypes{best_cog_idx};
end

% Overall best filter (based on position RMSE as primary criterion)
comparison.best_rmse = min(comparison.position_rmse.means);
if ~isnan(comparison.best_rmse)
    comparison.best_filter = filterTypes{best_pos_idx};
else
    comparison.best_filter = 'None';
    comparison.best_rmse = NaN;
end

% Calculate statistical significance for pairwise comparisons
% (This could be expanded with t-tests between filters)
comparison.statistical_tests = struct();
for i = 1:length(filterTypes)
    for j = i+1:length(filterTypes)
        filter1 = filterTypes{i};
        filter2 = filterTypes{j};
        
        % Position RMSE comparison
        if isfield(filterStats, filter1) && isfield(filterStats, filter2) && ...
           isfield(filterStats.(filter1), 'position_rmse') && isfield(filterStats.(filter2), 'position_rmse')
            
            values1 = filterStats.(filter1).position_rmse;
            values2 = filterStats.(filter2).position_rmse;
            
            valid1 = values1(~isnan(values1));
            valid2 = values2(~isnan(values2));
            
            if length(valid1) > 1 && length(valid2) > 1
                % Perform two-sample t-test
                [h, p] = ttest2(valid1, valid2);
                test_name = sprintf('%s_vs_%s_position', filter1, filter2);
                comparison.statistical_tests.(test_name) = struct('h', h, 'p', p, 'significant', h == 1);
            end
        end
    end
end

end