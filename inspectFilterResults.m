% filepath: inspectFilterResults.m
function inspectFilterResults()
    % Find the most recent filter results file
    filter_files = dir('output/best_filters_results_*.mat');
    if isempty(filter_files)
        fprintf('No filter results files found in output/ directory\n');
        return;
    end
    
    % Sort by date and get the most recent
    [~, idx] = sort([filter_files.datenum], 'descend');
    latest_file = fullfile('output', filter_files(idx(1)).name);
    
    fprintf('Inspecting: %s\n', latest_file);
    fprintf('=================================\n');
    
    % Load and inspect the structure
    data = load(latest_file);
    
    fprintf('Top-level fields:\n');
    fields = fieldnames(data);
    for i = 1:length(fields)
        field_name = fields{i};
        field_value = data.(field_name);
        fprintf('  %s: %s\n', field_name, class(field_value));
        
        if isstruct(field_value)
            fprintf('    Sub-fields: %s\n', strjoin(fieldnames(field_value), ', '));
        elseif iscell(field_value)
            fprintf('    Cell array size: [%s]\n', num2str(size(field_value)));
        elseif isnumeric(field_value)
            fprintf('    Numeric array size: [%s]\n', num2str(size(field_value)));
        end
    end
    
    % Look for common filter result patterns
    if isfield(data, 'results')
        fprintf('\nInspecting results structure:\n');
        inspectStructure(data.results, 'results');
    end
    
    if isfield(data, 'comparison')
        fprintf('\nInspecting comparison structure:\n');
        inspectStructure(data.comparison, 'comparison');
    end
    
    if isfield(data, 'filterTypes')
        fprintf('\nFilter types found: %s\n', strjoin(data.filterTypes, ', '));
    end
    
    % Look for RMSE data
    fprintf('\nLooking for RMSE data patterns...\n');
    searchForRMSEData(data, '');
end

function inspectStructure(s, prefix)
    if ~isstruct(s)
        return;
    end
    
    fields = fieldnames(s);
    for i = 1:min(10, length(fields))  % Limit to first 10 fields
        field_name = fields{i};
        field_value = s.(field_name);
        full_name = [prefix '.' field_name];
        
        if isstruct(field_value)
            fprintf('  %s: struct with fields [%s]\n', full_name, strjoin(fieldnames(field_value), ', '));
            if length(fieldnames(field_value)) <= 5
                inspectStructure(field_value, full_name);
            end
        elseif iscell(field_value)
            fprintf('  %s: cell [%s]\n', full_name, num2str(size(field_value)));
        elseif isnumeric(field_value)
            fprintf('  %s: numeric [%s]\n', full_name, num2str(size(field_value)));
        else
            fprintf('  %s: %s\n', full_name, class(field_value));
        end
    end
    
    if length(fields) > 10
        fprintf('  ... and %d more fields\n', length(fields) - 10);
    end
end

function searchForRMSEData(data, prefix)
    if isstruct(data)
        fields = fieldnames(data);
        for i = 1:length(fields)
            field_name = fields{i};
            if contains(lower(field_name), {'rmse', 'error', 'performance'})
                full_name = [prefix '.' field_name];
                fprintf('  Found: %s (%s)\n', full_name, class(data.(field_name)));
                if isnumeric(data.(field_name))
                    fprintf('    Size: [%s], Range: [%.3f to %.3f]\n', ...
                        num2str(size(data.(field_name))), ...
                        min(data.(field_name)(:)), max(data.(field_name)(:)));
                end
            end
            if isstruct(data.(field_name))
                searchForRMSEData(data.(field_name), [prefix '.' field_name]);
            end
        end
    end
end