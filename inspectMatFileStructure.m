% filepath: /Users/ariskaralis/Documents/AIS_State_Estimation/inspectMatFileStructure.m
function inspectMatFileStructure()
% INSPECTMATFILESTRUCTURE - Detailed inspection of nested .mat file structure

clear; clc;

fprintf('=== DETAILED .MAT FILE STRUCTURE INSPECTION ===\n\n');

% Load the file
mat_file = 'data/monte_carlo_results.mat';
if ~exist(mat_file, 'file')
    error('File not found: %s', mat_file);
end

fprintf('Loading: %s\n', mat_file);
data = load(mat_file);

% Get file info
file_info = dir(mat_file);
fprintf('File size: %.2f MB\n', file_info.bytes / 1024^2);
fprintf('Modified: %s\n\n', file_info.date);

% Start recursive inspection
inspectStructureRecursive(data, '', 0, 5); % Max depth of 5

fprintf('\n=== SEARCHING FOR TRAJECTORY DATA ===\n');
trajectory_paths = findTrajectoryData(data, '');

if ~isempty(trajectory_paths)
    fprintf('Found potential trajectory data at:\n');
    for i = 1:length(trajectory_paths)
        fprintf('  %s\n', trajectory_paths{i});
    end
    
    % Try to extract and display the first trajectory found
    fprintf('\n=== EXAMINING FIRST TRAJECTORY ===\n');
    first_path = trajectory_paths{1};
    try
        trajectory_data = getNestedField(data, first_path);
        examineTrajectoryData(trajectory_data, first_path);
    catch ME
        fprintf('Error accessing %s: %s\n', first_path, ME.message);
    end
else
    fprintf('No obvious trajectory data found. Showing all numeric arrays...\n');
    findNumericArrays(data, '');
end

end

function inspectStructureRecursive(s, path, depth, max_depth)
% Recursively inspect structure with detailed information

if depth > max_depth
    fprintf('%s... (max depth reached)\n', repmat('  ', 1, depth));
    return;
end

if isstruct(s)
    fields = fieldnames(s);
    if isempty(path)
        fprintf('ROOT STRUCTURE (%d fields):\n', length(fields));
    else
        fprintf('%s%s: STRUCT (%d fields)\n', repmat('  ', 1, depth), path, length(fields));
    end
    
    for i = 1:length(fields)
        field_name = fields{i};
        field_data = s.(field_name);
        current_path = [path, '.', field_name];
        if isempty(path)
            current_path = field_name;
        end
        
        % Show detailed information about this field
        showFieldInfo(field_data, current_path, depth + 1);
        
        % Recurse into structures
        if isstruct(field_data) && depth < max_depth
            inspectStructureRecursive(field_data, current_path, depth + 1, max_depth);
        elseif iscell(field_data) && depth < max_depth
            inspectCellArray(field_data, current_path, depth + 1, max_depth);
        end
    end
    
elseif iscell(s)
    fprintf('%s%s: CELL ARRAY [%s]\n', repmat('  ', 1, depth), path, num2str(size(s)));
    if depth < max_depth
        inspectCellArray(s, path, depth, max_depth);
    end
else
    showFieldInfo(s, path, depth);
end

end

function inspectCellArray(c, path, depth, max_depth)
% Inspect cell array contents

if depth > max_depth
    return;
end

[rows, cols] = size(c);
sample_size = min(3, rows * cols); % Show first 3 cells

for i = 1:sample_size
    if rows == 1
        idx = i;
        idx_str = sprintf('{%d}', i);
    elseif cols == 1
        idx = i;
        idx_str = sprintf('{%d}', i);
    else
        [r, c_idx] = ind2sub([rows, cols], i);
        idx = sub2ind([rows, cols], r, c_idx);
        idx_str = sprintf('{%d,%d}', r, c_idx);
    end
    
    cell_data = c{idx};
    cell_path = [path, idx_str];
    
    showFieldInfo(cell_data, cell_path, depth + 1);
    
    if isstruct(cell_data) && depth < max_depth
        inspectStructureRecursive(cell_data, cell_path, depth + 1, max_depth);
    end
end

if sample_size < rows * cols
    fprintf('%s... (%d more cells)\n', repmat('  ', 1, depth + 1), rows * cols - sample_size);
end

end

function showFieldInfo(data, path, depth)
% Show detailed information about a field

indent = repmat('  ', 1, depth);

if isnumeric(data)
    if isempty(data)
        fprintf('%s%s: EMPTY NUMERIC\n', indent, path);
    elseif isscalar(data)
        fprintf('%s%s: SCALAR = %.6g\n', indent, path, data);
    elseif isvector(data)
        fprintf('%s%s: VECTOR [%d] = [%.3g, %.3g, ..., %.3g]\n', indent, path, ...
               length(data), data(1), data(min(2,end)), data(end));
    else
        fprintf('%s%s: MATRIX [%s] range=[%.3g, %.3g]\n', indent, path, ...
               num2str(size(data)), min(data(:)), max(data(:)));
    end
    
elseif islogical(data)
    if isscalar(data)
        fprintf('%s%s: LOGICAL = %s\n', indent, path, mat2str(data));
    else
        fprintf('%s%s: LOGICAL [%s]\n', indent, path, num2str(size(data)));
    end
    
elseif ischar(data) || isstring(data)
    if length(data) < 50
        fprintf('%s%s: STRING = "%s"\n', indent, path, char(data));
    else
        fprintf('%s%s: STRING [%d chars] = "%s..."\n', indent, path, length(data), char(data(1:47)));
    end
    
elseif isdatetime(data)
    if isscalar(data)
        fprintf('%s%s: DATETIME = %s\n', indent, path, char(data));
    else
        fprintf('%s%s: DATETIME [%s] from %s to %s\n', indent, path, ...
               num2str(size(data)), char(data(1)), char(data(end)));
    end
    
elseif isduration(data)
    if isscalar(data)
        fprintf('%s%s: DURATION = %s\n', indent, path, char(data));
    else
        fprintf('%s%s: DURATION [%s] from %s to %s\n', indent, path, ...
               num2str(size(data)), char(data(1)), char(data(end)));
    end
    
elseif isstruct(data)
    if isscalar(data)
        fprintf('%s%s: STRUCT (fields: %s)\n', indent, path, ...
               strjoin(fieldnames(data), ', '));
    else
        fprintf('%s%s: STRUCT ARRAY [%s] (fields: %s)\n', indent, path, ...
               num2str(size(data)), strjoin(fieldnames(data), ', '));
    end
    
elseif iscell(data)
    fprintf('%s%s: CELL [%s]\n', indent, path, num2str(size(data)));
    
elseif isobject(data)
    fprintf('%s%s: OBJECT (%s)\n', indent, path, class(data));
    
else
    fprintf('%s%s: %s\n', indent, path, class(data));
end

end

function trajectory_paths = findTrajectoryData(s, current_path)
% Search for trajectory-like data in the structure

trajectory_paths = {};
trajectory_keywords = {'trajectory', 'groundtruth', 'ground_truth', 'true_states', ...
                      'position', 'velocity', 'states', 'x_true', 'path', 'track'};

if isstruct(s)
    fields = fieldnames(s);
    for i = 1:length(fields)
        field_name = fields{i};
        field_data = s.(field_name);
        
        if isempty(current_path)
            new_path = field_name;
        else
            new_path = [current_path, '.', field_name];
        end
        
        % Check if field name suggests trajectory data
        for j = 1:length(trajectory_keywords)
            if contains(lower(field_name), trajectory_keywords{j})
                trajectory_paths{end+1} = new_path;
                break;
            end
        end
        
        % Check if it's numeric data that could be trajectory
        if isnumeric(field_data) && ~isempty(field_data)
            sz = size(field_data);
            % Look for matrices that could be trajectories (time series data)
            if length(sz) == 2 && (sz(1) >= 2 && sz(2) >= 10) || (sz(2) >= 2 && sz(1) >= 10)
                trajectory_paths{end+1} = [new_path, ' [NUMERIC]'];
            end
        end
        
        % Recurse
        if isstruct(field_data)
            sub_paths = findTrajectoryData(field_data, new_path);
            trajectory_paths = [trajectory_paths, sub_paths];
        elseif iscell(field_data)
            for k = 1:min(3, numel(field_data)) % Check first 3 cells
                if isstruct(field_data{k})
                    cell_path = sprintf('%s{%d}', new_path, k);
                    sub_paths = findTrajectoryData(field_data{k}, cell_path);
                    trajectory_paths = [trajectory_paths, sub_paths];
                end
            end
        end
    end
end

end

function findNumericArrays(s, current_path)
% Find all numeric arrays in the structure

if isstruct(s)
    fields = fieldnames(s);
    for i = 1:length(fields)
        field_name = fields{i};
        field_data = s.(field_name);
        
        if isempty(current_path)
            new_path = field_name;
        else
            new_path = [current_path, '.', field_name];
        end
        
        if isnumeric(field_data) && ~isempty(field_data) && ~isscalar(field_data)
            sz = size(field_data);
            fprintf('  %s: [%s] range=[%.6g, %.6g]\n', new_path, ...
                   num2str(sz), min(field_data(:)), max(field_data(:)));
            
            % Show a sample of the data if it's reasonable size
            if numel(field_data) <= 50
                fprintf('    Sample: %s\n', mat2str(field_data, 3));
            elseif sz(1) <= 10 && sz(2) <= 10
                fprintf('    Sample:\n');
                disp(field_data);
            end
        end
        
        if isstruct(field_data)
            findNumericArrays(field_data, new_path);
        end
    end
end

end

function data = getNestedField(s, path)
% Get data from nested field path like 'results.trajectory.position'

parts = strsplit(path, '.');
data = s;

for i = 1:length(parts)
    part = parts{i};
    
    % Handle cell array indexing
    if contains(part, '{')
        field_part = extractBefore(part, '{');
        index_part = extractBetween(part, '{', '}');
        index_part = index_part{1};
        
        if ~isempty(field_part)
            data = data.(field_part);
        end
        
        if contains(index_part, ',')
            indices = str2num(['[' index_part ']']);
            data = data{indices(1), indices(2)};
        else
            idx = str2double(index_part);
            data = data{idx};
        end
    else
        data = data.(part);
    end
end

end

function examineTrajectoryData(trajectory_data, path)
% Examine trajectory data in detail

fprintf('Examining trajectory data at: %s\n', path);
fprintf('Type: %s\n', class(trajectory_data));
fprintf('Size: [%s]\n', num2str(size(trajectory_data)));

if isstruct(trajectory_data)
    fields = fieldnames(trajectory_data);
    fprintf('Structure fields (%d):\n', length(fields));
    
    for i = 1:length(fields)
        field_name = fields{i};
        field_data = trajectory_data.(field_name);
        
        if isnumeric(field_data)
            fprintf('  %s: [%s] range=[%.6g, %.6g]\n', field_name, ...
                   num2str(size(field_data)), min(field_data(:)), max(field_data(:)));
            
            % Show first few values
            if isvector(field_data) && length(field_data) >= 3
                fprintf('    First 3: [%.6g, %.6g, %.6g]\n', field_data(1), field_data(2), field_data(3));
            elseif size(field_data, 1) >= 2 && size(field_data, 2) >= 3
                fprintf('    First column: [%.6g; %.6g; ...]\n', field_data(1,1), field_data(2,1));
            end
        else
            fprintf('  %s: %s [%s]\n', field_name, class(field_data), num2str(size(field_data)));
        end
    end
    
elseif isnumeric(trajectory_data)
    fprintf('Numeric data analysis:\n');
    fprintf('  Min value: %.6g\n', min(trajectory_data(:)));
    fprintf('  Max value: %.6g\n', max(trajectory_data(:)));
    fprintf('  Mean: %.6g\n', mean(trajectory_data(:)));
    fprintf('  Std: %.6g\n', std(trajectory_data(:)));
    
    % Show structure if it looks like [states x time] or [time x states]
    [rows, cols] = size(trajectory_data);
    if rows <= 10 && cols > 10
        fprintf('  Likely format: [%d states x %d time_steps]\n', rows, cols);
        fprintf('  First few time steps:\n');
        disp(trajectory_data(:, 1:min(5, cols)));
    elseif cols <= 10 && rows > 10
        fprintf('  Likely format: [%d time_steps x %d states]\n', rows, cols);
        fprintf('  First few time steps:\n');
        disp(trajectory_data(1:min(5, rows), :));
    end
end

end