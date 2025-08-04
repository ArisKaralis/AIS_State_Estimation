function filter_estimates = createFilterEstimatesStruct(varargin)
% CREATEFILTERESTIMATESSTRUCT - Helper function to create filter estimates struct
%
% Usage examples:
%   % Method 1: Pass name-value pairs
%   filter_estimates = createFilterEstimatesStruct('KF', kf_est, 'EKF_CV', ekf_est_cv, ...
%                                                  'EKF_CA', ekf_est_ca, 'UKF', ukf_est, 'IMM', imm_est);
%
%   % Method 2: Pass struct directly
%   estimates.KF = kf_est;
%   estimates.EKF_CV = ekf_est_cv;
%   filter_estimates = createFilterEstimatesStruct(estimates);

if nargin == 1 && isstruct(varargin{1})
    % If a single struct is passed, use it directly
    filter_estimates = varargin{1};
elseif mod(nargin, 2) == 0
    % If even number of arguments, treat as name-value pairs
    filter_estimates = struct();
    for i = 1:2:nargin
        filter_name = varargin{i};
        filter_data = varargin{i+1};
        
        % Validate filter name is a string
        if ~ischar(filter_name) && ~isstring(filter_name)
            error('Filter name must be a string or char array');
        end
        
        % Handle both table and struct inputs
        if istable(filter_data)
            % Convert table to struct with consistent field names
            filter_struct = struct();
            required_table_columns = {'x_est', 'y_est', 'vx_est', 'vy_est', 'sog_est', 'cog_est'};
            
            % Validate table has required columns
            for j = 1:length(required_table_columns)
                if ~ismember(required_table_columns{j}, filter_data.Properties.VariableNames)
                    error('Filter %s table is missing required column: %s', filter_name, required_table_columns{j});
                end
            end
            
            % Convert table to struct
            filter_struct.x_est = filter_data.x_est;
            filter_struct.y_est = filter_data.y_est;
            filter_struct.vx_est = filter_data.vx_est;
            filter_struct.vy_est = filter_data.vy_est;
            filter_struct.sog_est = filter_data.sog_est;
            filter_struct.cog_est = filter_data.cog_est;
            
            filter_estimates.(filter_name) = filter_struct;
        else
            % Direct struct assignment - validate it has required fields
            required_fields = {'x_est', 'y_est', 'vx_est', 'vy_est', 'sog_est', 'cog_est'};
            for j = 1:length(required_fields)
                if ~isfield(filter_data, required_fields{j})
                    error('Filter %s struct is missing required field: %s', filter_name, required_fields{j});
                end
            end
            filter_estimates.(filter_name) = filter_data;
        end
    end
else
    error('Invalid number of arguments. Use name-value pairs or pass a single struct.');
end

% Validate that the struct is not empty
if isempty(fieldnames(filter_estimates))
    error('No filter estimates provided');
end

fprintf('Created filter estimates struct with %d filters:\n', length(fieldnames(filter_estimates)));
filter_names = fieldnames(filter_estimates);
for i = 1:length(filter_names)
    fprintf('  - %s\n', strrep(filter_names{i}, '_', '-'));
end

end