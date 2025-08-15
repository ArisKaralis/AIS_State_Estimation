function createAISTrackCSV()
% CREATEAISTRACKCSV - Create or verify AIS trajectory CSV file
%
% Ensures that a properly formatted AIS dataset exists for filter testing.
% Checks if file already exists and validates its contents, or creates
% a new one if needed.

csvPath = fullfile('data', 'specific_ais_movement.csv');

if exist(csvPath, 'file')
    % File exists - validate contents
    data = readtable(csvPath);
    fprintf('CSV file already exists at: %s\n', csvPath);
    fprintf('Contains %d rows with %d columns\n', height(data), width(data));
    
    % Check for required fields
    requiredNoisy = {'x', 'y', 'SOG', 'COG'};
    requiredTruth = {'x_true', 'y_true', 'vx_true', 'vy_true', 'sog_true', 'cog_true'};
    
    hasNoisyData = all(ismember(requiredNoisy, data.Properties.VariableNames));
    hasGroundTruth = all(ismember(requiredTruth, data.Properties.VariableNames));
    
    if hasNoisyData && hasGroundTruth
        fprintf('✓ CSV contains both noisy measurements and ground truth data\n');
        
        % Display sample of data
        fprintf('\nFirst 5 rows:\n');
        disp(head(data, 5));
        
        % Show data quality metrics
        if ismember('pos_error', data.Properties.VariableNames)
            fprintf('\nData Quality:\n');
            fprintf('  Position errors: %.2f to %.2f m (mean: %.2f m)\n', ...
                min(data.pos_error), max(data.pos_error), mean(data.pos_error));
        end
    else
        fprintf('⚠ CSV appears incomplete - regenerating...\n');
        generateNewCSV();
    end
else
    fprintf('CSV file not found - creating new one...\n');
    generateNewCSV();
end

% Provide usage instructions
fprintf('\nUsage:\n');
fprintf('  data = readtable(''%s'');\n', csvPath);
fprintf('  Export options:\n');
fprintf('    Excel: writetable(data, ''data/ais_data.xlsx'');\n');
fprintf('    MAT:   save(''data/ais_data.mat'', ''data'');\n');

end

function generateNewCSV()
% Generate new AIS dataset using simulation
    if ~exist('data', 'dir')
        mkdir('data');
    end
    
    % Run simulation to create comprehensive dataset
    if exist('simulateSpecificAISTrack.m', 'file')
        simulateSpecificAISTrack();
    else
        fprintf('Error: simulateSpecificAISTrack.m not found\n');
        fprintf('Please ensure all simulation functions are available\n');
    end
end