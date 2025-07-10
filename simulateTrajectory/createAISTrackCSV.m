function createAISTrackCSV()
    % Check if the CSV already exists
    csvPath = fullfile('simulate_data', 'specific_ais_movement.csv');
    
    if exist(csvPath, 'file')
        % File exists, load it to verify contents
        data = readtable(csvPath);
        fprintf('CSV file already exists at: %s\n', csvPath);
        fprintf('Contains %d rows with the following columns:\n', height(data));
        disp(data.Properties.VariableNames);
        
        % Verify it has both noisy data and ground truth
        hasNoisyData = all(ismember({'x', 'y', 'SOG', 'COG'}, data.Properties.VariableNames));
        hasGroundTruth = all(ismember({'x_true', 'y_true', 'vx_true', 'vy_true', 'sog_true', 'cog_true'}, ...
                                     data.Properties.VariableNames));
        
        if hasNoisyData && hasGroundTruth
            fprintf('CSV contains both noisy measurements and ground truth data.\n');
            
            % Display a small sample
            fprintf('\nFirst 5 rows of data:\n');
            disp(head(data, 5));
        else
            fprintf('CSV appears incomplete. Missing some expected columns.\n');
            fprintf('Creating a new CSV file with complete data...\n');
            
            % Run simulation again to create complete CSV
            simulateSpecificAISTrack();
        end
    else
        % CSV doesn't exist, create it
        fprintf('CSV file not found. Creating it now...\n');
        
        % Ensure directory exists
        if ~exist('simulate_data', 'dir')
            mkdir('simulate_data');
        end
        
        % Run simulation to create CSV
        simulateSpecificAISTrack();
    end
    
    % Provide instructions for accessing the data
    fprintf('\nTo load this data in MATLAB, use:\n');
    fprintf('data = readtable(''%s'');\n', csvPath);
    
    % Additional export options
    fprintf('\nWould you like to export this data in other formats?\n');
    fprintf('For Excel: writematrix(data, ''simulate_data/specific_ais_movement.xlsx'');\n');
    fprintf('For MAT file: save(''simulate_data/specific_ais_movement.mat'', ''data'');\n');
end