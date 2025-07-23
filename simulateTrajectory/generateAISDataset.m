function aisData = generateAISDataset(groundTruth, observations, metadata)
% GENERATEAISDATASET - Creates AIS-format CSV data from simulation results
%
% Converts simulation results into standard AIS message format compatible
% with maritime tracking systems and analysis tools. Includes both the
% noisy observations (what was measured) and ground truth (what actually happened)
% for filter performance evaluation.
%
% INPUTS:
%   groundTruth  - Perfect trajectory from simulation
%   observations - Noisy sensor measurements  
%   metadata     - Simulation information
%
% OUTPUTS:
%   aisData - Table with standard AIS fields plus ground truth and error analysis

% Extract only valid (non-dropped) observations
validObs = observations.available;
numValidObs = sum(validObs);

% Handle case of no valid observations
if numValidObs == 0
    aisData = table();
    return;
end

% Get indices for valid observations and corresponding ground truth
validIndices = find(validObs);
gtIndices = observations.groundTruthIndices(validIndices);

% Ensure ground truth indices are within bounds
maxGtIndex = size(groundTruth.position, 2);
validGtMask = gtIndices <= maxGtIndex;

if sum(validGtMask) == 0
    aisData = table();
    return;
end

% Filter to final valid set
finalValidIndices = validIndices(validGtMask);
finalGtIndices = gtIndices(validGtMask);

% Create timestamps for AIS reports
timestamps = datetime('now') + seconds(observations.time(finalValidIndices));

% Get motion mode labels and create segment numbers
modeLabels = groundTruth.mode(finalGtIndices);

% Create segment numbers - increment counter each time motion mode changes
segmentNumbers = ones(length(modeLabels), 1);
currentSegment = 1;

for i = 2:length(modeLabels)
    if ~strcmp(modeLabels{i}, modeLabels{i-1})
        currentSegment = currentSegment + 1;
    end
    segmentNumbers(i) = currentSegment;
end

% Create segment names with more descriptive labels
segmentNames = cell(length(modeLabels), 1);
for i = 1:length(modeLabels)
    switch modeLabels{i}
        case 'CV'
            segmentNames{i} = sprintf('CV');
        case 'CA'
            segmentNames{i} = 'CA';
        case 'CT'
            segmentNames{i} = 'CT';
        otherwise
            segmentNames{i} = modeLabels{i}; % fallback to original label
    end
end

% Build AIS dataset table
aisData = table();

% STANDARD AIS FIELDS - What AIS systems typically report
aisData.timestamp = timestamps;                                      % UTC timestamp
aisData.x = observations.position(1, finalValidIndices)';           % Longitude proxy (meters)
aisData.y = observations.position(2, finalValidIndices)';           % Latitude proxy (meters)
aisData.SOG = observations.velocity(finalValidIndices);             % Speed Over Ground (m/s)
aisData.COG = rad2deg(observations.course(finalValidIndices));      % Course Over Ground (degrees)

% GROUND TRUTH FIELDS - Perfect values for comparison
aisData.x_true = groundTruth.position(1, finalGtIndices)';          % True position X
aisData.y_true = groundTruth.position(2, finalGtIndices)';          % True position Y
aisData.vx_true = groundTruth.velocity(1, finalGtIndices)';         % True velocity X component
aisData.vy_true = groundTruth.velocity(2, finalGtIndices)';         % True velocity Y component
aisData.sog_true = groundTruth.speed(finalGtIndices);               % True speed
aisData.cog_true = rad2deg(groundTruth.heading(finalGtIndices));    % True heading

% ERROR ANALYSIS FIELDS - For filter performance evaluation
aisData.pos_error = sqrt((aisData.x - aisData.x_true).^2 + ...     % Position error magnitude
                        (aisData.y - aisData.y_true).^2);
aisData.sog_error = abs(aisData.SOG - aisData.sog_true);           % Speed error magnitude
aisData.cog_error = abs(angdiff(deg2rad(aisData.COG), ...          % Heading error (wrapped)
                               deg2rad(aisData.cog_true)));

% SEGMENT FIELDS - Motion mode tracking
aisData.segment = segmentNumbers;                                   % Numeric segment counter
aisData.segment_name = segmentNames;                               % Descriptive segment name

% VESSEL IDENTIFIER - Unique MMSI number
mmsi = randi([100000000, 999999999]);  % Random 9-digit MMSI
aisData.MMSI = repmat(mmsi, height(aisData), 1);

end