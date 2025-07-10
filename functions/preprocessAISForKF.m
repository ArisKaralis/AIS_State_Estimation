function [data_resampled, data_original] = preprocessAISForKF(filename, targetMMSI, dt_target)
% PREPROCESSAISFORKF - Preprocess AIS data for Kalman filtering and ground truth comparison

% === Read raw AIS CSV ===
opts = detectImportOptions(filename);
opts = setvartype(opts, {'SOG', 'COG', 'LAT', 'LON'}, 'double');
data = readtable(filename, opts);

% === Filter for selected vessel ===
data = data(data.MMSI == targetMMSI, :);
data = sortrows(data, 'BaseDateTime');
data = rmmissing(data, 'DataVariables', {'LAT', 'LON', 'SOG', 'COG'});
data_original = data;  % Save for raw output

% === Save raw CSV ===
writetable(data_original, fullfile('data', ['raw_ais_' num2str(targetMMSI) '.csv']));


% === Resample to uniform time ===
fields = {'LAT', 'LON', 'SOG', 'COG'};
t_uniform = (data.BaseDateTime(1):seconds(dt_target):data.BaseDateTime(end))';
data_resampled = table();
data_resampled.BaseDateTime = t_uniform;
for i = 1:length(fields)
    f = fields{i};
    data_resampled.(f) = interp1(data.BaseDateTime, data.(f), t_uniform, 'linear', 'extrap');
end

% === Convert to UTM ===
[x, y] = deg2utm(data_resampled.LAT, data_resampled.LON);
data_resampled.x = x;
data_resampled.y = y;

% === Save smoothed + resampled CSV (8-column) ===
data_resampled_final = formatAISTable(data_resampled, targetMMSI);
writetable(data_resampled_final, fullfile('data', ['resampled_ais_' num2str(targetMMSI) '.csv']));

% === Segment and Save Moving-Only Portions ===
sog_threshold = 0.2;  % Minimum speed to be considered moving
moving_mask = data_resampled.SOG > sog_threshold;
data_moving = data_resampled(moving_mask, :);

% Convert and save moving-only as 8-column CSV
data_moving_out = formatAISTable(data_moving, targetMMSI);
writetable(data_moving_out, fullfile('data', ['resampled_moving_ais_' num2str(targetMMSI) '.csv']));

end

