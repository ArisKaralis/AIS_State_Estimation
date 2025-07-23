function plotErrorVsParameters(results)
% PLOTERRORVSPARAMETERS - Analyze relationship between noise parameters and errors
%
% Creates scatter plot or histogram showing the relationship between
% measurement noise levels and resulting position errors. If all runs use
% the same noise level, shows error distribution instead of correlation.
%
% INPUTS:
%   results - Monte Carlo results structure

posNoiseValues = [];
posErrorValues = [];

% Extract noise levels and corresponding errors from all runs
for run = 1:length(results.runs)
    try
        params = results.runs{run}.parameters;
        gt = results.runs{run}.groundTruth;
        obs = results.runs{run}.observations;
        
        % Validate observation structure
        if ~isfield(obs, 'available') || ~isfield(obs, 'groundTruthIndices')
            continue;
        end
        
        validObs = obs.available;
        numValidObs = sum(validObs);
        
        if numValidObs == 0
            continue;
        end
        
        validIndices = find(validObs);
        
        % Robust bounds checking
        if isempty(validIndices) || length(obs.groundTruthIndices) < max(validIndices)
            validIndices = validIndices(validIndices <= length(obs.groundTruthIndices));
            if isempty(validIndices)
                continue;
            end
        end
        
        gtIndices = obs.groundTruthIndices(validIndices);
        
        if isempty(gtIndices)
            continue;
        end
        
        maxGtIndex = size(gt.position, 2);
        maxObsIndex = size(obs.position, 2);
        
        % Comprehensive bounds validation
        if max(gtIndices) > maxGtIndex || max(validIndices) > maxObsIndex
            validGtMask = gtIndices <= maxGtIndex & gtIndices >= 1;
            validObsMask = validIndices <= maxObsIndex & validIndices >= 1;
            combinedMask = validGtMask & validObsMask;
            
            if sum(combinedMask) == 0
                continue;
            end
            
            finalObsIndices = validIndices(combinedMask);
            finalGtIndices = gtIndices(combinedMask);
        else
            finalObsIndices = validIndices;
            finalGtIndices = gtIndices;
        end
        
        % Final safety check
        if isempty(finalGtIndices) || isempty(finalObsIndices) || ...
           max(finalGtIndices) > maxGtIndex || max(finalObsIndices) > maxObsIndex
            continue;
        end
        
        % Calculate position error for this run
        truePos = gt.position(:, finalGtIndices);
        obsPos = obs.position(:, finalObsIndices);
        posError = sqrt(mean(sum((truePos - obsPos).^2, 1)));
        
        % Extract noise parameter (try multiple field names for robustness)
        if isfield(params, 'measurementNoise') && isfield(params.measurementNoise, 'position')
            noiseValue = params.measurementNoise.position;
        elseif isfield(params, 'measurementNoise') && isfield(params.measurementNoise, 'pos')
            noiseValue = params.measurementNoise.pos;
        elseif isfield(params, 'positionNoise')
            noiseValue = params.positionNoise;
        else
            noiseValue = 20;  % Default fallback value
        end
        
        posNoiseValues = [posNoiseValues, noiseValue];
        posErrorValues = [posErrorValues, posError];
        
    catch ME
        fprintf('Error in parameter analysis for run %d: %s\n', run, ME.message);
        continue;
    end
end

% Handle empty results gracefully
if isempty(posNoiseValues)
    bar([]);
    title('No valid data for error vs parameters plot');
    xlabel('Measurement Noise (m)');
    ylabel('Position RMSE (m)');
    text(0.5, 0.5, 'No valid Monte Carlo data available', ...
         'Units', 'normalized', 'HorizontalAlignment', 'center', ...
         'FontSize', 12, 'Color', 'red');
    return;
end

% Determine plot type based on parameter variation
uniqueNoiseValues = unique(posNoiseValues);

if length(uniqueNoiseValues) == 1
    % Single noise level - show error distribution
    histogram(posErrorValues, min(20, length(posErrorValues)));
    xlabel('Position RMSE (m)');
    ylabel('Frequency');
    title(sprintf('Position Error Distribution (Noise = %.1f m)', uniqueNoiseValues(1)));
    grid on;
    
    % Add statistics
    meanError = mean(posErrorValues);
    stdError = std(posErrorValues);
    text(0.7, 0.8, sprintf('Mean: %.2f m\nStd: %.2f m\nRuns: %d', ...
         meanError, stdError, length(posErrorValues)), ...
         'Units', 'normalized', 'BackgroundColor', 'white', ...
         'EdgeColor', 'black', 'FontSize', 10);
else
    % Multiple noise levels - show correlation
    scatter(posNoiseValues, posErrorValues, 50, 'filled');
    xlabel('Measurement Noise (m)');
    ylabel('Position RMSE (m)');
    title('Error vs Noise Level');
    grid on;
    
    % Add trend line if sufficient data points
    if length(posNoiseValues) > 5
        hold on;
        p = polyfit(posNoiseValues, posErrorValues, 1);
        x_fit = linspace(min(posNoiseValues), max(posNoiseValues), 100);
        y_fit = polyval(p, x_fit);
        builtin('plot', x_fit, y_fit, 'r--', 'LineWidth', 2);  % FIXED: Use builtin
        
        % Display correlation coefficient
        corrCoeff = corrcoef(posNoiseValues, posErrorValues);
        text(0.05, 0.95, sprintf('R² = %.3f', corrCoeff(1,2)^2), ...
             'Units', 'normalized', 'BackgroundColor', 'white', ...
             'EdgeColor', 'black', 'FontSize', 10);
        hold off;
    end
end

end