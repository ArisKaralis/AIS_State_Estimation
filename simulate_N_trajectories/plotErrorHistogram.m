function plotErrorHistogram(results, errorType, titleStr)
% PLOTERRORHISTOGRAM - Display distribution of errors across Monte Carlo runs
%
% Creates histogram showing the distribution of position, velocity, or course
% errors across all Monte Carlo runs. This helps characterize the noise
% model performance and identify any systematic biases.
%
% INPUTS:
%   results   - Monte Carlo results structure
%   errorType - 'position', 'velocity', or 'course'
%   titleStr  - Title string for the plot

allErrors = [];

% Collect errors from all Monte Carlo runs
for run = 1:length(results.runs)
    try
        gt = results.runs{run}.groundTruth;
        obs = results.runs{run}.observations;
        
        % Validate data structure
        if ~isfield(obs, 'available') || ~isfield(obs, 'groundTruthIndices')
            continue;
        end
        
        validObs = obs.available;
        numValidObs = sum(validObs);
        
        if numValidObs == 0
            continue;
        end
        
        validIndices = find(validObs);
        
        % Robust bounds checking for observation indices
        if length(obs.groundTruthIndices) < max(validIndices)
            validIndices = validIndices(validIndices <= length(obs.groundTruthIndices));
            if isempty(validIndices)
                continue;
            end
        end
        
        gtIndices = obs.groundTruthIndices(validIndices);
        
        % Ensure ground truth indices are valid
        maxGtIndex = size(gt.position, 2);
        validGtMask = gtIndices <= maxGtIndex & gtIndices >= 1;
        
        if sum(validGtMask) == 0
            continue;
        end
        
        finalValidIndices = validIndices(validGtMask);
        finalGtIndices = gtIndices(validGtMask);
        
        % Final bounds check
        if max(finalGtIndices) > size(gt.position, 2) || max(finalValidIndices) > size(obs.position, 2)
            continue;
        end
        
        % Calculate errors based on type
        switch errorType
            case 'position'
                % Euclidean distance between true and observed positions
                trueVals = gt.position(:, finalGtIndices);
                obsVals = obs.position(:, finalValidIndices);
                errors = sqrt(sum((trueVals - obsVals).^2, 1));
                
            case 'velocity'
                % Speed magnitude error
                if max(finalGtIndices) > length(gt.speed) || max(finalValidIndices) > length(obs.velocity)
                    continue;
                end
                trueVals = gt.speed(finalGtIndices);
                obsVals = obs.velocity(finalValidIndices);
                errors = abs(trueVals - obsVals');
                
            case 'course'
                % Angular difference (handles wrap-around)
                if ~isfield(gt, 'heading') || ~isfield(obs, 'course')
                    continue;
                end
                if max(finalGtIndices) > length(gt.heading) || max(finalValidIndices) > length(obs.course)
                    continue;
                end
                trueVals = gt.heading(finalGtIndices);
                obsVals = obs.course(finalValidIndices);
                
                % Ensure consistent vector orientation
                trueVals = trueVals(:);
                obsVals = obsVals(:);
                
                % Calculate angular difference with proper wrap-around
                errors = abs(angdiff(trueVals, obsVals));
        end
        
        allErrors = [allErrors, errors(:)'];
        
    catch ME
        fprintf('Error in histogram calculation for run %d: %s\n', run, ME.message);
        continue;
    end
end

% Create histogram or show message if no data
if isempty(allErrors)
    bar(0);
    title('No valid data for histogram');
    xlabel(titleStr);
    ylabel('Frequency');
    return;
end

% Plot error distribution
histogram(allErrors, 30);
xlabel(titleStr);
ylabel('Frequency');
title(sprintf('%s Distribution', titleStr));
grid on;

% Add comprehensive statistics text including min and max
meanError = mean(allErrors);
stdError = std(allErrors);
minError = min(allErrors);
maxError = max(allErrors);

text(0.7, 0.8, sprintf('Mean: %.3f\nStd: %.3f\nMin: %.3f\nMax: %.3f\nSamples: %d', ...
     meanError, stdError, minError, maxError, length(allErrors)), ...
     'Units', 'normalized', 'BackgroundColor', 'white', ...
     'EdgeColor', 'black', 'FontSize', 10);

end