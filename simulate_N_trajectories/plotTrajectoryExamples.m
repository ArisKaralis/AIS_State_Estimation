function plotTrajectoryExamples(results, numExamples)
% PLOTTRAJECTORYEXAMPLES - Display sample trajectories from Monte Carlo results
%
% Creates a plot showing ground truth trajectories and corresponding noisy
% observations from multiple Monte Carlo runs. This helps visualize the
% quality of simulation data and the relationship between perfect and
% sensor measurements.

% REMOVED: figure; - This was creating a new figure and breaking the subplot
hold on;

% Prepare for plotting multiple trajectories with different colors
colors = lines(numExamples);
numRuns = min(numExamples, length(results.runs));

validPlots = 0;

% Plot each trajectory example
for i = 1:numRuns
    try
        gt = results.runs{i}.groundTruth;
        obs = results.runs{i}.observations;
        
        % Skip empty trajectories
        if isempty(gt) || isempty(obs)
            continue;
        end
        
        if size(gt.position, 2) == 0
            continue;
        end
        
        % Plot ground truth trajectory (smooth line)
        builtin('plot', gt.position(1, :), gt.position(2, :), '-', 'Color', colors(i, :), ...
             'LineWidth', 1.5, 'DisplayName', sprintf('Ground Truth %d', i));
        
        % Plot noisy observations (scattered points)
        validObs = obs.available;
        if sum(validObs) > 0
            validIndices = find(validObs);
            
            % Check bounds to prevent indexing errors
            maxObsSize = size(obs.position, 2);
            validIndices = validIndices(validIndices <= maxObsSize);
            
            if ~isempty(validIndices)
                obsX = obs.position(1, validIndices);
                obsY = obs.position(2, validIndices);
                scatter(obsX, obsY, 20, colors(i, :), 'filled', 'MarkerFaceAlpha', 0.6, ...
                        'DisplayName', sprintf('Observations %d', i));
                validPlots = validPlots + 1;
            end
        end
        
    catch ME
        fprintf('Error plotting trajectory %d: %s\n', i, ME.message);
    end
end

% Handle case where no valid trajectories were plotted
if validPlots == 0
    builtin('plot', 0, 0, 'k.');
    text(0, 0, 'No valid trajectories to display', 'HorizontalAlignment', 'center');
end

% Format plot
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Sample Trajectories');
grid on;
axis equal;

if validPlots > 0
    legend('show', 'Location', 'best');
end

end