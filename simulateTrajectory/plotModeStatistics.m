function plotModeStatistics(results)
% PLOTMODESTATISTICS - Visualize motion mode usage across Monte Carlo runs
%
% Creates a bar chart showing how much time was spent in each motion mode
% (CV, CA, CT) across all Monte Carlo runs. This helps verify that the
% mode generation is working correctly and shows the operational profile.
%
% INPUTS:
%   results - Monte Carlo results structure

modes = {'CV', 'CA', 'CT'};  % Constant Velocity, Constant Acceleration, Constant Turn
modeCounts = zeros(1, 3);

% Count mode usage across all runs
for run = 1:length(results.runs)
    try
        modeSeq = results.runs{run}.groundTruth.mode;
        
        % Count occurrences of each mode
        for i = 1:3
            modeCounts(i) = modeCounts(i) + sum(strcmp(modeSeq, modes{i}));
        end
    catch
        % Skip runs with missing or invalid mode data
        continue;
    end
end

% Create bar chart
bar(modeCounts);
set(gca, 'XTickLabel', modes);
xlabel('Motion Mode');
ylabel('Total Time Steps');
title('Mode Usage Statistics');
grid on;

% Add percentage labels on bars
totalSteps = sum(modeCounts);
if totalSteps > 0
    for i = 1:3
        percentage = 100 * modeCounts(i) / totalSteps;
        text(i, modeCounts(i) + totalSteps*0.02, sprintf('%.1f%%', percentage), ...
             'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    end
end

end