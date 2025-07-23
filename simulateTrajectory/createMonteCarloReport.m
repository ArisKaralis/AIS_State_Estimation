function createMonteCarloReport(results, saveDir)
% CREATEMONTECARLOREPORT - Generate comprehensive analysis report
%
% Creates a multi-panel figure with complete Monte Carlo analysis including
% error distributions, trajectory examples, mode statistics, and parameter
% correlations. Saves the report as PNG for documentation.
%
% INPUTS:
%   results - Monte Carlo results structure
%   saveDir - Directory path for saving the report

figure;

% Error distribution histograms
subplot(2, 3, 1);
plotErrorHistogram(results, 'position', 'Position Error (m)');

subplot(2, 3, 2);
plotErrorHistogram(results, 'velocity', 'Velocity Error (m/s)');

subplot(2, 3, 3);
plotErrorHistogram(results, 'course', 'Course Error (rad)');

% Trajectory examples
subplot(2, 3, 4);
plotTrajectoryExamples(results,  10);

% Motion mode statistics
subplot(2, 3, 5);
plotModeStatistics(results);

% Parameter correlation analysis
subplot(2, 3, 6);
plotErrorVsParameters(results);

% Add overall title with run count
sgtitle(sprintf('Monte Carlo Analysis (%d runs)', results.summary.numRuns));

% Save report
[filepath, name, ~] = fileparts(saveDir);
figureFile = fullfile(filepath, [name '_analysis.png']);
saveas(gcf, figureFile);

fprintf('Monte Carlo report saved to: %s\n', figureFile);

end