function saveMonteCarloResults(results, params)
% SAVEMONTECAROLORESULTS - Save Monte Carlo simulation results to file
%
% Saves complete simulation results including all runs, statistics, and
% metadata for later analysis. Creates timestamped files to avoid overwrites
% and generates a comprehensive analysis report for immediate review.
%
% FILE CONTENTS SAVED:
%   - Complete results structure with all Monte Carlo runs
%   - Original simulation parameters used
%   - Summary statistics across all runs
%   - Individual run data (ground truth, observations, metadata)
%
% OUTPUTS CREATED:
%   - .mat file with complete results (for MATLAB analysis)
%   - .png analysis report (for documentation/presentations)
%
% INPUTS:
%   results - Complete Monte Carlo results structure containing:
%     .runs     - Cell array of individual run results
%     .summary  - Aggregated statistics across all runs
%     .metadata - Simulation info (timestamps, seeds, etc.)
%   params  - Original simulation parameters used for the runs

% Create output directory if it doesn't exist
% All Monte Carlo results are organized in a dedicated output folder
outputDir = 'output';
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

% Create timestamped filename to prevent overwrites
% Format: monte_carlo_results_YYYYMMDD_HHMMSS.mat
% This ensures each simulation run creates a unique file
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
filename = sprintf('monte_carlo_results_%s.mat', timestamp);
filepath = fullfile(outputDir, filename);

% Save results with metadata using MATLAB v7.3 format
% v7.3 format supports large files (>2GB) and HDF5 compression
% Saves both results and original parameters for complete reproducibility
save(filepath, 'results', 'params', '-v7.3');

fprintf('Monte Carlo results saved to: %s\n', filepath);

% Generate comprehensive analysis report
% Creates visual summary with error distributions, trajectories, and statistics
% Automatically saves as PNG file alongside the .mat file
createMonteCarloReport(results, filepath);

end