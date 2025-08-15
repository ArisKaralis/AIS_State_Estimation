function runMonteCarloFilterAnalysis(varargin)
% RUNMONTECARLOFILTERANALYSIS - Comprehensive Monte Carlo analysis of best performing filters
%
% This function runs Monte Carlo simulations to statistically evaluate the performance
% of the three best performing filters: 2-model IMM, 3-model IMM, and EKF-CTRV.
% It generates multiple trajectory types and analyzes filter robustness and consistency.
%
% USAGE:
%   results = runMonteCarloFilterAnalysis()  % Use defaults
%   results = runMonteCarloFilterAnalysis('NumRuns', 100, 'FilterTypes', {'KF', 'EKF_CV', 'EKF_CA', 'EKF_CTRV', 'UKF_CV', 'UKF_CTRV', 'IMM_2', 'IMM_3'})
%
% OPTIONAL PARAMETERS:
%   'NumRuns'       - Number of Monte Carlo runs (default: 50)
%   'FilterTypes'   - Cell array of filter names (default: {'IMM_2', 'IMM_3', 'EKF_CTRV'})
%   'SaveResults'   - Save results to files (default: true)
%   'Seed'          - Random seed for reproducibility (default: 42)
%   'Verbose'       - Display progress information (default: true)

% Parse input arguments
p = inputParser;
addParameter(p, 'NumRuns', 100, @isnumeric);
addParameter(p, 'FilterTypes', {'KF', 'EKF_CV', 'EKF_CA', 'EKF_CTRV', 'UKF_CV', 'UKF_CTRV', 'IMM_2', 'IMM_3'}, @iscell);
addParameter(p, 'SaveResults', true, @islogical);
addParameter(p, 'Seed', 42, @isnumeric);
addParameter(p, 'Verbose', true, @islogical);
parse(p, varargin{:});

config = struct();
config.numRuns = p.Results.NumRuns;
config.filterTypes = p.Results.FilterTypes;
config.saveResults = p.Results.SaveResults;
config.seedValue = p.Results.Seed;
config.verbose = p.Results.Verbose;

% Add required paths
addpath('simulate_N_trajectories');
addpath('functions');
addpath('monte_carlo');

if config.verbose
    fprintf('\n========= Monte Carlo Filter Analysis for Best Filters =========\n\n');
    fprintf('Number of runs: %d\n', config.numRuns);
    fprintf('Filter types: %s\n', strjoin(config.filterTypes, ', '));
    fprintf('Random seed: %d\n', config.seedValue);
end

% STEP 1: CONFIGURE SIMULATION PARAMETERS
params = configureSimulationParameters(config);

% STEP 2: RUN MONTE CARLO SIMULATION
fprintf('Starting Monte Carlo simulation...\n');
monteCarloResults = runMonteCarloSimulation(config.numRuns, params, 'Seed', config.seedValue);

% STEP 3: PROCESS MONTE CARLO RUNS AND EXECUTE FILTER ANALYSIS
[results, measurementStats] = processMonteCarloRuns(monteCarloResults, config);

% STEP 4: CALCULATE SUMMARY STATISTICS
results = calculateFilterStatistics(results, config);

% STEP 5: ANALYZE AND COMPARE RESULTS
results = analyzeResults(results, measurementStats, config);

if config.verbose
    fprintf('\n===== Filter Analysis Complete =====\n');
end

end