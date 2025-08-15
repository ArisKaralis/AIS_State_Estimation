function results = runMonteCarloSimulation(numRuns, simParams, varargin)
% RUNMONTECARLOSIMULATION - Main Monte Carlo simulation engine
%
% Runs multiple independent vessel trajectory simulations with randomized
% parameters to generate comprehensive datasets for filter testing and
% performance evaluation. Each run uses different vessel characteristics,
% environmental conditions, and noise levels.
%
% INPUTS:
%   numRuns   - Number of Monte Carlo runs to execute
%   simParams - Base simulation parameters (will be randomized per run)
%   varargin  - Optional: 'Seed', value for reproducible results
%
% OUTPUTS:
%   results - Structure containing all runs, statistics, and metadata

% Parse optional arguments for reproducible random seed
p = inputParser;
addParameter(p, 'Seed', [], @isnumeric);
parse(p, varargin{:});

% Set random seed for reproducibility if provided
if ~isempty(p.Results.Seed)
    seedValue = p.Results.Seed;
    rng(seedValue);
    fprintf('Running Monte Carlo simulation with %d runs (Seed: %d)...\n', numRuns, seedValue);
else
    seedValue = randi(10000);  % Random seed
    rng(seedValue);
    fprintf('Running Monte Carlo simulation with %d runs (Random seed: %d)...\n', numRuns, seedValue);
end

% Initialise results structure
results = struct();
results.runs = cell(numRuns, 1);
results.summary = struct();
results.metadata = struct();
results.metadata.seed = seedValue;
results.metadata.timestamp = datetime('now');

% Execute Monte Carlo runs
for run = 1:numRuns
    % fprintf('Run %d/%d\n', run, numRuns);
    
    % Randomize parameters for this run
    runParams = randomizeSimulationParameters(simParams);
    
    % Generate one complete trajectory with observations
    [groundTruth, observations, metadata] = generateSingleRun(runParams);
    
    % Store results for this run
    results.runs{run} = struct();
    results.runs{run}.groundTruth = groundTruth;
    results.runs{run}.observations = observations;
    results.runs{run}.metadata = metadata;
    results.runs{run}.parameters = runParams;
end

% Calculate summary statistics across all runs
results.summary = calculateMonteCarloStatistics(results);

% Save results to file
saveMonteCarloResults(results, simParams);

end