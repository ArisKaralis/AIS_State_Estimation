function [groundTruth, observations, metadata] = generateSingleRun(params)
% GENERATESINGLERUN - Creates one complete Monte Carlo simulation run
%
% Orchestrates the generation of one complete vessel trajectory with realistic
% motion and sensor measurements by combining motion modeling, mode sequencing,
% and sensor simulation.
%
% INPUTS:
%   params - Complete simulation parameter structure
%
% OUTPUTS:
%   groundTruth - Perfect vessel trajectory (what actually happened)
%   observations - Realistic sensor measurements (what AIS reported)
%   metadata - Simulation information and statistics

% Calculate simulation dimensions
numSteps = ceil(params.totalDuration / params.timeStep);
aisSteps = ceil(params.totalDuration / params.aisReportInterval);

% STEP 1: Generate motion mode sequence (CV/CA/CT transitions)
[modeSequence, switchTimes] = generateModeSequence(numSteps, params);

% STEP 2: Generate perfect ground truth trajectory
groundTruth = generateGroundTruthTrajectory(numSteps, modeSequence, params);

% STEP 3: Generate realistic sensor observations
observations = generateObservations(groundTruth, params);

% STEP 4: Collect metadata
metadata = struct();
metadata.modeSequence = modeSequence;
metadata.switchTimes = switchTimes;
metadata.numSteps = numSteps;
metadata.aisSteps = aisSteps;
metadata.actualAisReports = sum(observations.available);
metadata.timeVector = (0:numSteps-1)' * params.timeStep;
metadata.aisTimeVector = (0:aisSteps-1)' * params.aisReportInterval;

% Calculate quality metrics
metadata.dropoutRate = 1 - (metadata.actualAisReports / length(observations.available));
metadata.temporalCoverage = metadata.actualAisReports / aisSteps;

end