function params = configureSimulationParameters(config)
% CONFIGURESIMULATIONPARAMETERS - Configure simulation parameters for Monte Carlo analysis
%
% Inputs:
%   config - Configuration structure with analysis settings
%
% Outputs:
%   params - Simulation parameters structure

% Load standard simulation parameters based on real-world AIS systems
params = getDefaultSimulationParameters();

% Configure for specific testing scenario
% Temporal settings - 1-hour simulation with 10-second AIS reports
params.totalDuration = 3600;        % Total simulation time (seconds) - 1 hour
params.aisReportInterval = 10;      % AIS reporting frequency (seconds) - every 10s

% Measurement noise levels - Simulate high-quality AIS equipment
params.measurementNoise.position = 5;   % GPS position error std (meters) - good accuracy
params.measurementNoise.velocity = 0.5; % Speed measurement error std (m/s) - typical AIS

% Motion behavior - Moderate maneuvering frequency
params.modeTransition.probability = 0.03; % 3% chance per second of changing motion mode

if config.verbose
    fprintf('Configured simulation parameters:\n');
    fprintf('  Duration: %d seconds (%.1f hours)\n', params.totalDuration, params.totalDuration/3600);
    fprintf('  AIS interval: %d seconds\n', params.aisReportInterval);
    fprintf('  Position noise: %.1f meters\n', params.measurementNoise.position);
    fprintf('  Velocity noise: %.1f m/s\n', params.measurementNoise.velocity);
end

end
