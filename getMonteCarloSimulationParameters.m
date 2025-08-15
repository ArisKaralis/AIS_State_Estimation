function params = getMonteCarloSimulationParameters()
% GETMONTECARLOSIMULATIONPARAMETERS - Optimized parameters for filter testing
%
% Returns simulation parameters specifically tuned for comprehensive
% filter performance evaluation with realistic AIS-like conditions.
addpath('simulate_N_trajectories');

% Get base parameters and modify for comprehensive testing
params = getDefaultSimulationParameters();

% Extended simulation for comprehensive testing
params.totalDuration = 3600;        % 1 hour simulation
params.aisReportInterval = 10;      % 10-second intervals (typical AIS Class A)

% Realistic measurement noise levels for filter testing
params.measurementNoise.position = 6;        % GPS accuracy (meters)
params.measurementNoise.velocity = 1;         % Velocity measurement error (m/s)
params.measurementNoise.course = deg2rad(5.0);  % Heading error (radians)

% Process noise for realistic vessel dynamics
params.processNoise.position = 0.1;              % Position process noise
params.processNoise.velocity = 0.05;             % Velocity process noise
params.processNoise.acceleration = 0.02;         % Acceleration process noise
params.processNoise.turnRate = deg2rad(0.5);     % Turn rate process noise

% Realistic vessel dynamics for merchant vessels
params.vesselDynamics.maxSpeed = 15.0;           % 30 knots max speed
params.vesselDynamics.maxAcceleration = 0.5;     % Moderate acceleration
params.vesselDynamics.maxTurnRate = deg2rad(3.0); % 3 degrees/second max turn rate

% Mode transition probabilities for realistic maneuvering
params.modeTransition.probability = 0.02;        % 2% chance per second
params.modeTransition.minDuration = 60;          % Minimum 1 minute per mode
params.modeTransition.cvProbability = 0.6;       % 60% constant velocity
params.modeTransition.caProbability = 0.25;      % 25% constant acceleration
params.modeTransition.ctrvProbability = 0.15;    % 15% constant turn rate

% Communication settings
params.dropoutProbability = 0.05;                % 5% message loss rate

end