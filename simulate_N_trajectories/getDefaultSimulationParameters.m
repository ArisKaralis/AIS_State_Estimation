function params = getDefaultSimulationParameters()
% GETDEFAULTSIMULATIONPARAMETERS - Default configuration for AIS trajectory simulation
%
% Provides realistic default values for all simulation parameters based on
% real-world vessel operations and AIS system characteristics. These values
% create believable vessel trajectories and sensor measurements.
%
% OUTPUTS:
%   params - Struct containing all simulation parameters with realistic defaults

params = struct();

% TEMPORAL PARAMETERS - Time resolution and simulation duration
params.timeStep = 1.0;              % Simulation time step (seconds) - 1Hz typical
params.aisReportInterval = 5.0;     % AIS reporting interval (seconds) - IMO standard
params.totalDuration = 1800;        % Total simulation time (seconds) - 30 minutes default

% PROCESS NOISE - Random disturbances during vessel motion
% These model real-world variations in vessel motion due to environmental factors
params.processNoise = struct();
params.processNoise.position = 0.1;      % Position noise std (meters) - GPS/navigation uncertainty
params.processNoise.velocity = 0.05;     % Velocity noise std (m/s) - engine/propulsion variations
params.processNoise.acceleration = 0.02; % Acceleration noise std (m/s²) - control system variations  
params.processNoise.turnRate = deg2rad(0.5); % Turn rate noise std (rad/s) - steering variations (reduced)

% MEASUREMENT NOISE - AIS sensor measurement errors
% Based on IMO performance standards and real AIS system accuracy
params.measurementNoise = struct();
params.measurementNoise.position = 20.0;        % Position error std (meters) - GPS accuracy
params.measurementNoise.velocity = 0.5;         % Speed error std (m/s) - Doppler/GPS velocity error
params.measurementNoise.course = deg2rad(5.0);  % Heading error std (radians) - compass/GPS heading error

% VESSEL DYNAMICS - Physical limits and capabilities
% Typical values for merchant vessels, can be adjusted for different vessel types
params.vesselDynamics = struct();
params.vesselDynamics.maxSpeed = 15.0;           % Maximum speed (m/s) ≈ 30 knots
params.vesselDynamics.maxAcceleration = 0.5;     % Maximum acceleration (m/s²)
params.vesselDynamics.maxTurnRate = deg2rad(1.0); % Maximum turn rate (rad/s) ≈ 1°/s (REDUCED for realistic turns)

% MODE TRANSITION - How vessel switches between motion patterns
params.modeTransition = struct();
params.modeTransition.probability = 0.05;  % Probability of mode change per time step
params.modeTransition.minDuration = 30;    % Minimum time in each mode (seconds) (REDUCED)
params.modeTransition.maxTurnDuration = 45; % Maximum time in CT mode (seconds) (NEW - prevents loops)

% COMMUNICATION - AIS system reliability
params.dropoutProbability = 0.05;  % Probability of missing AIS report (5% typical)

% INITIAL CONDITIONS - Starting state for vessel
params.initialConditions = struct();
params.initialConditions.position = [0; 0];  % Starting position [x,y] (meters)
params.initialConditions.velocity = [5; 0];  % Starting velocity [vx,vy] (m/s) - 10 knots eastward
params.initialConditions.heading = 0;        % Starting heading (radians) - due east

% REPRODUCIBILITY - For consistent Monte Carlo results
params.randomSeed = 42;  % Default seed for reproducible simulations

end