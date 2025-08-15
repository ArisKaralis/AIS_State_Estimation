function randomParams = randomizeSimulationParameters(baseParams)
% RANDOMIZESIMULATIONPARAMETERS - Creates randomided parameter variations for Monte Carlo
%
% Takes base parameters and creates realistic variations to simulate different
% vessels, environmental conditions, and operational scenarios. This ensures
% Monte Carlo simulations test filter performance across diverse conditions.
%
% PARAMETER VARIATIONS:
% - Vessel characteristics (size affects dynamics)
% - Environmental conditions (sea state affects noise)
% - Sensor quality (different AIS equipment accuracy)
% - Operational patterns (different mission profiles)
%
% INPUTS:
%   baseParams - Default parameter structure from getDefaultSimulationParameters()
%
% OUTPUTS:
%   randomParams - Randomized parameter structure for one Monte Carlo run

% Start with base parameters
randomParams = baseParams;

% VESSEL DYNAMICS VARIATIONS - Different vessel types and sizes
% Larger vessels: slower acceleration, wider turns, higher max speed
% Smaller vessels: quicker acceleration, tighter turns, lower max speed
vesselSizeFactor = 0.5 + rand() * 1.0;  % Random factor [0.5, 1.5]

randomParams.vesselDynamics.maxSpeed = baseParams.vesselDynamics.maxSpeed * ...
    (0.7 + 0.6 * rand());  % ±30% speed variation

randomParams.vesselDynamics.maxAcceleration = baseParams.vesselDynamics.maxAcceleration * ...
    (1.0 / vesselSizeFactor) * (0.5 + 1.0 * rand());  % Smaller vessels accelerate faster

randomParams.vesselDynamics.maxTurnRate = baseParams.vesselDynamics.maxTurnRate * ...
    (1.0 / vesselSizeFactor) * (0.6 + 0.8 * rand());  % Smaller vessels turn faster

% ENVIRONMENTAL CONDITIONS - Sea state and weather effects
% Calm conditions: lower process noise
% Rough conditions: higher process noise
environmentFactor = 0.3 + 1.4 * rand();  % Random factor [0.3, 1.7]

randomParams.processNoise.position = baseParams.processNoise.position * environmentFactor;
randomParams.processNoise.velocity = baseParams.processNoise.velocity * environmentFactor;
randomParams.processNoise.acceleration = baseParams.processNoise.acceleration * environmentFactor;
randomParams.processNoise.turnRate = baseParams.processNoise.turnRate * environmentFactor;

% SENSOR QUALITY VARIATIONS - Different AIS equipment accuracy
% High-end systems: lower measurement noise
% Basic systems: higher measurement noise
sensorQualityFactor = 0.5 + 1.0 * rand();  % Random factor [0.5, 1.5]

randomParams.measurementNoise.position = baseParams.measurementNoise.position * sensorQualityFactor;
randomParams.measurementNoise.velocity = baseParams.measurementNoise.velocity * sensorQualityFactor;
randomParams.measurementNoise.course = baseParams.measurementNoise.course * sensorQualityFactor;

% OPERATIONAL PATTERN VARIATIONS - Different mission profiles
% Some vessels: frequent course/speed changes
% Others: steady long-distance transit
operationalFactor = 0.5 + 1.0 * rand();  % Random factor [0.5, 1.5]

randomParams.modeTransition.probability = baseParams.modeTransition.probability * operationalFactor;
randomParams.modeTransition.minDuration = baseParams.modeTransition.minDuration * ...
    (0.5 + 1.0 * rand());  % Vary mode persistence

% COMMUNICATION CONDITIONS - Different ranges and interference
randomParams.dropoutProbability = baseParams.dropoutProbability * (0.5 + 1.0 * rand());

% INITIAL CONDITION VARIATIONS - Different starting scenarios
% Randomize starting speed and direction
startSpeed = 3 + 7 * rand();  % Random speed 3-10 m/s (6-20 knots)
startHeading = 2 * pi * rand();  % Random heading 0-360°

randomParams.initialConditions.velocity = startSpeed * [cos(startHeading); sin(startHeading)];
randomParams.initialConditions.heading = startHeading;

% Random starting position within reasonable bounds
randomParams.initialConditions.position = [(-500 + 1000 * rand()); (-500 + 1000 * rand())];

end