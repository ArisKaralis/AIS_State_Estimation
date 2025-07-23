function observations = generateObservations(groundTruth, params)
% GENERATEOBSERVATIONS - Create realistic AIS sensor measurements from ground truth
%
% Converts perfect trajectory data into noisy AIS sensor measurements
% including dropouts and measurement errors.

% Calculate AIS observation timing
numSteps = size(groundTruth.position, 2);
dt_ais = params.aisReportInterval;
dt_sim = params.timeStep;

% Determine which simulation time steps correspond to AIS reports
aisIndices = 1:round(dt_ais/dt_sim):numSteps;
numObs = length(aisIndices);

% Initialize observation structure
observations = struct();
observations.time = zeros(numObs, 1);
observations.position = zeros(2, numObs);
observations.velocity = zeros(numObs, 1);
observations.course = zeros(numObs, 1);
observations.available = true(numObs, 1);
observations.groundTruthIndices = aisIndices;

% Generate realistic sensor measurements with noise
for i = 1:numObs
    idx = aisIndices(i);
    
    % SIMULATE AIS DROPOUT/MISSING REPORTS
    if rand() < params.dropoutProbability
        observations.available(i) = false;
        continue;
    end
    
    observations.time(i) = (idx - 1) * dt_sim;
    
    % Extract true values from ground truth
    truePos = groundTruth.position(:, idx);
    trueSpeed = groundTruth.speed(idx);
    trueHeading = groundTruth.heading(idx);
    
    % ADD REALISTIC MEASUREMENT NOISE
    % Position noise - GPS and antenna positioning errors
    posNoise = [randn(); randn()] * params.measurementNoise.position;
    
    % Velocity noise - Doppler measurement and GPS velocity errors
    velNoise = randn() * params.measurementNoise.velocity;
    
    % Course noise - Compass and GPS heading errors
    courseNoise = randn() * params.measurementNoise.course;
    
    % CREATE NOISY MEASUREMENTS
    observations.position(:, i) = truePos + posNoise;
    observations.velocity(i) = max(0, trueSpeed + velNoise);  % Ensure ≥0
    observations.course(i) = mod(trueHeading + courseNoise, 2*pi);  % Wrap to [0,2π]
end

% QUALITY CONTROL - Handle edge cases
for i = 1:numObs
    if observations.available(i) && isnan(observations.velocity(i))
        observations.available(i) = false;
    end
end

end