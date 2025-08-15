function groundTruth = generateGroundTruthTrajectory(numSteps, modeSequence, params)
% GENERATEGROUNDTRUTHTRAJECTORY - Creates perfect (noise-free) vessel trajectory
%
% Generates the "true" vessel motion using physical motion models.
% This represents the actual path the vessel follows, with perfect knowledge
% of position, velocity, acceleration, heading, speed, and turn rate.
%
% INPUTS:
%   numSteps     - Number of simulation time steps
%   modeSequence - Cell array of motion modes ('CV', 'CA', 'CT') for each step
%   params       - Simulation parameters including noise levels and dynamics
%
% OUTPUTS:
%   groundTruth  - Struct containing perfect trajectory data

% Initialize ground truth data structures
groundTruth = struct();
groundTruth.position = zeros(2, numSteps);      % [x; y] coordinates in meters
groundTruth.velocity = zeros(2, numSteps);      % [vx; vy] velocity components in m/s
groundTruth.acceleration = zeros(2, numSteps);  % [ax; ay] acceleration components in m/s²
groundTruth.heading = zeros(numSteps, 1);       % vessel heading in radians
groundTruth.speed = zeros(numSteps, 1);         % speed magnitude in m/s
groundTruth.turnRate = zeros(numSteps, 1);      % angular velocity in rad/s
groundTruth.mode = modeSequence;

% Set initial conditions
groundTruth.position(:, 1) = params.initialConditions.position;
groundTruth.velocity(:, 1) = params.initialConditions.velocity;
groundTruth.heading(1) = params.initialConditions.heading;
groundTruth.speed(1) = norm(params.initialConditions.velocity);

dt = params.timeStep;

% Generate trajectory by integrating motion equations
for k = 2:numSteps
    currentMode = modeSequence{k};
    
    % Extract previous state
    prevPos = groundTruth.position(:, k-1);
    prevVel = groundTruth.velocity(:, k-1);
    prevAccel = groundTruth.acceleration(:, k-1);
    prevHeading = groundTruth.heading(k-1);
    prevSpeed = groundTruth.speed(k-1);
    prevTurnRate = groundTruth.turnRate(k-1);
    
    % Apply motion model equations
    switch currentMode
        case 'CV'  % Constant Velocity - straight-line transit
            [newPos, newVel, newAccel, newHeading, newSpeed, newTurnRate] = ...
                updateConstantVelocity(prevPos, prevVel, dt, params);
            
        case 'CA'  % Constant Acceleration - speed changes
            [newPos, newVel, newAccel, newHeading, newSpeed, newTurnRate] = ...
                updateConstantAcceleration(prevPos, prevVel, prevAccel, dt, params);
            
        case 'CT'  % Constant Turn - course changes
            [newPos, newVel, newAccel, newHeading, newSpeed, newTurnRate] = ...
                updateConstantTurn(prevPos, prevVel, prevHeading, prevSpeed, prevTurnRate, dt, params);
    end
    
    % Store the new perfect state
    groundTruth.position(:, k) = newPos;
    groundTruth.velocity(:, k) = newVel;
    groundTruth.acceleration(:, k) = newAccel;
    groundTruth.heading(k) = newHeading;
    groundTruth.speed(k) = newSpeed;
    groundTruth.turnRate(k) = newTurnRate;
end

end