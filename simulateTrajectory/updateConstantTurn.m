function [newPos, newVel, newAccel, newHeading, newSpeed, newTurnRate] = ...
    updateConstantTurn(prevPos, prevVel, prevHeading, prevSpeed, prevTurnRate, dt, params)
% UPDATECONSTANTTURN - Realistic Course Change motion model (modified from CTRV)
%
% This implements a realistic CT model for vessels performing LIMITED course
% changes rather than continuous circular motion. Real vessels make course
% corrections to reach new headings, not endless loops.
%
% PHYSICS: Controlled course change with target heading approach
% - Turn rate decreases as vessel approaches target heading
% - Limited total course change (typically 15-90 degrees)
% - Realistic maritime maneuvering patterns
%
% REALISTIC SCENARIOS:
% - Course corrections for navigation waypoints
% - Collision avoidance maneuvers  
% - Traffic separation adjustments
% - Minor heading corrections during transit

% Initialize target heading if not already set (first time in CT mode)
persistent targetHeading totalTurnSoFar;
if isempty(targetHeading)
    % Generate realistic course change (15-90 degrees)
    courseChange = deg2rad(15 + 75 * rand()); % Random course change 15-90°
    turnDirection = 2 * (rand() > 0.5) - 1;   % Random turn direction: +1 or -1
    targetHeading = prevHeading + turnDirection * courseChange;
    targetHeading = mod(targetHeading, 2*pi);  % Keep in [0, 2π]
    totalTurnSoFar = 0;
end

% Calculate heading error (how far from target)
headingError = angdiff(targetHeading, prevHeading);

% ADAPTIVE TURN RATE - Slow down as we approach target heading
% This prevents overshoot and creates realistic S-curve approach
errorMagnitude = abs(headingError);
if errorMagnitude > deg2rad(5)  % Far from target - turn at max rate
    baseTurnRate = sign(headingError) * params.vesselDynamics.maxTurnRate;
else  % Close to target - reduce turn rate proportionally
    baseTurnRate = sign(headingError) * params.vesselDynamics.maxTurnRate * ...
                   (errorMagnitude / deg2rad(5));
end

% TURN RATE PROCESS NOISE - Models variations in turning
turnRateNoise = randn() * params.processNoise.turnRate;
newTurnRate = baseTurnRate + turnRateNoise;

% Limit turn rate to physical constraints
maxTurnRate = params.vesselDynamics.maxTurnRate;
newTurnRate = max(-maxTurnRate, min(newTurnRate, maxTurnRate));

% SPEED PROCESS NOISE - Speed can vary slightly during turns
speedNoise = randn() * params.processNoise.velocity;
newSpeed = prevSpeed + speedNoise;
newSpeed = max(0, min(newSpeed, params.vesselDynamics.maxSpeed));

% HEADING UPDATE
newHeading = prevHeading + newTurnRate * dt;
newHeading = mod(newHeading, 2*pi);

% Track total turn progress
totalTurnSoFar = totalTurnSoFar + abs(newTurnRate * dt);

% CHECK FOR TURN COMPLETION OR TIMEOUT
% Exit CT mode if we've reached target or turned too much
if abs(angdiff(targetHeading, newHeading)) < deg2rad(2) || ...
   totalTurnSoFar > deg2rad(120)  % Max 120° total turn to prevent loops
    % Reset for next CT mode entry
    targetHeading = [];
    totalTurnSoFar = [];
    % Signal mode transition by setting very small turn rate
    newTurnRate = 0.001 * sign(newTurnRate);
end

% POSITION UPDATE - Use exact circular motion equations for small turns
if abs(newTurnRate) > 0.001
    % Calculate turn radius and position change
    turnRadius = newSpeed / abs(newTurnRate);
    angleChange = newTurnRate * dt;
    
    % Exact circular motion integration
    deltaX = (newSpeed / newTurnRate) * (sin(prevHeading + angleChange) - sin(prevHeading));
    deltaY = (newSpeed / newTurnRate) * (-cos(prevHeading + angleChange) + cos(prevHeading));
    
    newPos = prevPos + [deltaX; deltaY];
else
    % Nearly straight line motion
    deltaX = newSpeed * cos(newHeading) * dt;
    deltaY = newSpeed * sin(newHeading) * dt;
    newPos = prevPos + [deltaX; deltaY];
end

% UPDATE VELOCITY VECTOR
newVel = newSpeed * [cos(newHeading); sin(newHeading)];

% CENTRIPETAL ACCELERATION
if abs(newTurnRate) > 0.001
    centripetalAccel = newSpeed * abs(newTurnRate);
    accelDirection = newHeading + sign(newTurnRate) * pi/2;
    newAccel = centripetalAccel * [cos(accelDirection); sin(accelDirection)];
else
    newAccel = [0; 0];
end

% ADD POSITION PROCESS NOISE
posNoise = [randn() * params.processNoise.position;
            randn() * params.processNoise.position];
newPos = newPos + posNoise;

end