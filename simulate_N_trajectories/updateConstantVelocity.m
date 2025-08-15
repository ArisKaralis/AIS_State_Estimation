function [newPos, newVel, newAccel, newHeading, newSpeed, newTurnRate] = ...
    updateConstantVelocity(prevPos, prevVel, dt, params)
% UPDATECONSTANTVELOCITY - Constant Velocity motion model with state-space approach
%
% This implements the CV model using a formal state-space representation,
% which is consistent with how Kalman filters work. The state vector is:
% state = [x; y; vx; vy] - position and velocity in 2D
%
% PHYSICS: Linear motion with small random disturbances
% - Position evolves as: x(k+1) = x(k) + vx(k)*dt + noise
% - Velocity evolves as: vx(k+1) = vx(k) + noise
%
% This creates realistic vessel motion during steady cruise conditions.

% PROCESS NOISE - Models random disturbances during motion
% In real vessels, even "constant velocity" has small variations due to:
% - Sea state (waves causing small course/speed changes)
% - Wind effects (pushing vessel slightly off course)  
% - Engine variations (slight speed fluctuations)
% - Autopilot corrections (small steering adjustments)
processNoise = [randn() * params.processNoise.position;    % x position noise
                randn() * params.processNoise.position;    % y position noise  
                randn() * params.processNoise.velocity;    % vx velocity noise
                randn() * params.processNoise.velocity];   % vy velocity noise

% STATE TRANSITION MATRIX (F) - Describes how state evolves over time
% This is the discrete-time constant velocity model:
% [x(k+1)]   [1 0 dt 0 ] [x(k) ]
% [y(k+1)] = [0 1 0  dt] [y(k) ] + process_noise
% [vx(k+1)]  [0 0 1  0 ] [vx(k)]
% [vy(k+1)]  [0 0 0  1 ] [vy(k)]
%
% Where dt is the time step. This matrix says:
% - New position = old position + velocity * time_step
% - New velocity = old velocity (constant velocity assumption)
F = [1 0 dt 0;    % x(k+1) = x(k) + vx(k)*dt
     0 1 0 dt;    % y(k+1) = y(k) + vy(k)*dt  
     0 0 1 0;     % vx(k+1) = vx(k) [constant velocity]
     0 0 0 1];    % vy(k+1) = vy(k) [constant velocity]

% APPLY MOTION MODEL
% Combine previous state into vector form
state = [prevPos; prevVel];  % [x; y; vx; vy] - current state vector

% Propagate state forward in time with process noise
newState = F * state + processNoise;  % Linear state evolution + random disturbances

% EXTRACT RESULTS from new state vector
newPos = newState(1:2);    % New position [x; y] 
newVel = newState(3:4);    % New velocity [vx; vy]
newAccel = [0; 0];         % Zero acceleration (constant velocity model)
newSpeed = norm(newVel);   % Speed magnitude |v| = sqrt(vx² + vy²)

% CALCULATE HEADING from velocity direction
% atan2(vy, vx) gives heading angle: 0°=East, 90°=North, etc.
if newSpeed > 0.01  % Avoid division by zero for very small velocities
    newHeading = atan2(newVel(2), newVel(1));  % Heading in radians
else
    newHeading = 0;  % Default heading if vessel is stationary
end

newTurnRate = 0;  % No turning in constant velocity model

% SUMMARY: This function takes the previous position and velocity,
% applies constant velocity physics with realistic process noise,
% and returns the new state. The process noise ensures that even
% "constant velocity" motion has small, realistic variations that
% filtering algorithms must handle.

end