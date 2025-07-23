function [newPos, newVel, newAccel, newHeading, newSpeed, newTurnRate] = ...
    updateConstantAcceleration(prevPos, prevVel, prevAccel, dt, params)
% UPDATECONSTANTACCELERATION - Constant Acceleration motion model
%
% This implements the CA model for vessels undergoing speed changes.
% The acceleration is maintained with small random variations due to
% process noise (engine control variations, sea conditions, etc.).
%
% PHYSICS: Constant acceleration with kinematic equations
% - Position: p(t+dt) = p(t) + v(t)*dt + 0.5*a*dt²
% - Velocity: v(t+dt) = v(t) + a*dt  
% - Acceleration: a(t+dt) = a(t) + noise
%
% REALISTIC SCENARIOS:
% - Vessel accelerating/decelerating (leaving/approaching port)
% - Speed changes due to traffic, weather, or operational requirements
% - Engine power adjustments during transit

% ACCELERATION PROCESS NOISE - Models variations in acceleration
% Real vessels don't maintain perfectly constant acceleration due to:
% - Engine control system variations (throttle adjustments)
% - Sea state effects (waves affecting propulsion efficiency)
% - Fuel flow variations, engine wear, loading changes
accelNoise = [randn() * params.processNoise.acceleration;    % ax noise  
              randn() * params.processNoise.acceleration];   % ay noise

% APPLY ACCELERATION LIMITS - Vessels have physical constraints
% Real ships cannot accelerate arbitrarily - limited by engine power,
% propeller efficiency, hydrodynamic drag, and safety considerations
maxAccel = params.vesselDynamics.maxAcceleration;  % Typical: 0.2-0.8 m/s² for ships

% Update acceleration with noise and physical limits
newAccel = prevAccel + accelNoise;  % Add random variations to acceleration

% Limit acceleration magnitude to realistic values
accelMagnitude = norm(newAccel);
if accelMagnitude > maxAccel
    newAccel = newAccel * (maxAccel / accelMagnitude);  % Scale to maximum allowed
end

% KINEMATIC EQUATIONS - Apply constant acceleration physics
% These are the standard equations of motion for constant acceleration:
% Position: s = s₀ + v₀t + ½at²  
% Velocity: v = v₀ + at
newPos = prevPos + prevVel * dt + 0.5 * newAccel * dt^2;  % Position update with acceleration
newVel = prevVel + newAccel * dt;                         % Velocity update
newSpeed = norm(newVel);                                  % Speed magnitude

% SPEED LIMITING - Vessels have maximum operational speeds
% Physical constraints: hull design, engine power, fuel efficiency
if newSpeed > params.vesselDynamics.maxSpeed
    % Scale velocity vector to maintain direction but limit magnitude
    newVel = newVel * (params.vesselDynamics.maxSpeed / newSpeed);
    newSpeed = params.vesselDynamics.maxSpeed;
end

% HEADING CALCULATION from velocity direction
if newSpeed > 0.01  % Avoid numerical issues when nearly stationary
    newHeading = atan2(newVel(2), newVel(1));  % Heading from velocity vector
else
    newHeading = 0;  % Default heading if vessel is stopped
end

newTurnRate = 0;  % No coordinated turning in CA model (pure linear acceleration)

% ADD POSITION PROCESS NOISE - Small random positioning errors
% Even during acceleration, there are small random position variations due to:
% - Sea state (wave-induced motion)
% - Steering corrections to maintain course
% - GPS/navigation system variations
posNoise = [randn() * params.processNoise.position;    % x position noise
            randn() * params.processNoise.position];   % y position noise
newPos = newPos + posNoise;  % Apply position noise to final result

% SUMMARY: This function models realistic vessel acceleration/deceleration
% with proper kinematic equations, physical limits, and process noise.
% It's used for scenarios like port approaches, speed adjustments, and
% any situation where the vessel is changing speed in a controlled manner.

end