function [x_est, P_est, R, H] = kalmanCV_2D(data)
% KALMANCV_2D - Kalman Filter with [x; vx; y; vy] state and [x; y] measurements
% Uses CV motion model with position-only measurement

n = height(data);
dt0 = seconds(data.timestamp(2) - data.timestamp(1));

% === Tuning parameters ===
q_pos = 10;    % Position process noise variance
q_vel = 0.5;   % Velocity process noise variance

% === Measurement model: only x, y measured ===
H = [1 0 0 0;
     0 0 1 0];

% === Measurement noise ===
r_pos = 15;  % Position measurement std dev
R = r_pos^2 * eye(2);

% === Initial state and filter ===
[x0, P0] = initState4D(data);
kf = initKalmanFilter4D(x0, P0, H, R);

% === Allocate outputs ===
x_est = zeros(4, n);
P_est = zeros(4, 4, n);

% === Filtering loop ===
for k = 1:n
    if k > 1
        dt = seconds(data.timestamp(k) - data.timestamp(k-1));
    else
        dt = dt0;
    end

    Q = makeCVProcessNoise4D(dt, q_pos, q_vel);
    [kf, x_est(:,k), P_est(:,:,k)] = predictCorrectKF4D(kf, dt, Q, [data.x(k); data.y(k)]);
end
end

% === Local Functions ===

function [x0, P0] = initState4D(data)
    x0 = [double(data.x(1)); 0; double(data.y(1)); 0];  % Initial velocity unknown
    P0 = diag([100, 25, 100, 25]);
end

function kf = initKalmanFilter4D(x0, P0, H, R)
    kf = trackingKF( ...
        'MotionModel', '2D Constant Velocity', ...
        'State', x0, ...
        'StateCovariance', P0, ...
        'MeasurementModel', H, ...
        'MeasurementNoise', R, ...
        'ProcessNoise', eye(2) ...  % Placeholder
    );
end

function Q = makeCVProcessNoise4D(dt, q_pos, q_vel)
    Q = [q_pos*dt^4/4 q_pos*dt^3/2       0           0;
         q_pos*dt^3/2 q_vel*dt^2         0           0;
         0           0           q_pos*dt^4/4 q_pos*dt^3/2;
         0           0           q_pos*dt^3/2 q_vel*dt^2];
end

function [kf, x_new, P_new] = predictCorrectKF4D(kf, dt, Q, z)
    kf.ProcessNoise = Q([1 3],[1 3]);  % trackingKF expects 2x2 Q for velocity components
    predict(kf, dt);
    correct(kf, double(z));  % z = [x; y] measurement
    x_new = kf.State;
    P_new = kf.StateCovariance;
end
