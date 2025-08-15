function [x_est, P_est, innovations, S_innovations] = ekfCV(data, q, pos_std, vel_std)
% EXTENDEDKALMANFILTERCV - Core EKF implementation for Constant Velocity (CV) model
% State: [x, vx, y, vy]
%
% Inputs:
%   data     - table with fields: timestamp, x, y (NaN allowed), optionally others
%   q        - process noise scale
%   pos_std  - position measurement std (meters)
%   vel_std  - velocity std for initial covariance (m/s)
%
% Outputs:
%   x_est            - 4 x N state history
%   P_est            - 4 x 4 x N covariance history
%   innovations      - 2 x N innovation vectors (NaN at k=1 or when no meas)
%   S_innovations    - 2 x 2 x N innovation covariances (NaN at k=1 or when no meas)

    n = height(data);
    dt = calculateTimeSteps(data.timestamp);

    % Initial velocity from first two points if available
    initial_vx = 0;
    initial_vy = 0;
    if n > 1 && dt(2) > 0 && ~any(isnan([data.x(1), data.x(2), data.y(1), data.y(2)]))
        initial_vx = (data.x(2) - data.x(1)) / dt(2);
        initial_vy = (data.y(2) - data.y(1)) / dt(2);
    end

    % Create EKF
    ekf = trackingEKF(@cvStateTransitionFcn, @cvMeasurementFcn, ...
                      [data.x(1); initial_vx; data.y(1); initial_vy], ...
                      'StateTransitionJacobianFcn', @cvStateTransitionJacobianFcn, ...
                      'MeasurementJacobianFcn', @cvMeasurementJacobianFcn);

    % Initial covariance and measurement noise
    ekf.StateCovariance = diag([pos_std^2/4, vel_std^2, pos_std^2/4, vel_std^2]);
    R_pos = diag([pos_std^2, pos_std^2]);

    % Outputs
    x_est = zeros(4, n);
    P_est = zeros(4, 4, n);
    if nargout >= 3
        innovations = zeros(2, n);
        S_innovations = zeros(2, 2, n);
    end

    % k = 1 (no correction; align with other filters)
    x_est(:,1) = ekf.State;
    P_est(:,:,1) = ekf.StateCovariance;
    if nargout >= 3
        innovations(:,1) = NaN;
        S_innovations(:,:,1) = NaN;
    end

    % Main loop
    for k = 2:n
        dt_k = dt(k);
        if dt_k <= 0 || dt_k > 60
            dt_k = 1.0;
        end

        % Process noise for CV (per-axis)
        Q = q * diag([dt_k^4/4, dt_k^3/2, dt_k^4/4, dt_k^3/2]);
        ekf.ProcessNoise = Q;

        % Predict
        predict(ekf, dt_k);

        % Measurement update if available
        if ~isnan(data.x(k)) && ~isnan(data.y(k))
            z = [data.x(k); data.y(k)];
            ekf.MeasurementNoise = R_pos;

            % Innovation before correction (for NIS)
            if nargout >= 3
                H = cvMeasurementJacobianFcn(ekf.State);
                z_pred = cvMeasurementFcn(ekf.State);
                innovation = z - z_pred;
                S = H * ekf.StateCovariance * H' + R_pos;

                innovations(:,k) = innovation;
                S_innovations(:,:,k) = S;
            end

            [x_est(:,k), P_est(:,:,k)] = correct(ekf, z);
        else
            % No measurement this step
            x_est(:,k) = ekf.State;
            P_est(:,:,k) = ekf.StateCovariance;

            if nargout >= 3
                innovations(:,k) = NaN;
                S_innovations(:,:,k) = NaN;
            end
        end
    end
end

% ----- CV model functions -----

function x_next = cvStateTransitionFcn(x, dt)
    F = [1, dt, 0,  0;
         0,  1, 0,  0;
         0,  0, 1, dt;
         0,  0, 0,  1];
    x_next = F * x;
end

function F = cvStateTransitionJacobianFcn(~, dt)
    F = [1, dt, 0,  0;
         0,  1, 0,  0;
         0,  0, 1, dt;
         0,  0, 0,  1];
end

function z = cvMeasurementFcn(x)
    % Position-only measurement
    H = [1, 0, 0, 0;
         0, 0, 1, 0];
    z = H * x;
end

function H = cvMeasurementJacobianFcn(~)
    H = [1, 0, 0, 0;
         0, 0, 1, 0];
end