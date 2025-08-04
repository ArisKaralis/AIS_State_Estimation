function [x_est, P_est] = ekfCA(data, q, pos_std, vel_std, acc_std)
    % EXTENDEDKALMANFILTERCA - Core EKF implementation for CA model
    
    n = height(data);
    dt = calculateTimeSteps(data.timestamp);
    
    % Initialize filter
    initial_vx = 0;
    initial_vy = 0;
    initial_ax = 0;
    initial_ay = 0;
    
    if n > 1 && dt(2) > 0
        initial_vx = (data.x(2) - data.x(1)) / dt(2);
        initial_vy = (data.y(2) - data.y(1)) / dt(2);
    end
    
    if n > 2 && dt(3) > 0
        vx2 = (data.x(3) - data.x(2)) / dt(3);
        vy2 = (data.y(3) - data.y(2)) / dt(3);
        initial_ax = (vx2 - initial_vx) / dt(3);
        initial_ay = (vy2 - initial_vy) / dt(3);
        initial_ax = max(-0.5, min(0.5, initial_ax));
        initial_ay = max(-0.5, min(0.5, initial_ay));
    end
    
    % Create trackingEKF instead of trackingKF
    ekf = trackingEKF(@caStateTransitionFcn, @caMeasurementFcn, ...
                     [data.x(1); initial_vx; initial_ax; data.y(1); initial_vy; initial_ay], ...
                     'StateCovariance', diag([pos_std^2/4, vel_std^2, acc_std^2, pos_std^2/4, vel_std^2, acc_std^2]), ...
                     'MeasurementNoise', diag([pos_std^2, pos_std^2]));
    
    % Set the Jacobian functions for trackingEKF
    ekf.StateTransitionJacobianFcn = @caStateTransitionJacobianFcn;
    ekf.MeasurementJacobianFcn = @caMeasurementJacobianFcn;
    
    % Initialize output
    x_est = zeros(6, n);
    P_est = zeros(6, 6, n);
    
    x_est(:,1) = ekf.State;
    P_est(:,:,1) = ekf.StateCovariance;
    
    % Main filtering loop
    for k = 2:n
        dt_k = dt(k);
        if dt_k <= 0 || dt_k > 60
            dt_k = 1.0;
        end
        
        % Process noise matrix
        Q11 = q * [dt_k^5/20, dt_k^4/8,  dt_k^3/6;
                   dt_k^4/8,  dt_k^3/3,  dt_k^2/2;
                   dt_k^3/6,  dt_k^2/2,  dt_k];
        
        Q = blkdiag(Q11, Q11);
        
        ekf.ProcessNoise = Q;
        
        predict(ekf, dt_k);
        
        % Apply acceleration limits
        max_accel = 0.5;
        ekf.State(3) = max(-max_accel, min(max_accel, ekf.State(3)));
        ekf.State(6) = max(-max_accel, min(max_accel, ekf.State(6)));
        
        if ~isnan(data.x(k)) && ~isnan(data.y(k))
            z = [data.x(k); data.y(k)];
            [x_est(:,k), P_est(:,:,k)] = correct(ekf, z);
            
            x_est(3,k) = max(-max_accel, min(max_accel, x_est(3,k)));
            x_est(6,k) = max(-max_accel, min(max_accel, x_est(6,k)));
            ekf.State = x_est(:,k);
        else
            x_est(:,k) = ekf.State;
            P_est(:,:,k) = ekf.StateCovariance;
        end
    end
end

% State transition function for CA model
function x_pred = caStateTransitionFcn(x, dt)
    % Constant Acceleration model state transition
    % State: [x; vx; ax; y; vy; ay]
    F = [1, dt, dt^2/2, 0, 0,  0;
         0, 1,  dt,     0, 0,  0;
         0, 0,  1,      0, 0,  0;
         0, 0,  0,      1, dt, dt^2/2;
         0, 0,  0,      0, 1,  dt;
         0, 0,  0,      0, 0,  1];
    x_pred = F * x;
end

% State transition Jacobian function for CA model
function F = caStateTransitionJacobianFcn(x, dt)
    % Jacobian of state transition function (linear, so same as F matrix)
    F = [1, dt, dt^2/2, 0, 0,  0;
         0, 1,  dt,     0, 0,  0;
         0, 0,  1,      0, 0,  0;
         0, 0,  0,      1, dt, dt^2/2;
         0, 0,  0,      0, 1,  dt;
         0, 0,  0,      0, 0,  1];
end

% Measurement function for CA model
function z = caMeasurementFcn(x)
    % CA measurement model: observe position only
    % State: [x; vx; ax; y; vy; ay] -> Measurement: [x; y]
    z = [x(1); x(4)];
end

% Measurement Jacobian function for CA model
function H = caMeasurementJacobianFcn(x)
    % Jacobian of measurement function
    H = [1, 0, 0, 0, 0, 0;
         0, 0, 0, 1, 0, 0];
end