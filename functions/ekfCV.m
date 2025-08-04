function [x_est, P_est] = ekfCV(data, q, pos_std, vel_std)
    % EXTENDEDKALMANFILTERCV - Core EKF implementation for CV model
    
    n = height(data);
    dt = calculateTimeSteps(data.timestamp);
    
    % Initialize filter
    initial_vx = 0;
    initial_vy = 0;
    if n > 1 && dt(2) > 0
        initial_vx = (data.x(2) - data.x(1)) / dt(2);
        initial_vy = (data.y(2) - data.y(1)) / dt(2);
    end
    
    % Use trackingEKF instead of extendedKalmanFilter
    ekf = trackingEKF(@cvStateTransitionFcn, @measurementFcn, ...
                     [data.x(1); initial_vx; data.y(1); initial_vy], ...
                     'StateCovariance', diag([pos_std^2/4, vel_std^2, pos_std^2/4, vel_std^2]), ...
                     'MeasurementNoise', diag([pos_std^2, pos_std^2]));
    
    % Set Jacobian functions for trackingEKF
    ekf.StateTransitionJacobianFcn = @stateTransitionJacobianFcn;
    ekf.MeasurementJacobianFcn = @measurementJacobianFcn;
    
    % Initialize output
    x_est = zeros(4, n);
    P_est = zeros(4, 4, n);
    
    x_est(:,1) = ekf.State;
    P_est(:,:,1) = ekf.StateCovariance;
    
    % Main filtering loop
    for k = 2:n
        dt_k = dt(k);
        if dt_k <= 0
            dt_k = 1;
        end
        
        Q = q * [dt_k^3/3, dt_k^2/2, 0, 0;
                 dt_k^2/2, dt_k,     0, 0;
                 0,        0,        dt_k^3/3, dt_k^2/2;
                 0,        0,        dt_k^2/2, dt_k];
        
        ekf.ProcessNoise = Q;
        
        predict(ekf, dt_k);
        
        if ~isnan(data.x(k)) && ~isnan(data.y(k))
            z = [data.x(k); data.y(k)];
            correct(ekf, z);
        end
        
        x_est(:,k) = ekf.State;
        P_est(:,:,k) = ekf.StateCovariance;
    end
end

% Helper functions
function x = cvStateTransitionFcn(x, dt)
    F = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1];
    x = F * x;
end

function F = stateTransitionJacobianFcn(x, dt)
    F = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1];
end

function z = measurementFcn(x)
    z = [x(1); x(3)];
end

function H = measurementJacobianFcn(x)
    H = [1 0 0 0; 0 0 1 0];
end