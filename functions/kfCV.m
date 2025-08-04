function [x_est, P_est] = kfCV(data, q, pos_std, vel_std)
    % KALMANFILTERCV - Core Kalman filter implementation for CV model
    % Returns state estimates and covariances for reuse in other filters
    
    n = height(data);
    dt = calculateTimeSteps(data.timestamp);
    
    % Initialize filter
    initial_vx = 0;
    initial_vy = 0;
    if n > 1 && dt(2) > 0
        initial_vx = (data.x(2) - data.x(1)) / dt(2);
        initial_vy = (data.y(2) - data.y(1)) / dt(2);
    end
    
    kf = trackingKF('MotionModel', 'Custom', ...
                   'StateTransitionModel', eye(4), ...
                   'MeasurementModel', [1 0 0 0; 0 0 1 0], ...
                   'State', [data.x(1); initial_vx; data.y(1); initial_vy], ...
                   'StateCovariance', diag([pos_std^2/4, vel_std^2, pos_std^2/4, vel_std^2]), ...
                   'MeasurementNoise', diag([pos_std^2, pos_std^2]));
    
    % Initialize output
    x_est = zeros(4, n);
    P_est = zeros(4, 4, n);
    
    % First measurement update
    if ~isnan(data.x(1)) && ~isnan(data.y(1))
        z = [data.x(1); data.y(1)];
        [x_est(:,1), P_est(:,:,1)] = correct(kf, z);
    else
        x_est(:,1) = kf.State;
        P_est(:,:,1) = kf.StateCovariance;
    end
    
    % Main filtering loop
    for k = 2:n
        dt_k = dt(k);
        if dt_k <= 0 || dt_k > 60
            dt_k = 1.0;
        end
        
        % State transition matrix
        F = [1 dt_k 0 0;
             0 1    0 0;
             0 0    1 dt_k;
             0 0    0 1];
        
        % Process noise matrix
        Q = q * [dt_k^3/3, dt_k^2/2, 0, 0;
                 dt_k^2/2, dt_k,     0, 0;
                 0,        0,        dt_k^3/3, dt_k^2/2;
                 0,        0,        dt_k^2/2, dt_k];
        
        kf.StateTransitionModel = F;
        kf.ProcessNoise = Q;
        
        predict(kf);
        
        % Measurement update
        if ~isnan(data.x(k)) && ~isnan(data.y(k))
            z = [data.x(k); data.y(k)];
            [x_est(:,k), P_est(:,:,k)] = correct(kf, z);
        else
            x_est(:,k) = kf.State;
            P_est(:,:,k) = kf.StateCovariance;
        end
    end
end