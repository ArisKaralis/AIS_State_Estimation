function [x_est, P_est] = ukfCV(data, q, pos_std, vel_std)
    % UNSCENTEDKALMANFILTERCV - Improved UKF implementation for CV model
    
    n = height(data);
    dt = calculateTimeSteps(data.timestamp);
    
    % Initialize filter with better velocity estimate
    initial_vx = 0;
    initial_vy = 0;
    if n > 1 && dt(2) > 0
        initial_vx = (data.x(2) - data.x(1)) / dt(2);
        initial_vy = (data.y(2) - data.y(1)) / dt(2);
    end
    
    % Create UKF with improved initial state
    ukf = trackingUKF(@cvStateTransitionFcn, @positionMeasurementFcn, ...
                      [data.x(1); initial_vx; data.y(1); initial_vy]);
    
    % Improved UKF parameters for better nonlinearity handling
    ukf.Alpha = 0.001;  % Increased from 1e-3 for better sigma point spread
    ukf.Beta = 2;       % Optimal for Gaussian distributions
    ukf.Kappa = 1;      % Changed from 0 to ensure positive definiteness
    
    % More conservative initial covariance
    ukf.StateCovariance = diag([pos_std^2/9, vel_std^2/4, pos_std^2/9, vel_std^2/4]);
    
    % Measurement noise
    R_pos = diag([pos_std^2, pos_std^2]);
    
    % Initialize output
    x_est = zeros(4, n);
    P_est = zeros(4, 4, n);
    
    x_est(:,1) = ukf.State;
    P_est(:,:,1) = ukf.StateCovariance;
    
    % Main filtering loop
    for k = 2:n
        dt_k = dt(k);
        if dt_k <= 0
            dt_k = 1;
        end
        
        % Improved process noise scaling
        Q = q * diag([dt_k^4/4, dt_k^3/2, dt_k^4/4, dt_k^3/2]);
        
        ukf.ProcessNoise = Q;
        
        predict(ukf, dt_k);
        
        if ~isnan(data.x(k)) && ~isnan(data.y(k))
            z = [data.x(k); data.y(k)];
            ukf.MeasurementNoise = R_pos;
            correct(ukf, z);
        end
        
        x_est(:,k) = ukf.State;
        P_est(:,:,k) = ukf.StateCovariance;
    end
end

function x = cvStateTransitionFcn(x, dt)
    F = [1 dt 0  0; 0 1  0  0; 0 0  1  dt; 0 0  0  1];
    x = F * x;
end

function z = positionMeasurementFcn(x)
    z = [x(1); x(3)];
end