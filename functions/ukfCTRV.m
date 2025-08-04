function [x_est, P_est] = ukfCTRV(data, q, pos_std, vel_std)
    % UNSCENTEDKALMANFILTERCTRV - Improved UKF implementation for CTRV model
    
    n = height(data);
    dt = calculateTimeSteps(data.timestamp);
    
    % Better initialization using multiple points
    initial_vx = 0;
    initial_vy = 0;
    initial_speed = 0;
    initial_yaw = 0;
    initial_yaw_rate = 0;
    
    if n > 1 && dt(2) > 0
        initial_vx = (data.x(2) - data.x(1)) / dt(2);
        initial_vy = (data.y(2) - data.y(1)) / dt(2);
        initial_speed = sqrt(initial_vx^2 + initial_vy^2);
        initial_yaw = atan2(initial_vy, initial_vx);
        
        % Use more points for better yaw rate estimation
        if n > 3 && dt(3) > 0
            % Use multiple points for more stable yaw rate estimate
            yaw_rates = [];
            for i = 3:min(5, n)
                if dt(i) > 0
                    vx_i = (data.x(i) - data.x(i-1)) / dt(i);
                    vy_i = (data.y(i) - data.y(i-1)) / dt(i);
                    yaw_i = atan2(vy_i, vx_i);
                    
                    vx_prev = (data.x(i-1) - data.x(i-2)) / dt(i-1);
                    vy_prev = (data.y(i-1) - data.y(i-2)) / dt(i-1);
                    yaw_prev = atan2(vy_prev, vx_prev);
                    
                    dyaw = angdiff(yaw_i, yaw_prev);
                    yaw_rate = dyaw / dt(i);
                    yaw_rates(end+1) = max(-0.05, min(0.05, yaw_rate));
                end
            end
            if ~isempty(yaw_rates)
                initial_yaw_rate = median(yaw_rates);  % Use median for robustness
            end
        end
    end
    
    % Create UKF with improved parameters
    ukf = trackingUKF(@ctrvStateTransitionFcn, @positionMeasurementFcn, ...
                      [data.x(1); data.y(1); initial_speed; initial_yaw; initial_yaw_rate]);
    
    % Optimized UKF parameters for CTRV model
    ukf.Alpha = 0.001;  % Increased for better sigma point spread
    ukf.Beta = 2;       % Optimal for Gaussian distributions  
    ukf.Kappa = 2;      % n-3=2 for 5D state space (recommended for CTRV)
    
    % More appropriate initial covariance for CTRV
    ukf.StateCovariance = diag([pos_std^2/9, pos_std^2/9, vel_std^2/4, deg2rad(10)^2, deg2rad(1)^2]);
    
    R_pos = diag([pos_std^2, pos_std^2]);
    
    % Initialize output
    x_est = zeros(5, n);
    P_est = zeros(5, 5, n);
    
    x_est(:,1) = ukf.State;
    P_est(:,:,1) = ukf.StateCovariance;
    
    % Main filtering loop
    for k = 2:n
        dt_k = dt(k);
        if dt_k <= 0
            dt_k = 1;
        end
        
        % Improved process noise model for CTRV
        Q = q * diag([dt_k^4/4, dt_k^4/4, dt_k^2, deg2rad(0.5)^2*dt_k, deg2rad(0.1)^2*dt_k]);
        
        ukf.ProcessNoise = Q;
        
        predict(ukf, dt_k);
        
        % Normalize yaw angle after prediction
        ukf.State(4) = mod(ukf.State(4) + pi, 2*pi) - pi;
        
        if ~isnan(data.x(k)) && ~isnan(data.y(k))
            z = [data.x(k); data.y(k)];
            ukf.MeasurementNoise = R_pos;
            correct(ukf, z);
            
            % Normalize yaw angle after correction
            ukf.State(4) = mod(ukf.State(4) + pi, 2*pi) - pi;
        end
        
        x_est(:,k) = ukf.State;
        P_est(:,:,k) = ukf.StateCovariance;
    end
end

function x = ctrvStateTransitionFcn(x, dt)
    % Extract state variables
    px = x(1);
    py = x(2);
    v = x(3);
    yaw = x(4);
    yaw_rate = x(5);
    
    % Handle near-zero yaw rate case
    if abs(yaw_rate) < 1e-6
        % Linear motion model
        x_new = [px + v * cos(yaw) * dt;
                 py + v * sin(yaw) * dt;
                 v;
                 yaw;
                 yaw_rate];
    else
        % CTRV motion model
        x_new = [px + (v / yaw_rate) * (sin(yaw + yaw_rate * dt) - sin(yaw));
                 py + (v / yaw_rate) * (-cos(yaw + yaw_rate * dt) + cos(yaw));
                 v;
                 yaw + yaw_rate * dt;
                 yaw_rate];
    end
    
    x = x_new;
end

function z = positionMeasurementFcn(x)
    z = [x(1); x(2)];
end