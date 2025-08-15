function [x_est, P_est, innovations, S_innovations] = ekfCTRV(data, q, pos_std, vel_std)
    % EXTENDEDKALMANFILTERCTRV - Core EKF implementation for CTRV model
    % State: [x, y, v, yaw, yaw_rate]
    
    n = height(data);
    dt = calculateTimeSteps(data.timestamp);
    
    % Initialize filter parameters
    initial_vx = 0;
    initial_vy = 0;
    initial_speed = 0;
    initial_yaw = 0;
    initial_yaw_rate = 0;
    
    % Better initial conditions from data
    if n > 1 && dt(2) > 0
        initial_vx = (data.x(2) - data.x(1)) / dt(2);
        initial_vy = (data.y(2) - data.y(1)) / dt(2);
        initial_speed = sqrt(initial_vx^2 + initial_vy^2);
        initial_yaw = atan2(initial_vy, initial_vx);
        
        if n > 2 && dt(3) > 0
            vx2 = (data.x(3) - data.x(2)) / dt(3);
            vy2 = (data.y(3) - data.y(2)) / dt(3);
            yaw2 = atan2(vy2, vx2);
            
            dyaw = angdiff(yaw2, initial_yaw);
            initial_yaw_rate = dyaw / dt(3);
            initial_yaw_rate = max(-0.1, min(0.1, initial_yaw_rate));
        end
    end
    
    % Use SOG/COG if available
    if ~isnan(data.SOG(1)) && ~isnan(data.COG(1))
        initial_speed = data.SOG(1);
        initial_yaw = deg2rad(data.COG(1));
    end
    
    % Create EKF filter
    ekf = extendedKalmanFilter(@ctrvStateTransitionFcn, @ctrvMeasurementFcn);
    ekf.StateTransitionJacobianFcn = @ctrvStateTransitionJacobianFcn;
    ekf.MeasurementJacobianFcn = @ctrvMeasurementJacobianFcn;
    
    % Initialize state and covariance
    ekf.State = [data.x(1); data.y(1); initial_speed; initial_yaw; initial_yaw_rate];
    ekf.StateCovariance = diag([pos_std^2/2, pos_std^2/2, vel_std^2 *4, deg2rad(5)^2, deg2rad(1)^2]);
    
    % Measurement noise (position only)
    R_pos = diag([pos_std^2, pos_std^2]);
    
    % Initialize output
    x_est = zeros(5, n);
    P_est = zeros(5, 5, n);
    
    % Initialize innovation tracking (optional outputs)
    if nargout >= 3
        innovations = zeros(2, n);        % For position measurements [x; y]
        S_innovations = zeros(2, 2, n);   % Innovation covariance matrices
    end
    
    x_est(:,1) = ekf.State;
    P_est(:,:,1) = ekf.StateCovariance;
    
    if nargout >= 3
        innovations(:,1) = NaN;  % No innovation for initial state
        S_innovations(:,:,1) = NaN;
    end
    
    % Main filtering loop
    for k = 2:n
        dt_k = dt(k);
        if dt_k <= 0
            dt_k = 1;
        end
        
        % Process noise matrix
        Q = q * diag([dt_k^3/3, dt_k^3/3, dt_k, deg2rad(0.5)^2*dt_k, deg2rad(0.2)^2*dt_k]);
        
        ekf.ProcessNoise = Q;
        
        % Prediction step
        predict(ekf, dt_k);
        
        % Normalize yaw angle
        ekf.State(4) = mod(ekf.State(4) + pi, 2*pi) - pi;
        
        % Measurement update
        if ~isnan(data.x(k)) && ~isnan(data.y(k))
            z = [data.x(k); data.y(k)];
            
            % Calculate innovation before correction (for NIS)
            if nargout >= 3
                H = ctrvMeasurementJacobianFcn(ekf.State);
                z_pred = ctrvMeasurementFcn(ekf.State);
                innovation = z - z_pred;
                S = H * ekf.StateCovariance * H' + R_pos;
                
                % Store innovation data
                innovations(:,k) = innovation;
                S_innovations(:,:,k) = S;
            end
            
            ekf.MeasurementNoise = R_pos;
            correct(ekf, z);
            
            % Normalize yaw angle after correction
            ekf.State(4) = mod(ekf.State(4) + pi, 2*pi) - pi;
        else
            if nargout >= 3
                innovations(:,k) = NaN;
                S_innovations(:,:,k) = NaN;
            end
        end
        
        x_est(:,k) = ekf.State;
        P_est(:,:,k) = ekf.StateCovariance;
    end
end

% CTRV State transition function
function x_pred = ctrvStateTransitionFcn(x, dt)
    px = x(1);
    py = x(2);
    v = x(3);
    yaw = x(4);
    yaw_rate = x(5);
    
    if abs(yaw_rate) < 1e-4
        px_pred = px + v * cos(yaw) * dt;
        py_pred = py + v * sin(yaw) * dt;
        yaw_pred = yaw;
    else
        px_pred = px + (v/yaw_rate) * (sin(yaw + yaw_rate*dt) - sin(yaw));
        py_pred = py + (v/yaw_rate) * (-cos(yaw + yaw_rate*dt) + cos(yaw));
        yaw_pred = yaw + yaw_rate * dt;
    end
    
    x_pred = [px_pred; py_pred; v; yaw_pred; yaw_rate];
end

% CTRV State transition Jacobian
function F = ctrvStateTransitionJacobianFcn(x, dt)
    v = x(3);
    yaw = x(4);
    yaw_rate = x(5);
    
    F = eye(5);
    
    if abs(yaw_rate) < 1e-4
        % Linear motion
        F(1,3) = cos(yaw) * dt;
        F(1,4) = -v * sin(yaw) * dt;
        F(2,3) = sin(yaw) * dt;
        F(2,4) = v * cos(yaw) * dt;
        F(4,5) = dt;
    else
        % Nonlinear motion
        sin_yaw = sin(yaw);
        cos_yaw = cos(yaw);
        sin_yaw_dt = sin(yaw + yaw_rate * dt);
        cos_yaw_dt = cos(yaw + yaw_rate * dt);
        
        F(1,3) = (sin_yaw_dt - sin_yaw) / yaw_rate;
        F(1,4) = (v/yaw_rate) * (cos_yaw_dt - cos_yaw);
        F(1,5) = (v/(yaw_rate^2)) * (sin_yaw - sin_yaw_dt) + (v*dt/yaw_rate) * cos_yaw_dt;
        
        F(2,3) = (-cos_yaw_dt + cos_yaw) / yaw_rate;
        F(2,4) = (v/yaw_rate) * (sin_yaw_dt - sin_yaw);
        F(2,5) = (v/(yaw_rate^2)) * (-cos_yaw + cos_yaw_dt) + (v*dt/yaw_rate) * sin_yaw_dt;
        
        F(4,5) = dt;
    end
end

% CTRV Measurement function
function z = ctrvMeasurementFcn(x)
    z = [x(1); x(2)];  % Position measurements only
end

% CTRV Measurement Jacobian
function H = ctrvMeasurementJacobianFcn(x)
    H = [1 0 0 0 0; 0 1 0 0 0];  % Position measurements only
end