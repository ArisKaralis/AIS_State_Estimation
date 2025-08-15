function [estimates, stats] = runIMMFilterEKF(data, q_cv, q_ca, q_ctrv, pos_std, vel_std, acc_std)
    % RUNIMMFILTEREKF - IMM filter using existing modular EKF implementations
    % Uses ekfCV.m, ekfCA.m, and ekfCTRV.m directly with NEES/NIS evaluation
    
    % fprintf('Starting IMM-EKF with process noise intensities: CV=%.3f, CA=%.3f, CTRV=%.3f\n', q_cv, q_ca, q_ctrv);
    
    n = height(data);
    num_models = 3;
    
    % IMM parameters
    model_probs = [0.95, 0.05, 0.05];  % Initial probabilities [CV, CA, CTRV]
    transition_matrix = [0.95, 0.025, 0.025;   % CV -> [CV, CA, CTRV]
                        0.025, 0.95, 0.025;    % CA -> [CV, CA, CTRV]
                        0.025, 0.025, 0.95];   % CTRV -> [CV, CA, CTRV]
    
    % Run each filter separately using existing functions with innovation tracking
    % fprintf('Running individual EKF models with innovation tracking...\n');
    [x_est_cv, P_est_cv, innovations_cv, S_innovations_cv] = ekfCV(data, q_cv, pos_std, vel_std);
    [x_est_ca, P_est_ca, innovations_ca, S_innovations_ca] = ekfCA(data, q_ca, pos_std, vel_std, acc_std);
    [x_est_ctrv, P_est_ctrv, innovations_ctrv, S_innovations_ctrv] = ekfCTRV(data, q_ctrv, pos_std, vel_std);
    
    % Initialize storage for IMM results
    estimates = struct();
    estimates.time = data.timestamp;
    estimates.x_est = zeros(n, 1);
    estimates.y_est = zeros(n, 1);
    estimates.vx_est = zeros(n, 1);
    estimates.vy_est = zeros(n, 1);
    estimates.sog_est = zeros(n, 1);
    estimates.cog_est = zeros(n, 1);
    
    % Model probabilities history
    model_probs_history = zeros(num_models, n);
    model_probs_history(:, 1) = model_probs';
    
    % Initialize NEES and NIS storage for IMM
    estimates.nees = zeros(n, 1);
    estimates.nis = zeros(n, 1);
    estimates.combined_P = zeros(4, 4, n);  % Combined covariance for NEES
    estimates.combined_innovations = zeros(2, n);  % Combined innovations for NIS
    estimates.combined_S = zeros(2, 2, n);  % Combined innovation covariances for NIS
    
    % Initialize first estimate
    estimates.x_est(1) = data.x(1);
    estimates.y_est(1) = data.y(1);
    estimates.vx_est(1) = x_est_cv(2, 1);  % Initial velocity from CV model
    estimates.vy_est(1) = x_est_cv(4, 1);
    estimates.sog_est(1) = sqrt(estimates.vx_est(1)^2 + estimates.vy_est(1)^2);
    estimates.cog_est(1) = mod(rad2deg(atan2(estimates.vy_est(1), estimates.vx_est(1))), 360);
    
    % First timestep NEES/NIS (use CV model as reference)
    P_cv_init = P_est_cv(:,:,1);
    true_state_init = [data.x_true(1); x_est_cv(2,1); data.y_true(1); x_est_cv(4,1)];
    est_state_init = [estimates.x_est(1); estimates.vx_est(1); estimates.y_est(1); estimates.vy_est(1)];
    error_init = est_state_init - true_state_init;
    estimates.nees(1) = error_init' * (P_cv_init \ error_init);
    estimates.nis(1) = NaN;  % No innovation for first timestep
    estimates.combined_P(:,:,1) = P_cv_init;
    
    % IMM combination for each time step
    for k = 2:n
        % Extract states from each model at time k
        state_cv = [x_est_cv(1,k); x_est_cv(2,k); x_est_cv(3,k); x_est_cv(4,k)];  % [x, vx, y, vy]
        state_ca = [x_est_ca(1,k); x_est_ca(2,k); x_est_ca(4,k); x_est_ca(5,k)];  % [x, vx, y, vy] from [x,vx,ax,y,vy,ay]
        
        % Convert CTRV state to CV format for combination
        px_ctrv = x_est_ctrv(1,k);
        py_ctrv = x_est_ctrv(2,k);
        v_ctrv = x_est_ctrv(3,k);
        yaw_ctrv = x_est_ctrv(4,k);
        vx_ctrv = v_ctrv * cos(yaw_ctrv);
        vy_ctrv = v_ctrv * sin(yaw_ctrv);
        state_ctrv = [px_ctrv; vx_ctrv; py_ctrv; vy_ctrv];
        
        % Extract covariances (convert to position-velocity format)
        P_cv = P_est_cv(:,:,k);
        
        % Convert CA covariance to CV format
        P_ca_full = P_est_ca(:,:,k);
        P_ca = [P_ca_full(1,1), P_ca_full(1,2), P_ca_full(1,4), P_ca_full(1,5);
                P_ca_full(2,1), P_ca_full(2,2), P_ca_full(2,4), P_ca_full(2,5);
                P_ca_full(4,1), P_ca_full(4,2), P_ca_full(4,4), P_ca_full(4,5);
                P_ca_full(5,1), P_ca_full(5,2), P_ca_full(5,4), P_ca_full(5,5)];
        
        % Convert CTRV covariance to CV format (approximate)
        P_ctrv_full = P_est_ctrv(:,:,k);
        % Jacobian for CTRV to CV transformation
        J = [1, 0, 0, 0, 0;                           % x
             0, 0, cos(yaw_ctrv), -v_ctrv*sin(yaw_ctrv), 0;  % vx
             0, 1, 0, 0, 0;                           % y  
             0, 0, sin(yaw_ctrv), v_ctrv*cos(yaw_ctrv), 0];   % vy
        P_ctrv = J * P_ctrv_full * J';
        
        % Calculate model likelihoods based on innovation
        z = [data.x(k); data.y(k)];
        H = [1, 0, 0, 0; 0, 0, 1, 0];  % Measurement matrix for position
        R = diag([pos_std^2, pos_std^2]);
        
        % Innovation and likelihood for each model
        innovation_cv = z - H * state_cv;
        S_cv = H * P_cv * H' + R;
        likelihood_cv = calculateGaussianLikelihood(innovation_cv, S_cv);
        
        innovation_ca = z - H * state_ca;
        S_ca = H * P_ca * H' + R;
        likelihood_ca = calculateGaussianLikelihood(innovation_ca, S_ca);
        
        innovation_ctrv = z - H * state_ctrv;
        S_ctrv = H * P_ctrv * H' + R;
        likelihood_ctrv = calculateGaussianLikelihood(innovation_ctrv, S_ctrv);
        
        likelihoods = [likelihood_cv; likelihood_ca; likelihood_ctrv];
        
        % Handle numerical issues
        likelihoods = max(likelihoods, 1e-10);
        
        % Update model probabilities
        predicted_probs = transition_matrix * model_probs';
        unnormalized_probs = predicted_probs .* likelihoods;
        
        if sum(unnormalized_probs) < 1e-10
            model_probs = [1/3, 1/3, 1/3];
        else
            model_probs = unnormalized_probs / sum(unnormalized_probs);
            model_probs = model_probs';
        end
        
        model_probs_history(:, k) = model_probs';
        
        % Combine estimates using model probabilities
        combined_state = model_probs(1) * state_cv + ...
                        model_probs(2) * state_ca + ...
                        model_probs(3) * state_ctrv;
        
        % Combine covariances using model probabilities (for NEES calculation)
        combined_P = model_probs(1) * P_cv + ...
                    model_probs(2) * P_ca + ...
                    model_probs(3) * P_ctrv;
        
        % Add interaction uncertainty (simplified)
        for i = 1:3
            state_i = [state_cv, state_ca, state_ctrv];
            diff_i = state_i(:,i) - combined_state;
            combined_P = combined_P + model_probs(i) * (diff_i * diff_i');
        end
        estimates.combined_P(:,:,k) = combined_P;
        
        % Combine innovations using model probabilities (for NIS calculation)
        combined_innovation = model_probs(1) * innovation_cv + ...
                             model_probs(2) * innovation_ca + ...
                             model_probs(3) * innovation_ctrv;
        
        % Combine innovation covariances
        combined_S = model_probs(1) * S_cv + ...
                    model_probs(2) * S_ca + ...
                    model_probs(3) * S_ctrv;
        
        estimates.combined_innovations(:,k) = combined_innovation;
        estimates.combined_S(:,:,k) = combined_S;
        
        % Store combined estimates
        estimates.x_est(k) = combined_state(1);
        estimates.vx_est(k) = combined_state(2);
        estimates.y_est(k) = combined_state(3);
        estimates.vy_est(k) = combined_state(4);
        
        % Calculate SOG and COG
        estimates.sog_est(k) = sqrt(combined_state(2)^2 + combined_state(4)^2);
        cog_rad = atan2(combined_state(4), combined_state(2));
        estimates.cog_est(k) = mod(rad2deg(cog_rad), 360);
        
        % Calculate NEES for this timestep
        true_state = [data.x_true(k); data.vx_true(k); data.y_true(k); data.vy_true(k)];
        error = combined_state - true_state;
        
        try
            estimates.nees(k) = error' * (combined_P \ error);
        catch
            estimates.nees(k) = NaN;
        end
        
        % Calculate NIS for this timestep
        try
            estimates.nis(k) = combined_innovation' * (combined_S \ combined_innovation);
        catch
            estimates.nis(k) = NaN;
        end
    end
    
    % Store model probabilities for analysis
    estimates.model_probs_cv = model_probs_history(1, :)';
    estimates.model_probs_ca = model_probs_history(2, :)';
    estimates.model_probs_ctrv = model_probs_history(3, :)';
    
    % Calculate statistics with NEES/NIS
    stats = calculateFilterStats(estimates, data, 'IMM-EKF-3');
    
    % Additional error calculation
    pos_errors = sqrt((estimates.x_est - data.x_true).^2 + (estimates.y_est - data.y_true).^2);
    vel_errors = sqrt((estimates.vx_est - data.vx_true).^2 + (estimates.vy_est - data.vy_true).^2);
    sog_errors = abs(estimates.sog_est - data.sog_true);
    cog_errors = angdiff(deg2rad(estimates.cog_est), deg2rad(data.cog_true)) * 180/pi;
    cog_errors = abs(cog_errors);
    
    stats.position_rmse = sqrt(mean(pos_errors.^2));
    stats.velocity_rmse = sqrt(mean(vel_errors.^2));
    stats.sog_rmse = sqrt(mean(sog_errors.^2));
    stats.cog_rmse = sqrt(mean(cog_errors.^2));
    

    % Generate plots using the dedicated plotting function
    % plotIMMResults(data, estimates, stats, 'IMM-3', {'CV', 'CA', 'CTRV'});
end

function likelihood = calculateGaussianLikelihood(innovation, S)
    % Calculate Gaussian likelihood
    try
        det_S = det(S);
        if det_S > 1e-10
            inv_S = inv(S);
            likelihood = exp(-0.5 * innovation' * inv_S * innovation) / sqrt((2*pi)^2 * det_S);
        else
            likelihood = 1e-10;
        end
        
        % Ensure reasonable bounds
        likelihood = max(likelihood, 1e-10);
        likelihood = min(likelihood, 1e10);
        
    catch
        likelihood = 1e-10;
    end
end