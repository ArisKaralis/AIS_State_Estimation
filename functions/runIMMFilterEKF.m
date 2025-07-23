function [estimates, stats] = runIMMFilterEKF(data, q_cv, q_ca, q_ctrv)
    % RUNIMMFILTEREKF - Interacting Multiple Model filter using existing EKF implementations
    % Uses: runExtendedKalmanFilterCV, runExtendedKalmanFilterCA, runExtendedKalmanFilterCTRV
    
    fprintf('Starting IMM-EKF with process noise intensities: CV=%.3f, CA=%.3f, CTRV=%.3f\n', q_cv, q_ca, q_ctrv);
    
    n = height(data);
    num_models = 3;
    
    % IMM parameters
    model_probs = [0.4, 0.3, 0.3];  % Initial model probabilities [CV, CA, CTRV]
    
    % Model transition matrix
    transition_matrix = [0.95, 0.03, 0.02;   % CV -> CV, CA, CTRV
                        0.03, 0.95, 0.02;   % CA -> CV, CA, CTRV  
                        0.02, 0.03, 0.95];  % CTRV -> CV, CA, CTRV
    
    % Initialize storage
    estimates = struct();
    estimates.time = data.timestamp;
    estimates.x = zeros(n, 1);
    estimates.y = zeros(n, 1);
    estimates.vx = zeros(n, 1);
    estimates.vy = zeros(n, 1);
    estimates.sog = zeros(n, 1);
    estimates.cog = zeros(n, 1);
    
    % Model probabilities history
    model_probs_history = zeros(num_models, n);
    model_probs_history(:, 1) = model_probs';
    
    % Run each filter individually to get all estimates
    fprintf('Running individual EKF models...\n');
    [cv_estimates, ~] = runExtendedKalmanFilterCV(data, q_cv);
    [ca_estimates, ~] = runExtendedKalmanFilterCA(data, q_ca);
    [ctrv_estimates, ~] = runExtendedKalmanFilterCTRV(data, q_ctrv);
    
    % Store initial estimates (first time step)
    estimates.x(1) = data.x(1);
    estimates.y(1) = data.y(1);
    estimates.vx(1) = 0;
    estimates.vy(1) = 0;
    estimates.sog(1) = data.SOG(1);
    estimates.cog(1) = data.COG(1);
    
    % IMM combination for each time step
    for k = 2:n
        % Get model estimates at time k
        model_estimates = {
            [cv_estimates.x_est(k); cv_estimates.y_est(k); cv_estimates.vx_est(k); cv_estimates.vy_est(k)],
            [ca_estimates.x_est(k); ca_estimates.y_est(k); ca_estimates.vx_est(k); ca_estimates.vy_est(k)],
            [ctrv_estimates.x_est(k); ctrv_estimates.y_est(k); ctrv_estimates.vx_est(k); ctrv_estimates.vy_est(k)]
        };
        
        % Calculate measurement likelihoods for each model
        likelihoods = calculateLikelihoods(data, k, model_estimates);
        
        % Update model probabilities
        predicted_probs = transition_matrix * model_probs';
        model_probs = (predicted_probs .* likelihoods);
        model_probs = model_probs / sum(model_probs);
        model_probs = model_probs';
        
        % Store model probabilities
        model_probs_history(:, k) = model_probs';
        
        % Combine estimates using model probabilities
        combined_state = zeros(4, 1);
        for m = 1:num_models
            combined_state = combined_state + model_probs(m) * model_estimates{m};
        end
        
        % Store combined estimates
        estimates.x(k) = combined_state(1);
        estimates.y(k) = combined_state(2);
        estimates.vx(k) = combined_state(3);
        estimates.vy(k) = combined_state(4);
        
        % Calculate SOG and COG
        estimates.sog(k) = sqrt(combined_state(3)^2 + combined_state(4)^2);
        estimates.cog(k) = rad2deg(atan2(combined_state(3), combined_state(4)));
        if estimates.cog(k) < 0
            estimates.cog(k) = estimates.cog(k) + 360;
        end
    end
    
    % Store model probabilities
    estimates.model_probs_cv = model_probs_history(1, :)';
    estimates.model_probs_ca = model_probs_history(2, :)';
    estimates.model_probs_ctrv = model_probs_history(3, :)';
    
    % Calculate error statistics
    pos_errors = sqrt((estimates.x - data.x_true).^2 + (estimates.y - data.y_true).^2);
    vel_errors = sqrt((estimates.vx - data.vx_true).^2 + (estimates.vy - data.vy_true).^2);
    sog_errors = abs(estimates.sog - data.sog_true);
    cog_errors = abs(estimates.cog - data.cog_true);
    
    % Handle COG wraparound
    cog_errors = min(cog_errors, 360 - cog_errors);
    
    stats = struct();
    stats.position_rmse = sqrt(mean(pos_errors.^2));
    stats.velocity_rmse = sqrt(mean(vel_errors.^2));
    stats.sog_rmse = sqrt(mean(sog_errors.^2));
    stats.cog_rmse = sqrt(mean(cog_errors.^2));
    stats.mean_pos_error = mean(pos_errors);
    stats.max_pos_error = max(pos_errors);
    stats.mean_vel_error = mean(vel_errors);
    stats.max_vel_error = max(vel_errors);
    
    fprintf('IMM-EKF completed successfully\n');
    fprintf('Position RMSE: %.2f m, Velocity RMSE: %.2f m/s, SOG RMSE: %.2f m/s, COG RMSE: %.2f deg\n', ...
            stats.position_rmse, stats.velocity_rmse, stats.sog_rmse, stats.cog_rmse);
    
    % Print model switching summary
    fprintf('\n--- Model Selection Summary ---\n');
    fprintf('EKF-CV Model:   %.1f%% average probability\n', mean(estimates.model_probs_cv) * 100);
    fprintf('EKF-CA Model:   %.1f%% average probability\n', mean(estimates.model_probs_ca) * 100); 
    fprintf('EKF-CTRV Model: %.1f%% average probability\n', mean(estimates.model_probs_ctrv) * 100);
    
    % Determine dominant model for each time step
    [~, dominant_models] = max([estimates.model_probs_cv, estimates.model_probs_ca, estimates.model_probs_ctrv], [], 2);
    cv_dominant = sum(dominant_models == 1) / n * 100;
    ca_dominant = sum(dominant_models == 2) / n * 100;
    ctrv_dominant = sum(dominant_models == 3) / n * 100;
    
    fprintf('EKF-CV dominant:   %.1f%% of time\n', cv_dominant);
    fprintf('EKF-CA dominant:   %.1f%% of time\n', ca_dominant); 
    fprintf('EKF-CTRV dominant: %.1f%% of time\n', ctrv_dominant);
end

function likelihoods = calculateLikelihoods(data, k, model_estimates)
    % Calculate measurement likelihood for each model
    % Simple Gaussian likelihood based on position measurement
    
    pos_noise_std = 15; % meters (measurement noise)
    num_models = length(model_estimates);
    likelihoods = zeros(num_models, 1);
    
    % Actual measurement
    z_actual = [data.x(k); data.y(k)];
    
    for m = 1:num_models
        % Predicted measurement from model
        z_pred = [model_estimates{m}(1); model_estimates{m}(2)];
        
        % Innovation
        innovation = z_actual - z_pred;
        
        % Measurement covariance
        R = pos_noise_std^2 * eye(2);
        
        % Gaussian likelihood
        try
            likelihoods(m) = exp(-0.5 * innovation' * (R \ innovation)) / sqrt((2*pi)^2 * det(R));
        catch
            likelihoods(m) = 1e-10; % Small value if computation fails
        end
        
        % Ensure minimum likelihood
        if likelihoods(m) < 1e-10
            likelihoods(m) = 1e-10;
        end
    end
    
    % Normalize likelihoods to prevent numerical issues
    likelihoods = likelihoods / sum(likelihoods);
end