function stats = calculateFilterStats(estimates, data, filterName, P_est, innovations, S_innovations)
    % CALCULATEFILTERSTATS - Calculate error metrics for filter estimates.
    % Computes position/velocity errors (if ground truth available),
    % position-only NEES (if P_est provided), and NIS (if innovations + S provided).

    if nargin < 4, P_est = []; end
    if nargin < 5, innovations = []; end
    if nargin < 6, S_innovations = []; end

    stats = struct();
    stats.filterName = filterName;

    hasPosGT = all(ismember({'x_true', 'y_true'}, data.Properties.VariableNames));
    hasVelGT = all(ismember({'vx_true', 'vy_true'}, data.Properties.VariableNames));
    hasSOGCOG = all(ismember({'sog_true', 'cog_true'}, data.Properties.VariableNames));

    if hasPosGT
        % Position error
        pos_error = sqrt((estimates.x_est - data.x_true).^2 + ...
                         (estimates.y_est - data.y_true).^2);

        stats.position_rmse = sqrt(mean(pos_error.^2));
        stats.position_mean = mean(pos_error);
        stats.position_max = max(pos_error);
        stats.position_std = std(pos_error);
        stats.position_errors = pos_error;

        % Velocity error
        if hasVelGT
            vel_error = sqrt((estimates.vx_est - data.vx_true).^2 + ...
                             (estimates.vy_est - data.vy_true).^2);
            stats.velocity_rmse = sqrt(mean(vel_error.^2));
            stats.velocity_mean = mean(vel_error);
            stats.velocity_max = max(vel_error);
        else
            stats.velocity_rmse = NaN;
            stats.velocity_mean = NaN;
            stats.velocity_max = NaN;
        end

        % SOG/COG errors
        if hasSOGCOG
            stats.sog_rmse = sqrt(mean((estimates.sog_est - data.sog_true).^2));
            stats.cog_rmse = sqrt(mean((angdiff(deg2rad(estimates.cog_est), ...
                                                deg2rad(data.cog_true)) * 180/pi).^2));
        else
            stats.sog_rmse = NaN;
            stats.cog_rmse = NaN;
        end

        % Position-only NEES
        if ~isempty(P_est)
            stats = calculatePositionNEES(stats, estimates, data, P_est);
        else
            stats.nees_values = [];
            stats.nees_mean = NaN;
            stats.nees_median = NaN;
            stats.nees_std = NaN;
            stats.nees_consistency = NaN;
            stats.nees_chi2_lower = NaN;
            stats.nees_chi2_upper = NaN;
            stats.nees_within_bounds = false;
        end

        % NIS
        if ~isempty(innovations) && ~isempty(S_innovations)
            stats = calculateNIS(stats, innovations, S_innovations);
        else
            stats.nis_values = [];
            stats.nis_mean = NaN;
            stats.nis_median = NaN;
            stats.nis_std = NaN;
            stats.nis_consistency = NaN;
            stats.nis_chi2_lower = NaN;
            stats.nis_chi2_upper = NaN;
            stats.nis_within_bounds = false;
        end

    else
        % No ground truth
        stats.position_rmse = NaN;
        stats.position_mean = NaN;
        stats.position_max = NaN;
        stats.position_std = NaN;
        stats.position_errors = [];

        stats.velocity_rmse = NaN;
        stats.velocity_mean = NaN;
        stats.velocity_max = NaN;

        stats.sog_rmse = NaN;
        stats.cog_rmse = NaN;

        stats.nees_values = [];
        stats.nees_mean = NaN;
        stats.nees_median = NaN;
        stats.nees_std = NaN;
        stats.nees_consistency = NaN;
        stats.nees_chi2_lower = NaN;
        stats.nees_chi2_upper = NaN;
        stats.nees_within_bounds = false;

        stats.nis_values = [];
        stats.nis_mean = NaN;
        stats.nis_median = NaN;
        stats.nis_std = NaN;
        stats.nis_consistency = NaN;
        stats.nis_chi2_lower = NaN;
        stats.nis_chi2_upper = NaN;
        stats.nis_within_bounds = false;
    end
end

function stats = calculatePositionNEES(stats, estimates, data, P_est)
    % Position-Only NEES (x,y).
    n = length(estimates.x_est);
    nees_values = NaN(n, 1);

    for k = 1:n
        x_true = [data.x_true(k); data.y_true(k)];
        x_est = [estimates.x_est(k); estimates.y_est(k)];
        position_error = x_est - x_true;

        P_k = P_est(:, :, k);

        % Determine position indices by model dimension
        switch size(P_k, 1)
            case 4 % [x vx y vy]
                pos_indices = [1, 3];
            case 5 % [x y v yaw yaw_rate]
                pos_indices = [1, 2];
            case 6 % [x vx ax y vy ay]
                pos_indices = [1, 4];
            otherwise
                pos_indices = [1, 2];
        end

        P_pos = P_k(pos_indices, pos_indices);

        try
            if rcond(P_pos) < 1e-12
                P_pos = P_pos + eye(size(P_pos)) * 1e-10;
            end
            nees_values(k) = position_error' * (P_pos \ position_error);
            if ~isreal(nees_values(k)) || nees_values(k) < 0
                nees_values(k) = NaN;
            end
        catch
            nees_values(k) = NaN;
        end
    end

    valid_nees = nees_values(~isnan(nees_values));
    if ~isempty(valid_nees)
        stats.nees_values = nees_values;
        stats.nees_mean = mean(valid_nees);
        stats.nees_median = median(valid_nees);
        stats.nees_std = std(valid_nees);

        expected_nees = 2.0; % 2D position
        stats.nees_consistency = abs(stats.nees_mean - expected_nees) / expected_nees;

        alpha = 0.05;
        n_valid = numel(valid_nees);
        dof = 2;
        chi2_lower = chi2inv(alpha/2, dof * n_valid) / n_valid;
        chi2_upper = chi2inv(1 - alpha/2, dof * n_valid) / n_valid;

        stats.nees_chi2_lower = chi2_lower;
        stats.nees_chi2_upper = chi2_upper;
        stats.nees_within_bounds = (stats.nees_mean >= chi2_lower) && (stats.nees_mean <= chi2_upper);
    else
        stats.nees_values = nees_values;
        stats.nees_mean = NaN;
        stats.nees_median = NaN;
        stats.nees_std = NaN;
        stats.nees_consistency = NaN;
        stats.nees_chi2_lower = NaN;
        stats.nees_chi2_upper = NaN;
        stats.nees_within_bounds = false;
    end
end

function stats = calculateNIS(stats, innovations, S_innovations)
    % Normalized Innovation Squared.
    n = size(innovations, 2);
    nis_values = NaN(n, 1);

    for k = 1:n
        if any(isnan(innovations(:, k))) || any(isnan(S_innovations(:, :, k)), 'all')
            nis_values(k) = NaN;
            continue;
        end
        try
            S_k = S_innovations(:, :, k);
            if rcond(S_k) < 1e-12
                S_k = S_k + eye(size(S_k)) * 1e-10;
            end
            nis_values(k) = innovations(:, k)' * (S_k \ innovations(:, k));
            if ~isreal(nis_values(k)) || nis_values(k) < 0
                nis_values(k) = NaN;
            end
        catch
            nis_values(k) = NaN;
        end
    end

    valid_nis = nis_values(~isnan(nis_values));
    if ~isempty(valid_nis)
        stats.nis_values = nis_values;
        stats.nis_mean = mean(valid_nis);
        stats.nis_median = median(valid_nis);
        stats.nis_std = std(valid_nis);

        expected_nis = size(innovations, 1);
        stats.nis_consistency = abs(stats.nis_mean - expected_nis) / expected_nis;

        alpha = 0.05;
        n_valid = numel(valid_nis);
        dof = size(innovations, 1);
        chi2_lower = chi2inv(alpha/2, dof * n_valid) / n_valid;
        chi2_upper = chi2inv(1 - alpha/2, dof * n_valid) / n_valid;

        stats.nis_chi2_lower = chi2_lower;
        stats.nis_chi2_upper = chi2_upper;
        stats.nis_within_bounds = (stats.nis_mean >= chi2_lower) && (stats.nis_mean <= chi2_upper);
    else
        stats.nis_values = nis_values;
        stats.nis_mean = NaN;
        stats.nis_median = NaN;
        stats.nis_std = NaN;
        stats.nis_consistency = NaN;
        stats.nis_chi2_lower = NaN;
        stats.nis_chi2_upper = NaN;
        stats.nis_within_bounds = false;
    end
end