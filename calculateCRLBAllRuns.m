function calculateCRLBAllRuns()
% CALCULATECRLBALLRUNS
% Computes CRLBs per run for CV/CA/CTRV with per-step Q(dt),
% meters-only coordinates, and either position-only or position+SOG(+COG)
% measurement models. Uses PCRLB for nonlinear cases.
addpath('monte_carlo');
fprintf('Loading Monte Carlo results...\n');
S = load('data/monte_carlo_results.mat', 'results');
results = S.results;
num_runs = numel(results.runs);

% ====== MEASUREMENT MODEL ======
% Choose: 'pos', 'pos_sog', or 'pos_sog_cog'
meas_model = 'pos_sog_cog';

% ====== MEASUREMENT NOISE ======
pos_noise_m        = 5.0;               % meters (per axis)
vel_noise_ms       = 0.5;               % m/s (SOG)
course_noise_rad   = deg2rad(5.0);      % radians (COG)

R_params = struct('pos', pos_noise_m, ...
                  'vel', vel_noise_ms, ...
                  'cog', course_noise_rad, ...
                  'meas_model', meas_model);

% ====== PROCESS NOISE (INTENSITIES) ======
% Use the SAME intensities you used to SIMULATE (truth-CRLB).
q_params_cv   = struct('q_acc', 0.25);                 % CV: [m^2/s^3]
q_params_ca   = struct('q_jerk', 0.5);                 % CA: [m^2/s^5]
q_params_ctrv = struct('q_a', 0.5, 'q_nu', 0.05, ...   % CTRV: v-acc, yaw-acc
                       'epsQ', 1e-12);

% Output collectors
cv_rows = [];  ca_rows = [];  ctrv_rows = [];
cv_ok = 0;     ca_ok = 0;     ctrv_ok = 0;

for run = 1:num_runs
    try
        gt  = results.runs{run}.groundTruth;
        obs = results.runs{run}.observations;

        % Align GT with available observations
        valid_idx = find(obs.available);
        gt_idx    = obs.groundTruthIndices(valid_idx);

        mask = gt_idx >= 1 & gt_idx <= size(gt.position,2);
        if nnz(mask) < 3, continue; end

        valid_idx = valid_idx(mask);
        gt_idx    = gt_idx(mask);

        % === Build data in meters (no lat/lon round-trip) ===
        data = struct();

        % Force everything to COLUMN vectors with (:)
        xv = gt.position(1, gt_idx);
        yv = gt.position(2, gt_idx);
        tv = obs.time(valid_idx);

        % Time to seconds (numeric) then column
        if isduration(tv), tv = seconds(tv); end
        if isdatetime(tv), tv = seconds(tv - tv(1)); end
        data.time = tv(:);
        data.x    = xv(:);
        data.y    = yv(:);

        % SOG/COG if measurement model needs them
        needs_sog = any(strcmp(meas_model, {'pos_sog','pos_sog_cog'}));
        needs_cog = strcmp(meas_model, 'pos_sog_cog');

        if needs_sog
            if isfield(gt,'speed') && ~isempty(gt.speed)
                data.sog = gt.speed(gt_idx);
                data.sog = data.sog(:);
            else
                dt = diff(data.time); dt(dt<=0) = eps;
                vx = [0; diff(data.x)./dt];
                vy = [0; diff(data.y)./dt];
                data.sog = hypot(vx,vy);
            end
        end

        if needs_cog
            if isfield(gt,'heading') && ~isempty(gt.heading)
                data.cog = rad2deg(gt.heading(gt_idx));
                data.cog = data.cog(:);
            else
                dt = diff(data.time); dt(dt<=0) = eps;
                vx = [0; diff(data.x)./dt];
                vy = [0; diff(data.y)./dt];
                data.cog = rad2deg(unwrap(atan2(vy, vx)));
            end
        end

        % ---- Length & NaN/Inf checks (conditional on meas_model) ----
        base_lengths = [numel(data.x), numel(data.y), numel(data.time)];
        ok_len = all(base_lengths == base_lengths(1));

        if needs_sog, ok_len = ok_len && (numel(data.sog) == base_lengths(1)); end
        if needs_cog, ok_len = ok_len && (numel(data.cog) == base_lengths(1)); end
        if ~ok_len
            fprintf('Run %d: length mismatch, skipping\n', run); continue;
        end

        vstack = [data.x; data.y; data.time];
        if needs_sog, vstack = [vstack; data.sog]; end
        if needs_cog, vstack = [vstack; data.cog]; end

        if any(isnan(vstack)) || any(isinf(vstack))
            fprintf('Run %d: NaN/Inf in data, skipping\n', run); continue;
        end

    catch ME
        fprintf('Run %d: data prep failed: %s\n', run, ME.message);
        continue;
    end

    % ===== CV =====
    try
        crlb_cv = calculateCRLB(data, 'CV', q_params_cv, R_params);
        cv_rows = [cv_rows; crlb_cv.final_position_crlb, crlb_cv.final_velocity_crlb];
        cv_ok = cv_ok + 1;
    catch ME
        if run <= 5 || mod(run,20)==0
            fprintf('Run %d CV failed: %s\n', run, ME.message);
        end
    end

    % ===== CA =====
    try
        crlb_ca = calculateCRLB(data, 'CA', q_params_ca, R_params);
        ca_rows = [ca_rows; crlb_ca.final_position_crlb, crlb_ca.final_velocity_crlb];
        ca_ok = ca_ok + 1;
    catch ME
        if run <= 5 || mod(run,20)==0
            fprintf('Run %d CA failed: %s\n', run, ME.message);
        end
    end

    % ===== CTRV =====
    try
        crlb_ctrv = calculateCRLB(data, 'CTRV', q_params_ctrv, R_params);
        ctrv_rows = [ctrv_rows; crlb_ctrv.final_position_crlb, crlb_ctrv.final_velocity_crlb];
        ctrv_ok = ctrv_ok + 1;
    catch ME
        if run <= 5 || mod(run,20)==0
            fprintf('Run %d CTRV failed: %s\n', run, ME.message);
        end
    end

    if mod(run,20)==0
        fprintf('Completed run %d/%d (CV:%d CA:%d CTRV:%d ok)\n', run, num_runs, cv_ok, ca_ok, ctrv_ok);
    end
end

% ===== Summary =====
fprintf('\n===== CRLB RESULTS =====\n');
fprintf('Success: CV %d/%d  CA %d/%d  CTRV %d/%d\n', cv_ok,num_runs, ca_ok,num_runs, ctrv_ok,num_runs);
fprintf('Measurement model: %s | R: pos=%.1fm, vel=%.1fm/s, cog=%.1f°\n', ...
        meas_model, pos_noise_m, vel_noise_ms, rad2deg(course_noise_rad));

if ~isempty(cv_rows)
    fprintf('\nCV (pos RMSE bound [m], speed RMSE bound [m/s])\n');
    fprintf('  %.2f ± %.2f   |   %.2f ± %.2f\n', mean(cv_rows(:,1)), std(cv_rows(:,1)), ...
                                               mean(cv_rows(:,2)), std(cv_rows(:,2)));
end
if ~isempty(ca_rows)
    fprintf('\nCA (pos, speed)\n');
    fprintf('  %.2f ± %.2f   |   %.2f ± %.2f\n', mean(ca_rows(:,1)), std(ca_rows(:,1)), ...
                                               mean(ca_rows(:,2)), std(ca_rows(:,2)));
end
if ~isempty(ctrv_rows)
    fprintf('\nCTRV (pos, speed)\n');
    fprintf('  %.2f ± %.2f   |   %.2f ± %.2f\n', mean(ctrv_rows(:,1)), std(ctrv_rows(:,1)), ...
                                               mean(ctrv_rows(:,2)), std(ctrv_rows(:,2)));
end

results_crlb = struct();
results_crlb.cv   = cv_rows;
results_crlb.ca   = ca_rows;
results_crlb.ctrv = ctrv_rows;
results_crlb.success = struct('cv',cv_ok/num_runs,'ca',ca_ok/num_runs,'ctrv',ctrv_ok/num_runs);
results_crlb.R_params = R_params;
results_crlb.q_params = struct('cv',q_params_cv,'ca',q_params_ca,'ctrv',q_params_ctrv);

if ~exist('output','dir'), mkdir('output'); end
save('output/crlb_results.mat','results_crlb');
fprintf('\nSaved: output/crlb_results.mat\n');
end
