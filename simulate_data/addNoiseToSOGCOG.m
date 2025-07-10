function [sog_meas, cog_meas, sog_true, cog_true] = addNoiseToSOGCOG(x_true, sigma_sog, sigma_cog)
    vx = x_true(2,:);
    vy = x_true(4,:);
    sog_true = sqrt(vx.^2 + vy.^2);
    cog_true = atan2d(vy, vx);
    cog_true = mod(cog_true, 360);

    n = length(sog_true);

    % === Colored noise via low-pass filter ===
    smooth_sog_noise = filter(0.2, [1 -0.8], randn(1, n)) * sigma_sog;
    smooth_cog_noise = filter(0.2, [1 -0.8], randn(1, n)) * sigma_cog;

    % === Sparse impulses (jumps) ===
    sog_spikes = zeros(1, n);
    cog_spikes = zeros(1, n);
    jump_indices = randperm(n, round(0.01 * n));  % 1% sparse jumps
    sog_spikes(jump_indices) = (rand(1, length(jump_indices)) - 0.5) * 3;  % ±1.5 m/s
    cog_spikes(jump_indices) = (rand(1, length(jump_indices)) - 0.5) * 20; % ±10 deg

    % === Apply noise ===
    sog_meas = sog_true + smooth_sog_noise + sog_spikes;
    % === Add noise in angular-safe way ===
    
    % Step 1: Total noise (smooth + impulse)
    cog_noise = smooth_cog_noise + cog_spikes;
    
    % Step 2: Wrap noise to [-180, 180]
    cog_noise = mod(cog_noise + 180, 360) - 180;
    
    % Step 3: Add noise and wrap result to [0, 360]
    cog_meas = mod(cog_true + cog_noise + 360, 360);


    figure;
    subplot(2,1,1); plot(sog_true); hold on; plot(sog_meas, '--'); title('SOG (True vs Measured)');
    subplot(2,1,2); plot(cog_true); hold on; plot(cog_meas, '--'); title('COG (True vs Measured)');
    figure;
    subplot(2,1,1); plot(wrapTo180(cog_meas - cog_true)); title('COG Measurement Error (wrapped)');
    ylabel('degrees'); xlabel('Time Step'); grid on;

    % === Compute SOG measurement error ===
    sog_error = sog_meas - sog_true;
    
    % === Plot SOG error ===
    figure;
    plot(sog_error, 'LineWidth', 1.2);
    title('SOG Measurement Error');
    xlabel('Time Step');
    ylabel('m/s');
    grid on;

end
