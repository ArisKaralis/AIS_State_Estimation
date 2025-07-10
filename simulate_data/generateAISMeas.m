function z_meas = generateAISMeas(x_true, sigma_pos)
    n = size(x_true, 2);
    noise = sigma_pos * randn(2, n);
    z_meas = x_true([1 3], :) + noise;  % Noisy [x; y]
end
