function [estimates, stats] = runEKFCA(data, q, pos_std, vel_std, acc_std)

if nargin < 2
    q = 0.1;
end

fprintf('\nEKF-CA: Process noise intensity: %.3f\n', q);


% Run core filter
[x_est, P_est] = ekfCA(data, q, pos_std, vel_std, acc_std);

% Create estimates table
estimates = createEstimatesTable(data.timestamp, x_est, 'ca');

% Calculate statistics
stats = calculateFilterStats(estimates, data, 'EKF-CA');

% Print results
printFilterResults(stats);

% Plot results
% plotFilterResults(data, estimates, 'Extended Kalman Filter (CA Model)');


end