dataPath = 'data/monte_carlo_sample.csv';
data = readtable(dataPath);

figure;
builtin('plot', data.COG, 'r.', 'MarkerSize', 10, 'DisplayName', 'COG Measurements');
hold on;
builtin('plot', data.cog_true, 'k-', 'LineWidth', 2, 'DisplayName', 'COG Ground Truth');
xlabel('Sample number');
ylabel('Course (degrees)');
title('Course Over Ground');
legend('Location', 'best');
grid on;