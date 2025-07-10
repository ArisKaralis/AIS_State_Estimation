function [x_true, time] = simulateTruth(T, dt)
    time = 0:dt:T;
    n = length(time);
    x_true = zeros(4, n);

    % Initial state: [x; vx; y; vy]
    x = [0; 5; 0; 0];  % Eastward 5 m/s
    x_true(:,1) = x;

    for k = 2:n
        t = time(k);

        % Maneuver schedule
        if t > 300 && t <= 600
            x(4) = 1.5;  % Smooth right turn
        elseif t > 600 && t <= 900
            x(2) = 7; 
            x(4) = 1.5;  % Speed up
        elseif t > 900 && t <= 1200
            x(2) = 5;
            x(4) = -2;   % Turn left
        elseif t > 1200
            x(2) = 4; 
            x(4) = 0;    % Slow cruise
        end

        % CV Model
        F = [1 dt 0  0;
             0  1 0  0;
             0  0 1 dt;
             0  0 0  1];

        x = F * x;
        x_true(:,k) = x;
    end
end
