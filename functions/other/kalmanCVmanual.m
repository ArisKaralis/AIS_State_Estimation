function [x_true, time] = simulateTruth(T, dt)
    time = 0:dt:T;
    n = length(time);
    x_true = zeros(4, n);

    % Initial state [x; vx; y; vy]
    x = [0; 5; 0; 0];  % eastward 5 m/s
    x_true(:,1) = x;

    for k = 2:n
        t = time(k);

        % === Maneuver Schedule ===
        if t > 300 && t <= 600
            % Begin smooth right turn
            x(4) = 1.5;  % Add vy (northward drift)
        elseif t > 600 && t <= 900
            % Increase speed
            x(2) = 7;   % vx
            x(4) = 1.5; % maintain mild north drift
        elseif t > 900 && t <= 1200
            % Turn left
            x(2) = 5;
            x(4) = -2;
        elseif t > 1200
            % Straight cruise
            x(2) = 4;
            x(4) = 0;
        end

        % CV motion model update
        F = [1 dt 0  0;
             0  1 0  0;
             0  0 1 dt;
             0  0 0  1];

        x = F * x;
        x_true(:,k) = x;
    end
end
