function dt = calculateTimeSteps(timestamps)
    % CALCULATETIMESTEPS - Calculate time steps from timestamps
    
    n = length(timestamps);
    dt = zeros(n, 1);
    
    for i = 2:n
        time_diff = timestamps(i) - timestamps(i-1);
        if isduration(time_diff)
            dt(i) = double(seconds(time_diff));
        else
            dt(i) = seconds(time_diff);
            if isduration(dt(i))
                dt(i) = double(dt(i));
            end
        end
    end
end