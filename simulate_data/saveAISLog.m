function saveAISLog(filename, time, z_meas, vx, vy, sog_meas, cog_meas, sog_true, cog_true)
    timestamp = datetime(2022,1,1,0,0,0) + seconds(time);
    T = table();
    T.timestamp = timestamp';
    T.x = z_meas(1,:)';
    T.y = z_meas(2,:)';
    T.vx = vx';
    T.vy = vy';
    T.sog = sog_meas';
    T.cog = cog_meas';
    T.sog_true = sog_true';
    T.cog_true = cog_true';
    writetable(T, filename);
end
