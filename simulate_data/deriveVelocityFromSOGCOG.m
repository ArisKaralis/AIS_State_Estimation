function [vx, vy] = deriveVelocityFromSOGCOG(sog, cog_deg)
    cog_rad = deg2rad(cog_deg);
    vx = sog .* cos(cog_rad);
    vy = sog .* sin(cog_rad);
end
