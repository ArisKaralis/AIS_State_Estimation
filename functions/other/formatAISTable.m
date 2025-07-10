function formatted = formatAISTable(data, targetMMSI)
cog_rad = deg2rad(data.COG);
vx = data.SOG .* cos(cog_rad);
vy = data.SOG .* sin(cog_rad);
formatted = table();
formatted.MMSI = repmat(targetMMSI, height(data), 1);
formatted.BaseDateTime = data.BaseDateTime;
formatted.LAT = data.LAT;
formatted.LON = data.LON;
formatted.SOG = data.SOG;
formatted.COG = data.COG;
formatted.x = data.x;
formatted.y = data.y;
formatted.vx = vx;
formatted.vy = vy;
end
