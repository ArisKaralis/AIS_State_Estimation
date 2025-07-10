function [x, y] = convertLatLonToUTM(lat, lon)
% CONVERTLATLONTOUTM - Approximate UTM conversion using equirectangular projection
% This function converts LAT/LON to UTM-like x/y using local approximation.
%
% Inputs:
%   lat - Vector of latitudes (degrees)
%   lon - Vector of longitudes (degrees)
%
% Outputs:
%   x   - Easting (meters, relative)
%   y   - Northing (meters, relative)

% === Constants ===
meters_per_deg_lat = 110540;           % avg meters per degree latitude
meters_per_deg_lon = 111320;           % avg meters per degree longitude

% === Use central point as reference origin ===
lat0 = mean(lat);
lon0 = mean(lon);

% === Approximate conversion ===
dx = (lon - lon0) * meters_per_deg_lon .* cosd(lat0);
dy = (lat - lat0) * meters_per_deg_lat;

x = dx;
y = dy;
end
