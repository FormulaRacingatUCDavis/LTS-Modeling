function Track = CurvatureTrace( Spline, Image )
%% Curvature Trace
% Script that generates a curvature trace and other track representation
% parameters to be used in laptime simulation
%
%% Inputs
%
% Spline: Structure representing the optimal path cubic spline. Structure
% components are as generated in TrackProfilingAndAnalysis.m
%
% Image: Structure containing information about the track image analyzed
% for laptime simulation. Structure components are as generated in
% TrackProflingAndAnalysis.m
%
%% Outputs
%
% Track: Structure containing the following
%
% Track.Distance (n X 1 numeric): Total arc length travelled along the 
% spline from index 1 to the current index
%
% Track.Curvature (n X 1 numeric): Curvature (inverse of turn radius) of 
% the spline at the current Distance
%
% Track.Coordinates (n X 2 numeric): The X,Y coordinates of the position
% along the spline at the current index
% 
% Track.Label (char): The label of the track image analyzed
% 
% Track.Image (j X m X 3): The r,g,b track image analyzed
% 
% Track.Event (char): String equal to 'Autocross' if event is autocross,
% 'Endurance' if event is endurance
% 
% Track.Heading (n X 1 numeric): 

resolution = 5;
t = resolution;
s = 1;
i = 1;

Track.Label = Image.File(1:end-4);
Track.Image = Image.Raw;

k = 0;
for j=1:(size(Track.Label,2) - 4)
    if strcmp(Track.Label(j:j+4), 'AutoX')
        k = k + 1;
        Track.Event = 'Autocross';
    end 
end
for j=1:(size(Track.Label,2) - 8)
    if strcmp(Track.Label(j:j+8), 'Endurance')
        k = k + 1;
        Track.Event = 'Endurance';
    end
end

if k ~= 1
    event = input('Enter 1 if track is Autocross, 2 if Endurance: ');
    switch event
        case 1
            Track.Event = 'Autocross';
        case 2
            Track.Event = 'Endurance';
    end
end


dy = Spline.Coeff.b(s,2); % dy/du
dx = Spline.Coeff.b(s,1); % dx/du
cy = 2 * Spline.Coeff.c(s, 2); % d^2y/du^2
cx = 2 * Spline.Coeff.c(s, 1); % d^2x/du^2
Track.Curvature = (dx*cy - dy*cx)/(dx^2+dy^2)^(3/2); % Curvature
Track.Distance = 0;
Track.Coordinates = [0,0];
while t <= Spline.Distance(end)
    while t > Spline.Distance(s*1000)
        s=s+1;
    end
    while t > Spline.Distance(i)
        i=i+1;
    end
    u = (t - Spline.Distance((s-1)*1000 +1))/...
        (Spline.Distance(s*1000) - Spline.Distance((s-1)*1000 +1));
    
    dy = Spline.Coeff.b(s,2) + 2 * Spline.Coeff.c(s,2) * u + 3 * Spline.Coeff.d(s,2) * u^2;
    dx = Spline.Coeff.b(s,1) + 2 * Spline.Coeff.c(s,1) * u + 3 * Spline.Coeff.d(s,1) * u^2;
    cy = 2 * Spline.Coeff.c(s, 2) + 6 * Spline.Coeff.d(s,2) * u;
    cx = 2 * Spline.Coeff.c(s, 1) + 6 * Spline.Coeff.d(s,1) * u;
    
    Track.Curvature = [Track.Curvature; (dx*cy - dy*cx)/(dx^2+dy^2)^(3/2)];
    Track.Distance = [Track.Distance; t];
    Track.Coordinates = [Track.Coordinates; Spline.Length(i,:)];
    t = t + resolution;
end
Track.Heading = atan2d( gradient(Track.Coordinates(:,2)), gradient(Track.Coordinates(:,1)) );
end