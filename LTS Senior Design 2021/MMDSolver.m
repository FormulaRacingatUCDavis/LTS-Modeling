function Solution = MMDSolver( MMD, Track )
%% MMD Solver - Milliken Moment Diagram Solver
% This function takes the performance envelope and derives lower fidelity
% vehicle performance limits.
% 
% Inputs:
%   PE   - (struct) Performance Envelope
%       .(field) - (n,m,p,q numeric) Arbritrary Dynamic State 
% 
% Outputs:
%   LAS  - (struct) Limit Acceleration Surface
%       .(field) - (2*(n+m-2),p,q numeric) Arbritrary Dynamic State 
%   BSBS - (struct) Base Speed Boundary Surface
%       .(field) - (2*(n+m-2),q   numeric) Arbritrary Dynamic State 
%   BSBC - (struct) Base Speed Boundary Curve
%       .(field) - (2*(n+m-2)     numeric) Arbritrary Dynamic State 
%
% Notes:
%   n - Number of Unique Steer Angles
%   m - Number of Unique Body Slips
%   p - Number of Unique Speeds
%   q - Number of Unique Longitudinal Accelerations
%
% Author(s):
% Blake Christierson (bechristierson@ucdavis.edu)
%
% Last Updated: 06-May-2021

%% BSBC Fields & Solution Initialization
Field = fieldnames( MMD );

for i = 1:numel(Field)
    Solution.(Field{i}) = zeros( size(Track.Distance) );
end

%% Corners on Track
RadiusThreshold = 250;

CornerIdx = abs(Track.Curvature) > 1/RadiusThreshold;

%% Optimal Mesh Point Identification
Solution.Steer    = zeros( size(Track.Distance) );
Solution.BodySlip = zeros( size(Track.Distance) );

ds = diff(Track.Distance([1 2]));
Curvature_p = gradient( Track.Curvature, ds );

Correction = 1;
while Correction > 0.01   
    BodySlip_pp = gradient( gradient( Solution.BodySlip, ds ), ds );
    
    Solution.LatAcc = interp2( MMD.BodySlip, MMD.Steer, MMD.LatAcc, Solution.BodySlip, Solution.Steer );
    Solution.YawAcc = interp2( MMD.BodySlip, MMD.Steer, MMD.YawAcc, Solution.BodySlip, Solution.Steer );
    
    Speed(:,:,1) = sqrt( Solution.LatAcc(CornerIdx) ./ (Track.Curvature(CornerIdx) .* ...
        (cosd(Solution.BodySlip(CornerIdx)) + tand(Solution.BodySlip(CornerIdx)).* ...
        sind(Solution.BodySlip(CornerIdx)))) );

    Speed(:,:,2) = sqrt( Solution.YawAcc(CornerIdx) ./ ...
        (Curvature_p(CornerIdx) - BodySlip_pp(CornerIdx) + ...
        Track.Curvature(CornerIdx).^2 .* tand(Solution.BodySlip(CornerIdx))) );
    
    Speed( imag(Speed)~= 0 ) = -1;
    [Solution.Speed(CornerIdx), Solution.MeshIdx(CornerIdx)] = ...
        max( min( Speed, [], 3 ), [], 2);
    
    for i = 1:numel(Field)
        if ~strcmp(Field{i}, Solution.Speed)
            Solution.(Field{i}) = MMD.(Field{i})(Solution.MeshIdx);
        end
    end
    
    Correction = sum( Solution.MeshIdx ~= OldIdx ) ./ numel(Solution.MeshIdx)
end