function Solution = BSBCSolver( BSBC, Track )
%% BSBS Solver - Base Speed Boundary Curve Solver
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
Field = fieldnames( BSBC );

for i = 1:numel(Field)
    Solution.(Field{i}) = zeros( size(Track.Distance) );
end

%% Corners on Track
RadiusThreshold = 150;

CornerIdx = abs(Track.Curvature) > 1/RadiusThreshold;

%% Optimal Mesh Point Identification
Solution.MeshIdx  = ones( size(Track.Distance) );

ds = diff(Track.Distance([1 2]));
Curvature_p = gradient( Track.Curvature, ds );

Correction = 1;
while Correction > 0.01
    OldIdx = Solution.MeshIdx;
    
    Speed(:,:,1) = sqrt( BSBC.LatAcc' ./ (Track.Curvature(CornerIdx) .* ...
        (cosd(BSBC.BodySlip') + tand(BSBC.BodySlip').*sind(BSBC.BodySlip'))) );

    BodySlip_pp = gradient( gradient( Solution.BodySlip, ds ), ds );
    Speed(:,:,2) = sqrt( BSBC.YawAcc' ./ ...
        (Curvature_p(CornerIdx) - BodySlip_pp(CornerIdx) + ...
        Track.Curvature(CornerIdx).^2 .* tand(BSBC.BodySlip')) );
    
    Speed( imag(Speed)~= 0 ) = -1;
    [Solution.Speed(CornerIdx), Solution.MeshIdx(CornerIdx)] = ...
        min( max( Speed, [], 2 ), [], 3);
    
    for i = 1:numel(Field)
        if ~strcmp(Field{i}, Solution.Speed)
            Solution.(Field{i}) = BSBC.(Field{i})(Solution.MeshIdx);
        end
    end
    
    Correction = sum( Solution.MeshIdx ~= OldIdx ) ./ numel(Solution.MeshIdx)
end