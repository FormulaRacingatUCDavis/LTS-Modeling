function [Solution] = QTLTSSolver( Track, Vehicle )
%% QTLTSSolver - Quasi-Transient Laptime Simulation Solver
% Initializes velocity solution using top speed and maximum trim lateral
% acceleration. Then conducts forward/backward iterations without
% considering body slip until convergence. Body slip iterations are then
% done until convergence of final solution.
% 
% Inputs:
%   Track   - (struct) Track Constraints
%       .Distance    - (k,1 numeric) Track Arc Length
%       .Curvature   - (k,1 numeric) Discretized Curvature
%       .Coordinates - (k,2 numeric) Track Coordinates
%       .Event       - (char)        'Autocross' or 'Endurance'
%           
%   Vehicle - (struct) Vehicle Constraints
%       .PE   - (struct) Performance Envelope
%           .(field) - (n,m,p,q numeric) Arbritrary Dynamic State
%       .LAS  - (struct) Limit Acceleration Surface
%           .(field) - (2*(n+m-2),p,q numeric) Arbritrary Dynamic State 
%       .BSBS - (struct) Base Speed Boundary Surface
%           .(field) - (2*(n+m-2),q   numeric) Arbritrary Dynamic State 
%       .BSBC - (struct) Base Speed Boundary Curve
%           .(field) - (2*(n+m-2),1   numeric) Arbritrary Dynamic State 
% 
% Outputs:
%   Solution - (struct) State Solutions
%       .(field) - (k,1 numeric) Arbritrary Dynamic State 
%
% Notes:
%   n - Number of Unique Steer Angles
%   m - Number of Unique Body Slips
%   p - Number of Unique Speeds
%   q - Number of Unique Longitudinal Accelerations
%
%   k - Number of Track Samples
%
% Author(s):
% Blake Christierson (bechristierson@ucdavis.edu)
%
% Last Updated: 12-May-2021

%% Max Trim Lateral Initialization
