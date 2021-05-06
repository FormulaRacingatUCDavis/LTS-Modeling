function [LAS, BSBS, BSBC, GGV] = ConstraintExtraction( PE )
%% ConstraintExtraction - Derives Simplified Vehicle Constraints
% This function takes the performance envelope and derives lower fidelity
% vehicle performance limits.
% 
% Inputs:
%   PE - (struct) Performance Envelope
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

%% Input Parsing
if ~exist('PE', 'var')
    PE = load('TestPE.mat');
end

if ~exist('BaseSpeed', 'var')
    BaseSpeed = 15; % Base Speed [m/s]
end

%% Performance Envelope Characteristics
Field = fieldnames( PE );

n = size( PE.(Field{1}), 1 );
m = size( PE.(Field{1}), 2 );
p = size( PE.(Field{1}), 3 );
q = size( PE.(Field{1}), 4 );

%% Limit Acceleration Surface (LAS) Shelling
%%% Generating Shelling Index List
ShellIdx = [ [1:n           ; ones(1,n)      ], ...
             [n.*ones(1,m-1); 2:m            ], ...
             [n-1:-1:1      ; m.*ones(1,n-1) ], ...
             [ones(1,m-2)   ; m-1:-1:2       ] ];         
      
LAS.Sub = [];
for j3 = 1:p
    for j4 = 1:q
        LAS.Sub = [LAS.Sub, [ShellIdx; [j3; j4] .* ones(2, 2*(n+m-2))] ];
    end
end

LAS.Ind = sub2ind( [n,m,p,q], LAS.Sub(1,:), LAS.Sub(2,:), LAS.Sub(3,:), LAS.Sub(4,:) );

%%% Indexing into Performance Envelope
for f = 1:numel(Field)
    LAS.(Field{f}) = reshape( PE.(Field{f})(LAS.Ind), [], p, q );
end
            
%% Base Speed Boundary Surface (BSBS) Slicing
%%% BSBS is found by taking a slice in the speed dimension
for f = 1:numel(Field)
    BSBS.(Field{f}) = reshape( LAS.(Field{f})(LAS.Speed==BaseSpeed), [], q );
end

%% Base Speed Boundary Surface (BSBC) Slicing
%%% BSBC is found by taking a slice in the longitudinal acceleration dimension
for f = 1:numel(Field)
    BSBC.(Field{f}) = BSBS.(Field{f})(BSBS.LongAcc==0);
end

%% GGV
%%% GGV is generated by solving for max trim lateral acceleration of the LAS
GGV = [];
