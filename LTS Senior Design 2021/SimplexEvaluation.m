function P = SimplexEvaluation( V, Chi )

%% Simplex Evaluation
% This scripts evaluates a n-d simplex based on its vertices and the
% parameterized length of the edges.
% 
% Inputs:
%   V   - (n,n+1 numeric) Simplex Vertices
%   Chi - (n,k   numeric) Parameterized Edge Lengths
%
% Outputs:
%   P   - (n,k   numeric) Evaluated Points
%
% Author(s):
% Blake Christierson (bechristierson@ucdavis.edu) 
% 
% Last Updated: 01-May-2021

%% Test Case
if nargin == 0
    warning( 'Executing SimplexEvaluation.m Test Case' );
    clc; clear; close all;
    
    %%% 2D Test
    V = [ 2, 4, 6; ...
         -3  2,-8];
     
    [X1, X2] = meshgrid( linspace(0,1,11), linspace(0,1,11) ); 
    Chi = [X1(:), X2(:)]';
    
    P = SimplexEvaluation( V, Chi );
    
    figure
    scatter( P(1,sum(Chi,1)<=1), P(2,sum(Chi,1)<=1), 'k.' );
    
    %%% 3D Test
    V = [ 2, 4, 6, 3; ...
         -3, 2,-8, 0; ...
          5,-2, 2, 9];
    
    [X1, X2, X3] = ndgrid( linspace(0,1,11), linspace(0,1,11), linspace(0,1,11) ); 
    Chi = [X1(:), X2(:), X3(:)]';
    
    P = SimplexEvaluation( V, Chi );
    
    figure
    scatter3( P(1,sum(Chi,1)<=1), P(2,sum(Chi,1)<=1), P(3,sum(Chi,1)<=1), 'k.' );
    
    return
end

%% Generating Edges
E = V(:,2:end) - V(:,1);

%% Evaluating Points
P = repmat( V(:,1), 1, size(Chi, 2) );

for k = 1:size(Chi,2)    
    for n = 1:size(Chi,1)
        P(:,k) = P(:,k) + Chi(n,k) .* E(:,n);
    end
end