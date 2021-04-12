clc; clear; close all;

%% GriddedTestMMM - Creates 4d Gridded Test MMM
% Generates a test MMM from gridded input data.

%% Gridded Inputs
Beta  = -8:4:8;
Delta = -120:120:120;
ax    = (-2:2) .* 9.81;
xDot  = 5:5:50;

[Beta, Delta, ax, xDot] = ndgrid( Beta, Delta, ax, xDot );

%% MMM Control Points
LSBC_ay_lim = 1.9 .* 9.81;
LSBC_psi_lim = -0.15;

LSBC_ay_pk  = 0.05 .* 9.81;
LSBC_psi_pk = 20;

%% Gridded Outputs;
ay      = zeros( size( Beta ) );
psiddot = zeros( size( Beta ) );

%% LSBC Data
ay(:,:,ax(1,1,:,1)==0,1) = ...
    [ 0.90, 0.90, 0.90; ...
     -0.05, 0.67, 0.85; ...
     -0.60, 0.00, 0.60; ...
     -0.85,-0.67, 0.05; ...
     -0.90,-0.90,-0.90] .* 9.81*(1.4/0.9);
 
psiddot(:,:,ax(1,1,:,1)==0,1) = ...
    [-0.09,-0.09,-0.09; ...
     -0.33,-0.10,-0.04; ...
     -0.12, 0.00, 0.12; ...
      0.04, 0.10, 0.33; ...
      0.09, 0.09, 0.09] .* 1.525*9.81*265/130*(2);
  
mesh( ay(:,:,ax(1,1,:,1)==0,1), psiddot(:,:,ax(1,1,:,1)==0,1), zeros(5,3) )

%% Extrapolating Data