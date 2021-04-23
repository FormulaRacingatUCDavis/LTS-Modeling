clc; clear; close all;

%% Test Script

LAS = load('TestLAS.mat');

%% LAS Volumetric Interpolation
%LatAcc = griddedInterpolant( LAS.BodySlip, LAS.Steer, LAS.Speed, LAS.LongAcc, LAS.LatAcc );
%YawAcc = griddedInterpolant( LAS.BodySlip, LAS.Steer, LAS.Speed, LAS.LongAcc, LAS.YawAcc );

%% LAS 4D Shelling
%LAS.ShellIdx = 

%% LSBC Extraction (Indexing LongAcc == 0)
LSBC.Speed = LAS.Speed( LAS.LongAcc == 0 );
LSBC.LatAcc = LAS.LatAcc( LAS.LongAcc == 0 );
LSBC.YawAcc = LAS.YawAcc( LAS.LongAcc == 0 );

scatter3( LSBC.Speed, LSBC.LatAcc, LSBC.YawAcc, 'k.' );

