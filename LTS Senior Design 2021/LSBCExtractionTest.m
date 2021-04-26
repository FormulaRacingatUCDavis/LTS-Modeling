clc; clear; close all;

%% Test Script

PE = load('TestPE.mat');

%% Performance Envelope Interpolation
% PE.State(SteerIdx,BodySlipIdx,SpeedIdx,LongAccIdx)

%PE.Interp.LatAcc = interpn( PE.BodySlip, PE.Steer, PE.Speed, PE.LongAcc, PE.LatAcc );
%PE.Interp.YawAcc = interpn( PE.BodySlip, PE.Steer, PE.Speed, PE.LongAcc, PE.YawAcc );

%% LAS Shelling
% LAS.State(:,2*(SteerIdx+BodySlipIdx-2),SpeedIdx,LongAccIdx) % Dimensionality
LAS.ShellIdx = [[(1:size(PE.BodySlip, 1))                            ; ones(1, size(PE.BodySlip, 1))                       ], ...
                [size(PE.BodySlip,1).*ones(1, size(PE.BodySlip, 2)-1); (2:size(PE.BodySlip, 2))                            ], ...
                [(size(PE.BodySlip, 1)-1:-1:1)                       ; size(PE.BodySlip,2).*ones(1, size(PE.BodySlip, 1)-1)], ...
                [ones(1, size(PE.BodySlip, 2)-2)                     ;(size(PE.BodySlip, 2)-1:-1:2)                        ]] ;         

      
LAS.ShellIdx = repmat( 
LAS.BodySlip = PE.BodySlip( sub2ind( size(PE.BodySlip), LAS.ShellIdx(1,:), LAS.ShellIdx(2,:), ones(1,length(LAS.ShellIdx)), ones(1,length(LAS.ShellIdx)) ) ); 
            
%% LSBS Extraction (Indexing LongAcc == 0)
% 
LSBS.Speed = LAS.Speed( PE.LongAcc == 0 );
LSBS.LatAcc = LAS.LatAcc( PE.LongAcc == 0 );
LSBS.YawAcc = LAS.YawAcc( PE.LongAcc == 0 );

scatter3( LSBS.Speed, LSBS.LatAcc, LSBS.YawAcc, 'k.' );

%% LSBC Extraction (Indexing Speed = BaseSpeed from LSBS)

%% GGV-V Extraction (Slicing where YawAcc = 0)


