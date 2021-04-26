clc; clear; close all;

set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

%% GriddedTestPE - Creates 4d Gridded Test Performance Envelope
% Generates a test PE from gridded test data.

%% PE Control Points
%%% Base Speed Data
Base.Speed     = 15;
Base.LatGrip   = 1.6;
Base.BrakeGrip = 1.8;
Base.Traction  = 1.2;
Base.Rotation  = 23;

%%% Top Speed Data
Top.Speed     = 50;
Top.LatGrip   = 1.9;
Top.BrakeGrip = 2.2;
Top.Traction  = 0;
Top.Rotation  = 20;

%%% Vehicle Parameters
Wheelbase  = 1.525; % Wheelbase [m]
Mass       = 265;   % Vehicle Mass [kg]
YawInertia = 130;   % Vehicle Yaw Inertia [kg-m^2]

%% Gridded Inputs
BodySlip = linspace(-8,0,3); 
BodySlip = [BodySlip, abs(BodySlip(end-1:-1:1))];

Steer = linspace(-120,0,2); 
Steer = [Steer, abs(Steer(end-1:-1:1))];

n = 10;
LongAcc = [linspace(-Base.BrakeGrip,0,n), linspace(0+Base.Traction/(n-1),Base.Traction,(n-1))];

Speed = 5 : 5 : Top.Speed;

[Steer, BodySlip, Speed, LongAcc] = ndgrid( Steer, BodySlip, Speed, LongAcc );

%% Base MMM Data
% MMM data taken from RCVD and scaled to be reflective of FSAE Car

%%% Inputting Raw RCVD Data
LatAcc = repmat( [ 0.90, 0.90, 0.90; ...
                  -0.05, 0.67, 0.85; ...
                  -0.60, 0.00, 0.60; ...
                  -0.85,-0.67, 0.05; ...
                  -0.90,-0.90,-0.90]', 1, 1, size(Speed,3), size(LongAcc,4) );
 
YawAcc = repmat( [-0.09,-0.09,-0.09; ...
                  -0.33,-0.10,-0.04; ...
                  -0.12, 0.00, 0.12; ...
                   0.04, 0.10, 0.33; ...
                   0.09, 0.09, 0.09]', 1, 1, size(Speed,3), size(LongAcc,4) );

LatAcc = LatAcc .* Base.LatGrip ./ max( abs(LatAcc), [], 'all' );
YawAcc = YawAcc .* Wheelbase .* Mass .* 9.81 ./ YawInertia;
    YawAcc = YawAcc .* Base.Rotation ./ max( abs(YawAcc), [], 'all' );

%% Speed Dependent Effects
%%% Scaling Coefficients
BrakeLimit.c2    = (Top.BrakeGrip - Base.BrakeGrip) ./ (Top.Speed^2 - Base.Speed^2);
BrakeLimit.c0    = (Top.BrakeGrip + Base.BrakeGrip - BrakeLimit.c2   .* (Top.Speed^2 + Base.Speed^2))/2;

TractionLimit.c2 = (Top.Traction  - Base.Traction ) ./ (Top.Speed^2 - Base.Speed^2);
TractionLimit.c0 = (Top.Traction  + Base.Traction  - TractionLimit.c2 .* (Top.Speed^2 + Base.Speed^2))/2;

LateralLimit.c2  = (Top.LatGrip   - Base.LatGrip  ) ./ (Top.Speed^2 - Base.Speed^2);
LateralLimit.c0  = (Top.LatGrip   + Base.LatGrip   - LateralLimit.c2  .* (Top.Speed^2 + Base.Speed^2))/2;

RotationLimit.c2 = (Top.Rotation  - Base.Rotation ) ./ (Top.Speed^2 - Base.Speed^2);
RotationLimit.c0 = (Top.Rotation  + Base.Rotation  - RotationLimit.c2 .* (Top.Speed^2 + Base.Speed^2))/2;

%%% Longitudinal Acceleration Scaling
LongAcc(LongAcc<0) = LongAcc(LongAcc<0) .* (BrakeLimit.c2    .* Speed(LongAcc<0).^2 + BrakeLimit.c0   ) ./ Base.BrakeGrip;
LongAcc(LongAcc>0) = LongAcc(LongAcc>0) .* (TractionLimit.c2 .* Speed(LongAcc>0).^2 + TractionLimit.c0) ./ Base.Traction;

%%% Lateral Acceleration Scaling
LatAcc = LatAcc .* (LateralLimit.c2 .* Speed.^2 + LateralLimit.c0) ./ Base.LatGrip;

%%% Yaw Acceleration Scaling
YawAcc = YawAcc .* (RotationLimit.c2 .* Speed.^2 + RotationLimit.c0) ./ Base.Rotation;

%% Friction Ellipse
Speeds = unique(Speed);
for i = 1:numel(Speeds)
    %%% Lateral Acceleration Scaling
    LatAcc(LongAcc<0 & Speed==Speeds(i)) = LatAcc(LongAcc<0 & Speed==Speeds(i)) .* ...
        sqrt( 1 - LongAcc(LongAcc<0 & Speed==Speeds(i)).^2 ./ max( abs(LongAcc(LongAcc<0 & Speed==Speeds(i))) ).^2);
    
    LatAcc(LongAcc>0 & Speed==Speeds(i)) = LatAcc(LongAcc>0 & Speed==Speeds(i)) .* ...
        sqrt( 1 - LongAcc(LongAcc>0 & Speed==Speeds(i)).^2 ./ max( abs(LongAcc(LongAcc>0 & Speed==Speeds(i))) ).^2);
    
    %%% Yaw Acceleration Scaling
    YawAcc(LongAcc<0 & Speed==Speeds(i)) = YawAcc(LongAcc<0 & Speed==Speeds(i)) .* ...
        sqrt( 1 - LongAcc(LongAcc<0 & Speed==Speeds(i)).^2 ./ max( abs(LongAcc(LongAcc<0 & Speed==Speeds(i))) ).^2);
    
    YawAcc(LongAcc>0 & Speed==Speeds(i)) = YawAcc(LongAcc>0 & Speed==Speeds(i)) .* ...
        sqrt( 1 - LongAcc(LongAcc>0 & Speed==Speeds(i)).^2 ./ max( abs(LongAcc(LongAcc>0 & Speed==Speeds(i))) ).^2);
end

%% Unit Conversion
LongAcc = 9.81 .* LongAcc;
LatAcc = 9.81 .* LatAcc;

%% Plotting
figure;
for i = 1:size(LongAcc,4)
    mesh( LongAcc(:,:,Speed(1,1,:,1)==Base.Speed,i) , ...
           LatAcc(:,:,Speed(1,1,:,1)==Base.Speed,i) , ...
           YawAcc(:,:,Speed(1,1,:,1)==Base.Speed,i) ); hold on;

    scatter3( LongAcc(Speed(:)==Base.Speed) , ...
               LatAcc(Speed(:)==Base.Speed) , ...
               YawAcc(Speed(:)==Base.Speed) , 'k.' );
end 

title( 'Base Speed PE' )
xlabel( 'Longtudinal Acceleration, $\ddot{x}$' )
ylabel( 'Lateral Acceleration, $\ddot{y}$' )
zlabel( 'Yaw Acceleration, $\ddot{\psi}$' )

%% Data Export
save( [fileparts( which( 'GriddedTestPE.m' ) ), '\TestPE'], ...
    'BodySlip', 'Steer', 'Speed', 'LongAcc', 'LatAcc', 'YawAcc' )