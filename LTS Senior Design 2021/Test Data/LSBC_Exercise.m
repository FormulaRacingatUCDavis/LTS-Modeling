
% Load LAS - note: 25 mins into recording
% xdim - LatAcc, ydim - YawAcc, zdim - LongAcc, 4Dim - Speed

%xdotMax = sqrt(ayMax/R(s)) :longAcc max = latAccMax / curvature

%% LSBC Extraction (Indexing LongAcc == 0)

LAS = load( 'TestLAS.mat');

LSBC.Speed = LAS.Speed( LAS.LongAcc == 0 );
LSBC.LatAcc = LAS.LatAcc( LAS.LongAcc == 0 );
LSBC.YawAcc = LAS.YawAcc( LAS.LongAcc == 0 );

%plot at speed=15m/s
scatter3( LSBC.Speed(LSBC.Speed==15), LSBC.LatAcc(LSBC.Speed==15), LSBC.YawAcc(LSBC.Speed==15), 'k.' );
LSBCBound = boundary(LSBC.LatAcc(LSBC.Speed==15), LSBC.YawAcc(LSBC.Speed==15),0);
% Setting shrink to 0 gives convex hull
hold on
plot3(15*ones(length(LSBC.LatAcc(LSBCBound))),LSBC.LatAcc(LSBCBound),LSBC.YawAcc(LSBCBound))

%loop gathering each LSBC surface and plotting then interpolating between
%indices??

