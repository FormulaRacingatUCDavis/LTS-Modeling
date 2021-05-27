function [Track, Solution] = QTSLaptimeSimulation( Vehicle, Track )
%% QTSLaptimeSimulation - Quasi-Transient Laptime Simulation Main Script

%% Test Case
if nargin == 0
    Vehicle.PE = load('TestPE.mat');
    Track = load('2015-18_Lincoln_AutoX.mat');
    Track = Track.Track2;
    Track.Event = 'AutoCross';
    Track.Heading = atan2d( gradient(Track.Coordinates(:,1)), gradient(Track.Coordinates(:,2)) );
    Track.Heading( Track.Heading < 0 ) = Track.Heading( Track.Heading < 0 ) + 360;
end

%% Vehicle Constraint Extraction
[Vehicle.LAS, Vehicle.BSBS, Vehicle.BSBC, Vehicle.MMD, Vehicle.GGV] = ConstraintExtraction( Vehicle.PE );

Vehicle.TrimLat = 1.6.*9.81;

%% Solution State Initialization 
ds = diff( Track.Distance([1 2]) );

Solution.sDot = min( max(Vehicle.PE.Speed, [], 'all'), ... % Max Top Speed
    sqrt( abs(Vehicle.TrimLat ./ Track.Curvature) ) );

Solution.sDDot = zeros( size(Track.Distance) );

Field = fieldnames(Vehicle.PE);
for f = 1:numel(Field)
    Solution.(Field{f}) = zeros( size(Track.Distance) );
end

drhods = gradient( Track.Curvature, ds );
dchids = gradient( deg2rad(Track.Heading), ds );

%% Determining Initial Track Index
if strcmpi( Track.Event, 'Autocross' )
    IdxVector = 1:length( Track.Distance );
    Solution.sDot(1) = 0.1;
elseif strcmpi( Track.Event, 'Endurance' )
    [~, ApexIdx] = findpeaks( -Solution.sDot );
    IdxVector = [ApexIdx(1):length( Track.Distance ), 1:ApexIdx(1)-1];
else
    warning( 'Event Not Specified Properly. Assuming Standing Start.' )
    IdxVector = 1:length( Track.Distance );
    Solution.sDot(1) = 0;
end

%% Stepping Algorithm
Correction = 1;
while Correction > 0.02 % Iterate Until Body Slip Correction-Based Convergence
    %%% Calculate Second Spatial Derivative of Body Slip
    d2betads2 = gradient( Track.Curvature, ds, 2 );
    
    for i = IdxVector % Loop Over Track Indices
        %%% Calculate Required Accelerations Given Current State
        Solution.LongAcc(i) = Solution.sDDot(i) .* sind( Solution.BodySlip(i) ) - ...
            Track.Curvature(i) .* Solution.sDot(i)^2 .* cosd( Solution.BodySlip(i) );
        
        Solution.LatAcc(i) = Track.Curvature(i) .* Solution.sDot(i)^2 .* ...
            cosd( Solution.BodySlip(i) ) + Solution.sDDot(i) .* sind( Solution.BodySlip(i) );

        Solution.YawAcc(i) = (drhods(i) - d2betads2(i)) .* Solution.sDot(i)^2 + ...
            Track.Curvature(i) .* Solution.sDDot(i);
        
        %%% Check if Required Performance State is Feasible
        if FeasibleVehicleState( Vehicle, Solution, i )
            %%% Accelerate and Step Forward
            Solution = MaximizeAcceleration( Track, Vehicle, Solution, i, '+' );
            
            Solution.sDot(i+1) = sqrt( Solution.sDot(i)^2 + 2*Solution.sDDot(i)*ds );
            Solution.BodySlip(i+1) = sqrt( Solution.BodySlip(i)^2 + 2*rad2deg(Solution.BodySlipDot(i))*ds );
        else
            %%% Compute Maximum Feasible Speed
            Solution = MaximizeSpeed( Track, Vehicle, Solution, i );
            
            error( 'Not Completed' );
        end
    end
end

%% Vehicle State Feasibility
function IsFeasible = FeasibleVehicleState( Vehicle, Solution, i )
    xDot = Solution.sDot(i) * cosd( Solution.BodySlip(i) ); % Longitudinal Velocity
    
    %%% Slice Performance Envelope in Longitudinal Velocity
    if xDot <= min( Vehicle.LAS.Speed, [], 'all' )
        LongAcc = squeeze( Vehicle.LAS.LongAcc(:,1,:) );
        LatAcc  = squeeze( Vehicle.LAS.LatAcc(:,1,:) );
        YawAcc  = squeeze( Vehicle.LAS.YawAcc(:,1,:) );
    elseif xDot >= max( Vehicle.LAS.Speed, [], 'all' )
        IsFeasible = false;
        return;
    else
        Idx = find( unique(Vehicle.LAS.Speed) < xDot, 1, 'last' );
        x = squeeze( Vehicle.LAS.Speed(1,Idx:Idx+1,1) );
        
        LongAcc = squeeze( interp1( x, permute( Vehicle.LAS.LongAcc(:,Idx:Idx+1,:), [2,1,3] ), xDot ) );
        LatAcc  = squeeze( interp1( x, permute( Vehicle.LAS.LatAcc(:,Idx:Idx+1,:) , [2,1,3] ), xDot ) );
        YawAcc  = squeeze( interp1( x, permute( Vehicle.LAS.YawAcc(:,Idx:Idx+1,:) , [2,1,3] ), xDot ) );
    end
    
    %%% Slice Performance Envelope in Longitudinal Acceleration
    if Solution.LongAcc(i) <= min( LongAcc, [], 'all' )
        IsFeasible = false;
        return;
    elseif Solution.LongAcc(i) >= max( LongAcc, [], 'all' )
        IsFeasible = false;
        return;
    else
        Idx = find( unique(LongAcc) < Solution.LongAcc(i), 1, 'last' );
        x = squeeze( LongAcc(1,Idx:Idx+1) );
        
        LatAcc  = squeeze( interp1( x, permute( LatAcc(:,Idx:Idx+1) , [2,1] ), Solution.LongAcc(i) ) );
        YawAcc  = squeeze( interp1( x, permute( YawAcc(:,Idx:Idx+1) , [2,1] ), Solution.LongAcc(i) ) );
    end
    
    %%% Check if Lateral and Yaw Acceleration are Feasible in MMD
    if inpolygon( Solution.LatAcc(i), Solution.YawAcc(i), LatAcc, YawAcc )
        IsFeasible = true;
    else
        IsFeasible = false;
    end
end

%% Maximizing Acceleration Given Speed
function Solution = MaximizeAcceleration( Track, Vehicle, Solution, i, Sign )
    %%% Variable Allocation
    sDot = Solution.sDot(i);
    rho = Track.Curvature(i);
    beta = Solution.BodySlip(i);
    
    xDot = sDot / cosd(beta);
    
    %%% Interpolate Speed Condition
    Speeds = squeeze( Vehicle.PE.Speed(1,1,:,1) );
    
    if xDot <= min( Vehicle.PE.Speed, [], 'all' )
        LongAcc = squeeze( Vehicle.PE.LongAcc(:,:,1,:) );
        LatAcc  = squeeze( Vehicle.PE.LatAcc(:,:,1,:) );
        YawAcc  = squeeze( Vehicle.PE.YawAcc(:,:,1,:) );
    else 
        LongAcc = squeeze( interp1( Speeds, permute( Vehicle.PE.LongAcc, [3,1,2,4] ), xDot ) );
        LatAcc  = squeeze( interp1( Speeds, permute( Vehicle.PE.LatAcc , [3,1,2,4] ), xDot ) );
        YawAcc  = squeeze( interp1( Speeds, permute( Vehicle.PE.YawAcc , [3,1,2,4] ), xDot ) );
    end
    
    %%% Interpolate Body Slip Condition
    BodySlips = squeeze( Vehicle.PE.BodySlip(1,:,1,1) );
        
    LongAcc = squeeze( interp1( BodySlips, permute( LongAcc, [2,1,3] ), beta ) );
    LatAcc  = squeeze( interp1( BodySlips, permute( LatAcc , [2,1,3] ), beta ) );
    YawAcc  = squeeze( interp1( BodySlips, permute( YawAcc , [2,1,3] ), beta ) );
    
    %%% Compute Mesh Point Accelerations
    MeshAcc = [ (LongAcc(:) + rho*sDot^2*sind(beta)) / cosd(beta), ...
                (LatAcc(:)  - rho*sDot^2*cosd(beta)) / sind(beta), ...
                (YawAcc(:)  - (drhods(i) - d2betads2(i))*sDot^2) / rho ];
            
    %%% Compute Optimal Mesh Points
    if strcmpi( Sign, '+' )
        [sDDot, jOpt] = maxk( min( MeshAcc, [], 2 ), 1 );
    else 
        [sDDot, jOpt] = mink( max( MeshAcc, [], 2 ), 1 );
    end
    
    figure
    plot3( LongAcc, LatAcc, YawAcc, 'k' )
    hold on
    plot3( LongAcc', LatAcc', YawAcc', 'k' )
    scatter3( LongAcc(jOpt), LatAcc(jOpt), YawAcc(jOpt), 'rx' )
    fplot3( @(sDDot) sDDot.*cosd(beta) - rho.*sDot.^2.*sind(beta), ...
            @(sDDot) rho.*sDot.^2.*cosd(beta) + sDDot.*sind(beta), ...
            @(sDDot) (drhods(i) - d2betads2(i)).*sDot.^2 + rho.*sDDot, [-15 15], 'b')
        
    %%% Allocate Solution Fields
    Solution.sDDot(i) = sDDot;
    
    for ff = 1:numel(Field)
        if ~(strcmp( Field{ff}, 'BodySlip' ) || strcmp( Field{ff}, 'Speed' ))  
            Solution.(Field{ff})(i) = Vehicle.LAS.(Field{ff})(jOpt); 
        end
    end
    
    %%% Compute Body Slip Derivative 
    Solution.BodySlipDot(i) = dchids(i) * Solution.sDot(i) - ... 
        trapz( Track.Distance(1:i), Solution.YawAcc(1:i) ./ Solution.sDot(i), 1 );
end

%% Maximizing Speed Given No Acceleration
function Solution = MaximizeSpeed( Track, Vehicle, Solution, i )
    %%% Variable Allocation
    rho = Track.Curvature(i);
    beta = Solution.BodySlip(i);
    
    %%% Compute Mesh Point Accelerations
    MeshSpe2 = [(Vehicle.LAS.Speed(:)   ./ cosd(beta)).^2, ...
                ...%-Vehicle.LAS.LongAcc(:) ./ rho*sind(beta), ...
                 Vehicle.LAS.LatAcc(:)  ./ rho*cosd(beta), ...
                 Vehicle.LAS.YawAcc(:)  ./ (drhods(i) - d2betads2(i))];
            
    %%% Compute Optimal Mesh Points
    [sDot2, jOpt] = maxk( min( MeshSpe2, [], 2 ), 1 );
    
    %%% Allocate Solution
    Solution.sDot(i) = sqrt(sDot2);
    
    for ff = 1:numel(Field)
        Solution.(Field{ff})(i) = Vehicle.LAS.(Field{ff})(jOpt); 
    end
end

end