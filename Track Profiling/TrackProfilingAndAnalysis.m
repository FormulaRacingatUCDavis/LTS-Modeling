clc; clear; close all;

%% Data or Image Selection
File = uigetfile( {'*'}, 'Select Track Layout Image or Previously Generated Matlab Data' );

if ~isempty( regexp( File, '.mat*', 'Once' ) )
    load( File );
else
    Image.File = File;
    Image.Raw = imread( Image.File );    

    %% Image Scaling
    Scale.Length = input("Enter Scaling Length (m): ");

    Scale.Figure = figure;
    imshow( Image.Raw )
    title( 'Draw the Scaling Line' )

    Scale.Bar = drawline;

    Scale.Mask = createMask( Scale.Bar );
    Scale.Pixels = numel( find( Scale.Mask ) );
    Scale.Ratio = Scale.Length ./ Scale.Pixels;

    close( Scale.Figure );

    %% Control Point Identification
    Points.Figure = figure;
    imshow( Image.Raw )
    title( 'Select Points Along the Track' )

    [Points.Pixels(:,1), Points.Pixels(:,2)] = getpts;

    if regexp( Image.File, '*Endurance' )
       Points.Pixels(end+1,:) = Points.Pixels(1,:); % Close Cicuit Loop if Endurance Image
    end

    Points.Length = Points.Pixels .* Scale.Ratio; 

    Points.Length = Points.Length - Points.Length(1,:);

    close( Points.Figure );
end

clear File

%% Fit Cardinal Spline & Interpolation
Spline.N = 1000;
Spline.Tension=0;

Spline.Coordinate = [];

for k = 1 : length( Points.Length ) - 3
    XiYi = crdatnplusoneval( [Points.Length(k  ,1), Points.Length(k  ,2)], ...
                             [Points.Length(k+1,1), Points.Length(k+1,2)], ...
                             [Points.Length(k+2,1), Points.Length(k+2,2)], ...
                             [Points.Length(k+3,1), Points.Length(k+3,2)], ...
                             Spline.Tension , Spline.N );
                         
    [Spline.Coordinate] = [Spline.Coordinate; XiYi'];
end

clear k XiYi

%% Calculate Arc Length & Curvature
dx  = gradient( Spline.Coordinate(:,1) );
ddx = gradient( dx );
dy  = gradient( Spline.Coordinate(:,2) );
ddy = gradient( dy );

% Arc Length
Spline.Length(1) = 0;
for i = 2 : length( Spline.Coordinate )
   Spline.Length(i) = Spline.Length(i-1) + ...
       sqrt( (Spline.Coordinate(i,1) - Spline.Coordinate(i-1,1)).^2 + ...
             (Spline.Coordinate(i,1) - Spline.Coordinate(i-1,1)).^2 );
end

% Curvature
num   = dx .* ddy - ddx .* dy;
denom = (dx .* dx + dy .* dy).^(3/2);

Spline.Kappa = num ./ denom;
Spline.Kappa( -1/75 < Spline.Kappa & Spline.Kappa < 1/75 ) = 0;

Spline.Kappa = sgolayfilt( Spline.Kappa, 2, ...
    2 .* ceil( length( Spline.Length ) ./ Spline.Length(end) ) + 1 ) ;

clear i dx ddx dy ddy num denom

%% Plotting
Figure = figure;
sgtitle( [Image.File(1:end-4), ' Analysis'] )
subplot(2,2,1)
hold on;

imshow( Image.Raw )
plot( Points.Pixels(:,1), Points.Pixels(:,2), 'rx')

title( 'Original Image and Control Points' )

subplot(2,2,2)
hold on; axis equal;

plot( Points.Length(:,1), -Points.Length(:,2), 'rx')
plot( Spline.Coordinate(:,1), -Spline.Coordinate(:,2), 'b' )

title( 'Scaled & Interpolated Line' )
xlabel( 'X-Coordinate [m]' )
ylabel( 'Y-Coordinate [m]' )

subplot(2,2,3)

plot( Spline.Length, Spline.Kappa )

title( 'Track Curvature' )
xlabel( 'Track Distance [m]' )
ylabel( 'Track Curvature [1/m]' )

subplot(2,2,4)
hold on;

Histogram = histogram( abs(1./Spline.Kappa), 0:5:75, 'Normalization', 'Probability' );

title( ['Distribution of Turning Radius: ', ...
    num2str( 100 * (1 - sum( Histogram.Values ) ), 4 ), '% > 75 [m]' ] )
xlabel( 'Turning Radius [m]' )
ylabel( 'Probability [ ]' )

%% Save Results
save( [fileparts( which( 'TrackProfilingAndAnalysis.m' ) ), '\Analysis Results\' ...
       strrep( Image.File(1:end-4), ' ', '_' ) ] );

%% Local Functions
function [MatNbyNPlusOne]=crdatnplusoneval(P0,P1,P2,P3,T,n)
% Evaluate cubic Cardinal spline at n+1 values for given four points
% and tesion. Uniform parameterization is used.

% INPUT
% P0,P1,P2,P3:  Given four points in N-dimional space such that
%     P1 & P2 are endpoints of spline.
%     P0 & P3 are used to calculate the slope of the endpoints
%     (i.e slope of P1 & P2).
%     For example: for 2-dimensional data P0=[x, y],
%     For example: for 3-dimensional data P0=[x, y, z]
%     similar analogy for P1, P2, AND P3
%
% T: Tension (T=0 for Catmull-Rom type)
% n: number of intervals (spline is evaluted at n+1 values).
%
% OUTPUT
% MatNbyNPlusOne: Evaluated (interpolated) values of cardinal spline at
%     parameter value u. Values are in N-dimensional space.

    MatNbyNPlusOne=[];

    % u vareis b/w 0 and 1.
    % at u=0 cardinal spline reduces to P1.
    % at u=1 cardinal spline reduces to P2.

    u=0;
    MatNbyNPlusOne(:,1)=[evalcrdnd(P0,P1,P2,P3,T,u)]'; % MatNbyNPlusOne(:,1)=length(P0)
    du=1/n;
    
    for k=1:n
        u=k*du;
        MatNbyNPlusOne(:,k+1)=[evalcrdnd(P0,P1,P2,P3,T,u)]';
    end
end

function [Pu] = evalcrdnd(P0,P1,P2,P3,T,u)
% Evaluate (interpolate) N-dimensional cubic Cardinal Spline
% at given value of parameter u (a single value)
%
% INPUT
% P0,P1,P2,P3:  Given four points in N-dimional space such that
%     P1 & P2 are endpoints of spline.
%     P0 & P3 are used to calculate the slope of the endpoints
%     (i.e slope of P1 & P2).
%     For example: for 2-dimensional data P0=[x, y],
%     For example: for 3-dimensional data P0=[x, y, z]
%     similar analogy for P1, P2, AND P3
%
% T: Tension (T=0 for Catmull-Rom type)
% u: parameter value, spline is evaluated at u
%
% OUTPUT
% Pu: Evaluated (interpolated) values of cardinal spline at
%     parameter value u. Pu values are in N-dimensional space.

    Pu=[];

    s= (1-T)./2;
    % MC is cardinal matrix
    MC=[-s     2-s   s-2        s;
        2.*s   s-3   3-(2.*s)   -s;
        -s     0     s          0;
        0      1     0          0];

    for i=1:length(P0)
        G(:,i)=[P0(i);   P1(i);   P2(i);   P3(i)];
    end

    U=[u.^3    u.^2    u    1];

    for i=1:length(P0)
        Pu(i)=U*MC*G(:,i);
    end
end