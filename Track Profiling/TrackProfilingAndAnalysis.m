clc; clear; close all;

%% Track Profiling & Analysis
% Interactive script that generates a Catmull-Rom spline from user selected
% control points and then saves the spline and analysis data into a track
% object.
%
% Blake Christierson - bechristierson@ucdavis.edu
%
% v2.0 - (12/25/2020) Uses RoI objects for better interactivity & more
% stable curvature calculation

cd( fileparts( which( 'TrackProfilingAndAnalysis.m' ) ) )
%{
%% Data Selection
File = uigetfile( {'*'}, 'Select Track Layout Image or Previously Generated Matlab Data' );

if ~isempty( regexp( File, '.mat*', 'Once' ) )
    load( File );
else
    Image.File = File;
    Image.Raw = imread( Image.File );    

    %% Image Scaling
    Scale.Figure = figure;
    imshow( Image.Raw )
    title( 'Draw the Scaling Line' )

    Scale.Bar = drawline;
    Scale.Mask = createMask( Scale.Bar );
    Scale.Pixels = numel( find( Scale.Mask ) );

    close( Scale.Figure );
    
    Scale.Length = input("Enter Scaling Length (m): ");
    Scale.Ratio = Scale.Length ./ Scale.Pixels;
    
    %% Track Creation    
    [Points.Pixels, Spline] = CreateTrack( Image );

    Points.Length = Points.Pixels .* Scale.Ratio; 
    Points.Length = Points.Length - Points.Length(1,:);
    
    Spline.Length = Spline.Pixels .* Scale.Ratio; 
    Spline.Length = Spline.Length - Spline.Length(1,:);
end

clear File
%}

load('2015_18LincolnEnduranceTrack.mat')
Spline = SplineGeneration( Points.Pixels, 1000 );
Spline.Length = Spline.Pixels .* Scale.Ratio; 
Spline.Length = Spline.Length - Spline.Length(1,:);
    
%% Calculate Arc Length & Curvature
% Arc Length
Spline.Distance(1) = 0;
for i = 2 : length( Spline.Length )
   Spline.Distance(i) = Spline.Distance(i-1) + ...
       sqrt( (Spline.Length(i,1) - Spline.Length(i-1,1)).^2 + ...
             (Spline.Length(i,1) - Spline.Length(i-1,1)).^2 );
end

% Curvature
dx  = gradient( Spline.Length(:,1) );
ddx = gradient( dx );
dy  = gradient( Spline.Length(:,2) );
ddy = gradient( dy );

num   = dx .* ddy - ddx .* dy;
denom = (dx .* dx + dy .* dy).^(3/2);

Spline.Kappa = num ./ denom;
Spline.Kappa( -1/250 < Spline.Kappa & Spline.Kappa < 1/250 ) = .001;

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
function [Points, Spline] = CreateTrack( Image )
    Figure = figure( 'KeyPressFcn', @AddControlPoint );
    imshow( Image.Raw ); hold on;
    title( 'Select Points Along the Track' )
    
    Points = [0,0];
    PointRoI = images.roi.Point( 'Position', Points(end,:), 'Visible', 'off' );
    
    waitfor( Figure );
    
    Points = Points(2:end,:);
    if regexp( Image.File, '*Endurance' )
        Points(end+1,:) = Points(1,:);
    end
    
    Spline = SplineGeneration( Points, 5000 );
    
    function AddControlPoint( ~, Event )
        if strcmp( Event.Key, 'space' )
            Axes = gca;
            Cursor = get( Axes, 'CurrentPoint' );
            Cursor = Cursor(1,1:2);
            
            Points(end+1,1) = round( axes2pix(size(Image,2), [1 size(Image,2)], Cursor(1)) );
            Points(end,2) = round( axes2pix(size(Image,1), [1 size(Image,1)], Cursor(2)) );
            
            PointRoI(end+1) = images.roi.Point( Axes, 'Position', Points(end,:), ...
                'Label', num2str(length(PointRoI)+1), 'LabelVisible', 'off' );
            addlistener( PointRoI(end), 'ROIMoved', @ControlPointMoved );

            if size(Points,1) > 5
                Spline = SplineGeneration( Points(2:end,:), 50 );

                delete( findobj( 'Color', 'b', 'LineStyle', '--' ) );
                plot( Spline.Pixels(:,1), Spline.Pixels(:,2), 'b--' );
            end
        end
    end

    function ControlPointMoved( Source, ~ )
        Points( str2double(Source.Label), : ) = Source.Position;
        
        if size(Points,1) > 5
            Spline = SplineGeneration( Points(2:end,:), 50 );

            delete( findobj( 'Color', 'b', 'LineStyle', '--' ) );
            plot( Spline.Pixels(:,1), Spline.Pixels(:,2), 'b--' );
        end
    end
end

function Spline = SplineGeneration(Points, N )
    Spline.N = N;
    Spline.Tension = 0;
    
    Spline.Pixels = [];
    Spline.Slopes = [];
    Spline.Cons = [];
    
    Points = [Points(1,:); Points; Points(end,:)];
    
    for j = 1 : length( Points ) - 3
        [XiYi,Si,Ci] = crdatnplusoneval( [Points(j  ,1) , Points(j  ,2)], ...
                                 [Points(j+1,1) , Points(j+1,2)], ...
                                 [Points(j+2,1) , Points(j+2,2)], ...
                                 [Points(j+3,1) , Points(j+3,2)], ...
                                  Spline.Tension, Spline.N     );
                         
        [Spline.Pixels] = [Spline.Pixels; XiYi'];
        [Spline.Slopes] = [Spline.Slopes; Si'];
        [Spline.Cons] = [Spline.Cons; Ci'];
    end
    
    function [P,S,C]=crdatnplusoneval(P0,P1,P2,P3,T,n)
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

        P=[];

        % u vareis b/w 0 and 1.
        % at u=0 cardinal spline reduces to P1.
        % at u=1 cardinal spline reduces to P2.

        u=0;
        [P(:,1),S(:,1),C(:,1)]=evalcrdnd(P0,P1,P2,P3,T,u); % MatNbyNPlusOne(:,1)=length(P0)
        du=1/n;

        for k=1:n
            u=k*du;
            [P(:,k+1),S(:,k+1),C(:,k+1)]=evalcrdnd(P0,P1,P2,P3,T,u);
        end
    end

    function [Pu,Du,Cu] = evalcrdnd(P0,P1,P2,P3,T,u)
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
        Du=[];
        Cu=[];
        
        s= (1-T)./2;
        % MC is cardinal matrix
        MC=[-s     2-s   s-2        s;
            2.*s   s-3   3-(2.*s)   -s;
            -s     0     s          0;
            0      1     0          0];

        for i=1:length(P0)
            G(:,i)=[P0(i);   P1(i);   P2(i);   P3(i)]; %#ok<AGROW>
        end

        U=[u.^3    u.^2    u    1];
        D=[3.*u.^2 2.*u    1    0];
        C=[6.*u    2       0    0];
        
        for i=1:length(P0)
            Pu(i)=U*MC*G(:,i); %#ok<AGROW>
            Du(i)=D*MC*G(:,i); %#ok<AGROW>
            Cu(i)=C*MC*G(:,i); %#ok<AGROW>
        end
        
        Pu = Pu';
        Du = Du';
        Cu = Cu';
    end
end