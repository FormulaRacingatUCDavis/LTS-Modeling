clc; clear; close all;

%% Track Profiling & Analysis
% Interactive script that generates a Catmull-Rom spline from user selected
% control points and then saves the spline and analysis data into a track
% object.
%
% Blake Christierson - bechristierson@ucdavis.edu
%
% v1.0 - (04/15/2020) Initial Version
% v2.0 - (12/20/2020) RoI Objects for Better Interactivity
% v2.1 - (12/22/2020) C2 Interpolation Splines for Continuous Curvature

cd( fileparts( which( 'TrackProfilingAndAnalysis.m' ) ) )

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
    title( 'Draw the Scaling Line (Click & Drag)' )

    Scale.Bar = drawline;
    Scale.Mask = createMask( Scale.Bar );
    Scale.Pixels = numel( find( Scale.Mask ) );

    close( Scale.Figure );
    
    Scale.Length = input("Enter Scaling Length (m): ");
    Scale.Ratio = Scale.Length ./ Scale.Pixels;
    
    %% Track Creation    
    [Points, Spline] = CreateTrack( Image, Scale );
end

clear File

%% Plotting
Figure = figure;
sgtitle( [strrep(Image.File(1:end-4),'_','-'), ' Analysis'] )
subplot(1,2,1)
hold on;

imshow( Image.Raw )
plot( Points.Pixels(:,1), Points.Pixels(:,2), 'rx')
plot( Spline.Pixels(:,1), Spline.Pixels(:,2), 'b' )

title( 'Racing Line' )

subplot(1,2,2)
hold on;

Histogram = histogram( abs(Spline.Radius), 0:5:100, 'Normalization', 'Probability' );

title( ['Distribution of Turning Radius: ', ...
    num2str( 100 * (1 - sum( Histogram.Values ) ), 4 ), '% > 100 [m]' ] )
xlabel( 'Turning Radius [m]' )
ylabel( 'Probability [ ]' )

%% Save Results
save( [fileparts( which( 'TrackProfilingAndAnalysis.m' ) ), '\Analyses\' ...
    strrep( Image.File(1:end-4), ' ', '_' ) ] );
   
%% Local Functions
function [Points, Spline] = CreateTrack( Image, Scale )
    Figure = figure( 'WindowButtonDownFcn', @AddControlPoint );
    imshow( Image.Raw ); hold on;
    title( {'Select Points Along the Track', ...
            '(Add via Right or Middle Click - Existing Points May be Dragged)'} )
    
    Points.Pixels = [0,0];
    PointRoI = images.roi.Point( 'Position', Points.Pixels(end,:), 'Visible', 'off' );
    
    waitfor( Figure );
    
    Points.Pixels = Points.Pixels(2:end,:);
    Points.Length = Points.Pixels .* Scale.Ratio; 
    Points.Length = Points.Length - Points.Length(1,:);
    
    function AddControlPoint( Source, ~ )
        if ~strcmp( Source.SelectionType, 'normal' )
            Axes = gca;
            Cursor = get( Axes, 'CurrentPoint' );
            Cursor = Cursor(1,1:2);
            
            Points.Pixels(end+1,1) = round( axes2pix(size(Image,2), [1 size(Image,2)], Cursor(1)) );
            Points.Pixels(end,2) = round( axes2pix(size(Image,1), [1 size(Image,1)], Cursor(2)) );
            
            PointRoI(end+1) = images.roi.Point( Axes, 'Position', Points.Pixels(end,:), ...
                'Label', num2str(length(PointRoI)+1), 'LabelVisible', 'off' );
            addlistener( PointRoI(end), 'ROIMoved', @ControlPointMoved );

            if size(Points.Pixels,1) > 5
                Spline = SplineGeneration( Points.Pixels(2:end,:), Image, Scale );

                delete( findobj( 'Color', 'b', 'LineStyle', '--' ) );
                plot( Spline.Pixels(:,1), Spline.Pixels(:,2), 'b--' );
            end
        end
    end

    function ControlPointMoved( Source, ~ )
        Points.Pixels( str2double(Source.Label), : ) = Source.Position;
        
        if size(Points.Pixels,1) > 5
            Spline = SplineGeneration( Points.Pixels(2:end,:), Image, Scale );

            delete( findobj( 'Color', 'b', 'LineStyle', '--' ) );
            plot( Spline.Pixels(:,1), Spline.Pixels(:,2), 'b--' );
        end
    end
end

function Spline = SplineGeneration( Points, Image, Scale )
    % Compute Control Derivatives
    D = zeros( size(Points) ); 
    for j = 1 : size(Points,2)
        A = diag( 4*ones( size(Points,1)  ,1 )   ) + ...
            diag(   ones( size(Points,1)-1,1 ),-1) + ...
            diag(   ones( size(Points,1)-1,1 ), 1);
        
        if regexp( Image.File, '*Endurance' )
            A(1  ,1) = 4;   A(1  ,end) = 1;
            A(end,1) = 1;   A(end,end) = 4;
        else
            A(1  ,1  ) = 2;
            A(end,end) = 2;
        end
        
        b          = zeros( size(Points,1),1 );
        b(1)       = 3*( Points(2    ,j) - Points(1      ,j) );
        b(2:end-1) = 3*( Points(3:end,j) - Points(1:end-2,j) );
        b(end)     = 3*( Points(end  ,j) - Points(end-1  ,j) );
        
        D(:,j) = A\b;
    end
    
    Spline.Coeff.a = Points(1:end-1,:);
    Spline.Coeff.b = D(1:end-1,:);
    Spline.Coeff.c = 3*(Points(2:end  ,:) - Points(1:end-1,:)) - 2*D(1:end-1,:) - D(2:end,:);
    Spline.Coeff.d = 2*(Points(1:end-1,:) - Points(2:end  ,:)) +   D(1:end-1,:) + D(2:end,:);
    
    u = linspace(0,1,1000)';
    Spline.Pixels = [];
    d             = [];
    c             = [];
    
    for i = 1 : size(Points,1)-1
        pi = Spline.Coeff.a(i,:)       + Spline.Coeff.b(i,:).*u   + ...
             Spline.Coeff.c(i,:).*u.^2 + Spline.Coeff.d(i,:).*u.^3;  
        
        di =    Spline.Coeff.b(i,:) + 2.*Spline.Coeff.c(i,:) .* u + 3.*Spline.Coeff.d(i,:) .* u.^2;
        ci = 2.*Spline.Coeff.c(i,:) + 6.*Spline.Coeff.d(i,:) .* u;
        
        Spline.Pixels = [Spline.Pixels; pi];
        d = [d; di];
        c = [c; ci];
    end
    
    Spline.Length = Spline.Pixels .* Scale.Ratio;
    Spline.Length = Spline.Length - Spline.Length(1,:);
    
    Spline.Distance = zeros( size(Spline.Length,1), 1 );
    for i = 2 : size(Spline.Length,1)
        Spline.Distance(i) = Spline.Distance(i-1) + norm( Spline.Length(i,:)-Spline.Length(i-1,:), 2 );
    end
    
    Spline.Radius = (d(:,1).^2 + d(:,2).^2).^(3/2) ./ (c(:,1).*d(:,2) - c(:,2).*d(:,1));
end
