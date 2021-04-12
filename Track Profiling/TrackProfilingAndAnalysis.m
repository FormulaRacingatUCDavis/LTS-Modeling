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
% v2.2 - (12/23/2020) Corner Linear Interpolation Smoothing

addpath( genpath( fileparts( which( 'TrackProfilingAndAnalysis.m' ) ) ) )
cd( fileparts( which( 'TrackProfilingAndAnalysis.m' ) ) )
RadiusThresh = 250;
    
%% Data Selection
% Either Select:
%   - Previously generated MATLAB analysis (.mat in "Analyses" folder) 
%     to visualize existing results
% or 
%   - Raw track image (.png or .jpg in the "Track Map Images" folder)
%     to generate a new track analysis

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
    
    %% Spline Creation  
    [Points, Spline] = SelectSpline( Image, Scale, RadiusThresh );
    
    %% Corner Correction
    Spline = CornerSmoothing( Spline, RadiusThresh );
end

clear File

%% Plotting
Figure = figure( 'WindowState', 'Maximized' );
sgtitle( [strrep(Image.File(1:end-4),'_',' '), ' Analysis'] )
subplot(2,2,[1 3])
hold on;

imshow( Image.Raw )
plot( Points.Pixels(:,1), Points.Pixels(:,2), 'rx', 'MarkerSize', 6 )
scatter( Spline.Pixels(Spline.IsCorner,1), Spline.Pixels(Spline.IsCorner,2), 2, 'r' )
scatter( Spline.Pixels(~Spline.IsCorner,1), Spline.Pixels(~Spline.IsCorner,2), 2, 'k' )

title( 'Racing Line' )

subplot(2,2,4)
hold on;

Histogram = histogram( abs(Spline.Radius(:,2)), 0:2.5:RadiusThresh, ...
    'Normalization', 'Probability' );

title( ['Radius Distribution: ', ...
    num2str( 100 * (1 - sum( Histogram.Values ) ), 4 ), '\% $>$ ', num2str(RadiusThresh), ' [m]' ] )
xlabel( 'Turning Radius [m]' )
ylabel( 'Probability [ ]' )

subplot(2,2,2)
hold on;

scatter( Spline.Distance(~Spline.IsCorner), Spline.Radius(~Spline.IsCorner,1), 2, 'k.' );
scatter( Spline.Distance( Spline.IsCorner), Spline.Radius( Spline.IsCorner,1), 2, 'r.' ); 
scatter( Spline.Distance( Spline.IsCorner), Spline.Radius( Spline.IsCorner,2), 2, 'b.' ); 

title( 'Turning Radius Trace' )
xlabel( 'Track Distance [m]' )
ylabel( 'Turning Radius [m]' ); ylim([-300, 300])

legend( {'Raw Straights', 'Raw Corners', 'Linearized Corners'} )

%% Save Results
save( [fileparts( which( 'TrackProfilingAndAnalysis.m' ) ), '\Analyses\' ...
    strrep( Image.File(1:end-4), ' ', '_' ) ], ...
    'Histogram', 'Image', 'Points', 'Scale', 'Spline' );

saveas( Figure, [fileparts( which( 'TrackProfilingAndAnalysis.m' ) ), '\Analyses\' ...
    strrep( Image.File(1:end-4), ' ', '_' ) ], 'png' )

Figure.WindowState = 'normal';

%% Local Functions
function [Points, Spline] = SelectSpline( Image, Scale, RadiusThresh )
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
                Spline = SplineGeneration( Points.Pixels(2:end,:), Scale );

                delete( findobj( 'Type', 'scatter' ) );
                scatter( Spline.Pixels(abs(Spline.Radius)>RadiusThresh,1), Spline.Pixels(abs(Spline.Radius)>RadiusThresh,2), 2, 'k' );
                scatter( Spline.Pixels(abs(Spline.Radius)<RadiusThresh,1), Spline.Pixels(abs(Spline.Radius)<RadiusThresh,2), 2, 'r' );
            end
        end
    end

    function ControlPointMoved( Source, ~ )
        Points.Pixels( str2double(Source.Label), : ) = Source.Position;
        
        if size(Points.Pixels,1) > 5
            Spline = SplineGeneration( Points.Pixels(2:end,:), Scale );

            delete( findobj( 'Type', 'scatter' ) );
            scatter( Spline.Pixels(abs(Spline.Radius)>RadiusThresh,1), Spline.Pixels(abs(Spline.Radius)>RadiusThresh,2), 2, 'k' );
            scatter( Spline.Pixels(abs(Spline.Radius)<RadiusThresh,1), Spline.Pixels(abs(Spline.Radius)<RadiusThresh,2), 2, 'r' );
        end
    end

    function Spline = SplineGeneration( Points, Scale )
        % Compute Control Derivatives
        D = zeros( size(Points) ); 
        for j = 1 : size(Points,2)
            A = diag( 4*ones( size(Points,1)  ,1 )   ) + ...
                diag(   ones( size(Points,1)-1,1 ),-1) + ...
                diag(   ones( size(Points,1)-1,1 ), 1);

            A(1  ,1  ) = 2;
            A(end,end) = 2;

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
            d = [d; di]; %#ok<AGROW>
            c = [c; ci]; %#ok<AGROW>
        end

        Spline.Length = Spline.Pixels .* Scale.Ratio;
        Spline.Length = Spline.Length - Spline.Length(1,:);

        Spline.Distance = zeros( size(Spline.Length,1), 1 );
        for i = 2 : size(Spline.Length,1)
            Spline.Distance(i) = Spline.Distance(i-1) + norm( Spline.Length(i,:)-Spline.Length(i-1,:), 2 );
        end

        Spline.Radius = (d(:,1).^2 + d(:,2).^2).^(3/2) ./ (c(:,1).*d(:,2) - c(:,2).*d(:,1));
    end
end

function Spline = CornerSmoothing( Spline, RadiusThresh )
    Spline.IsCorner = abs(Spline.Radius) < RadiusThresh;
    
    StraightIdx = find(~Spline.IsCorner);
    Entry.Idx = diff(StraightIdx) > 1;
    Entry.Idx = StraightIdx(Entry.Idx)+1;
    
    CornerIdx = find(Spline.IsCorner);
    Exit.Idx = find( diff(CornerIdx) > 1 );
    Exit.Idx = [Exit.Idx; length(CornerIdx)];
    Exit.Idx = CornerIdx(Exit.Idx);
    
    Entry.Distance = Spline.Distance(Entry.Idx);
    Exit.Distance  = Spline.Distance(Exit.Idx );
    
    Entry.Radius = Spline.Radius(Entry.Idx);
    Exit.Radius  = Spline.Radius(Exit.Idx );
    
    Entry.Slope = (Spline.Radius(Entry.Idx+1)   - Spline.Radius(Entry.Idx-1)) ./ ...
                  (Spline.Distance(Entry.Idx+1) - Spline.Distance(Entry.Idx-1));
    Exit.Slope  = (Spline.Radius(Exit.Idx+1)    - Spline.Radius(Exit.Idx-1))  ./ ...
                  (Spline.Distance(Exit.Idx+1)  - Spline.Distance(Exit.Idx-1) ); 
    
    Spline.Radius(:,2) = Spline.Radius(:,1);
    for i = 1 : length( Entry.Idx )
        xData = Spline.Distance( Entry.Idx(i) : Exit.Idx(i) );
        yData = abs(Spline.Radius( Entry.Idx(i) : Exit.Idx(i) ));
        
        IsMin = find( islocalmin( yData, 'MinProminence', 2 ) );
        if ~isscalar( IsMin )
            xMin = xData( IsMin([1 end]) );
            yMin = yData( IsMin([1 end]) );
            
            yData(IsMin(1) : IsMin(end)) = interp1( xMin, yMin, xData(IsMin(1) : IsMin(end)) );
            
            Spline.Radius(Entry.Idx(i) : Exit.Idx(i),2) = ...
                sign( Spline.Radius(Entry.Idx(i)) ) .* yData; 
        end
    end
end