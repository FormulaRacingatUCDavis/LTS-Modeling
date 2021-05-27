%clc; clear; close all;

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
%
% Drew La Spada - dalaspada@ucdavis.edu
%
% v3.0 - (05/08/2021) Initial track boundary definition

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
    
    %% Boundary or Arbitrary Spline
    SplineType= input('Enter 1 to select arbitrary spline, 2 to define course boundaries: ');
    switch SplineType
        case 1
            %% Spline Creation  
            [Points, Spline] = SelectSpline( Image, Scale, RadiusThresh );
            %moved out of funct spline generation for course boundary purposes
            Spline.Length = Spline.Length - Spline.Length(1,:); 
            Points.Length = Points.Length - Points.Length(1,:);
            
            %% Corner Correction
            Spline = CornerSmoothing( Spline, RadiusThresh );
        case 2
            %% Define Right Boundary
            disp('Select points along the right boundary')
            
            [RightPoints, RightSpline]=SelectSpline( Image, Scale, RadiusThresh );
            
            %% Define Left Boundary
            disp('Select points along the left boundary')
            
            [LeftPoints, LeftSpline]=SelectSpline( Image, Scale, RadiusThresh );
            
            %% Define Median line
            BW = roipoly(Image.Raw);
            Mid = bwskel(BW);
            imshow(labeloverlay(Image.Raw, Mid))
            Median = MedianOrdering(Mid, Scale);
            
            %% Linking and Discretizing
            % resample boundaries, orient, link boundary
            
            Boundaries = BoundaryDefine( RightPoints, LeftPoints, RightSpline, LeftSpline, Scale );
            
        case 3
            %% test
            
            BW = roipoly(Image.Raw);
            Mid = bwskel(BW);
            imshow(labeloverlay(Image.Raw, Mid))
            MedianOverResolve = MedianOrdering(Mid, Scale);
            Median = MedianOverResolve(1,:);
            for j=1:round(size(MedianOverResolve,1) / 5)
                Median = [Median; MedianOverResolve(j*5,:)];
            end
            MedianFit = csaps(Median(:,1),Median(:,2));
            plot(Median(:,1),Median(:,2))
            Boundaries = BoundaryDefine( RightPoints, LeftPoints, RightSpline, LeftSpline, Scale );
    end
end

clear File

%% Plotting
if SplineType == 1
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

    title( ['Radius distribution: ', ...
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
end

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
            xMin = xData( IsMin([1 end]) );% errorlog: Array indices must be positive integers or logical values.
            yMin = yData( IsMin([1 end]) );
            
            yData(IsMin(1) : IsMin(end)) = interp1( xMin, yMin, xData(IsMin(1) : IsMin(end)) );
            
            Spline.Radius(Entry.Idx(i) : Exit.Idx(i),2) = ...
                sign( Spline.Radius(Entry.Idx(i)) ) .* yData; 
        end
    end
end

function Boundaries = BoundaryDefine( RightPoints, LeftPoints, RightSpline, LeftSpline, Scale )
%% Generates n X 4 boundary points (x,y,x,y) with reduced resolution for pathing procedure

LinkCount=0;
LoopCount=0;
DistanceFindFail=0;
DistanceFindBounds=0;
DistanceFindOscillate=0;

RDistanceOld = 0;
LDistanceOld = 0;
RDistance = 1;
LDistance = 1;

Boundaries.Points = [RightPoints.Length(1,:), LeftPoints.Length(1,:)];
link = Boundaries.Points;
LeftIsInside = true;

rs = 1; % Index of RightPoints
ls = 1; % Index of LeftPoints

while RDistance < RightSpline.Distance(end) && LDistance < LeftSpline.Distance(end)
    LoopCount=LoopCount+1;
    
    rsIncrease = 0;
    while RDistance > RightSpline.Distance(rs*1000)
        rs = rs + 1;
        rsIncrease = rsIncrease + 1;
    end
    
    lsIncrease = 0;
    while LDistance > LeftSpline.Distance(ls*1000)
        ls = ls + 1;
        lsIncrease = lsIncrease + 1;
    end
    
    if rs > size( RightPoints.Length, 1) || ls > size( LeftPoints.Length, 1)
        break
    end
    
    LeftResamp = SplineResample(LeftSpline, ls, Scale);
    RightResamp = SplineResample(RightSpline,rs, Scale);
    i = 1; % Index of RightResamp
    n = 1; % Index of LeftResamp
    
    if RDistance < RightResamp.Distance(1)
        disp('wrong right spline selected')
        continue
    end
    if LDistance < LeftResamp.Distance(1)
        disp('wrong left spline selected')
        continue
    end
    
    while RDistance > RightResamp.Distance(i)
        i=i+1;
    end
    while LDistance > LeftResamp.Distance(n)
        n=n+1;
    end
    
    if i+1 <= size(RightResamp.Length,1) && n+1 <= size(LeftResamp.Length,1) && n-1 > 0 && i-1 > 0
        % Average curvature to determine turn direction
        RightCurvature = Curvature(RightResamp, i);
        LeftCurvature = Curvature(LeftResamp, n);
        AvgCurv =( RightCurvature + LeftCurvature ) / 2;
    end
    
    if AvgCurv <= 0
        PrevInside = LeftIsInside;
        LeftIsInside = true;
        rs=rs-rsIncrease;
        if size(link,1) == 1 && size(link,2) == 4
            linkOld = [link(3:4), link(1:2)];
        end
        link = LinkSplines( LeftResamp, RightSpline, RightPoints.Length, n, rs, Scale, linkOld );
        if link == false
            disp('no right intersect found')
            RDistance=RDistance+1;
            LDistance=LDistance+1;
        elseif link == true
            disp('no right point found')
            RDistance=RDistance+1;
            LDistance=LDistance+1;
        elseif size(link) ~= [1,4]
            disp('wrong link size')
            disp(link)
            RDistance=RDistance+1;
            LDistance=LDistance+1;
        else
            Boundaries.Points = [Boundaries.Points; [link(3:4), link(1:2)]];
            Diff = RightResamp.Length - link(3:4);
            Index = 1;
            minimum = norm([Diff(1,1), Diff(1,2)]);
            for j=1:size(Diff,1)
                if norm([Diff(j,1), Diff(j,2)]) < minimum
                    minimum = norm([Diff(j,1), Diff(j,2)]);
                    Index = j;
                end
            end
            
            if RightResamp.Distance(Index(1)) >= RDistanceOld
                RDistanceOld = RightResamp.Distance(Index(1));
                RDistance = RightResamp.Distance(Index(1)) + 1;
                LinkCount=LinkCount+1;

            elseif RightResamp.Distance(Index(1)) + 1 >= RDistanceOld
                disp('distance found < distance old')
                disp(RightResamp.Distance(Index(1)))
                disp(RDistanceOld)
                DistanceFindBounds = DistanceFindBounds + 1;
                RDistance = RDistance + 1;
            else
                disp('distance found + 1 < distance old')
                DistanceFindOscillate = DistanceFindOscillate + 1;
                RDistance = RDistance + 1;
            end
            
            LDistanceOld = LDistance;
            LDistance=LDistance+1;
        end
        
    else
        PrevInside = LeftIsInside;
        LeftIsInside = false;
        ls=ls-lsIncrease;
        if size(link,1) == 1 && size(link,2) == 4
            linkOld = link;
        end
        link = LinkSplines( RightResamp, LeftSpline, LeftPoints.Length, i, ls, Scale, linkOld );
        if link == false
            disp('no left intersect found')
            RDistance=RDistance+1;
            LDistance=LDistance+1;
        elseif link == true
            disp('no left point found')
            RDistance=RDistance+1;
            LDistance=LDistance+1;
        elseif size(link) ~= [1,4]
            disp('wrong link size')
            disp(link)
            RDistance=RDistance+1;
            LDistance=LDistance+1;
        else
            
            Boundaries.Points = [Boundaries.Points; [link(1:2), link(3:4)]];
            Diff = LeftResamp.Length - link(3:4);
            Index = 1;
            minimum = norm([Diff(1,1), Diff(1,2)]);
            for j=1:size(Diff,1)
                if norm([Diff(j,1), Diff(j,2)]) < minimum
                    minimum = norm([Diff(j,1), Diff(j,2)]);
                    Index = j;
                end
            end

            if LeftResamp.Distance(Index(1)) >= LDistanceOld
                LDistanceOld = LeftResamp.Distance(Index(1));
                LDistance = LeftResamp.Distance(Index(1)) + 1;
                LinkCount=LinkCount+1;

            elseif LeftResamp.Distance(Index(1)) + 1 >= LDistanceOld
                disp('distance found < distance old')
                disp(LeftResamp.Distance(Index(1)))
                disp(LDistanceOld)
                DistanceFindBounds = DistanceFindBounds + 1;
                LDistance = LDistance + 1;
            else
                disp('distance found + 1 < distance old')
                DistanceFindOscillate = DistanceFindOscillate + 1;
                LDistance = LDistance + 1;
            end

            RDistanceOld = RDistance;
            RDistance = RDistance+1;
        end
        
    end
end
disp1=['points found: ',num2str(LinkCount)];
disp2=['number of loops: ',num2str(LoopCount)];
disp3=['distance find failures: ',num2str(DistanceFindFail)];
disp4=['distance found < distance old: ',num2str(DistanceFindBounds)];
disp5=['distance found + 1 < distance old: ',num2str(DistanceFindOscillate)]; 
disp(disp1)
disp(disp2)
disp(disp3)
disp(disp4)
disp(disp5)
for j=1:size(Boundaries.Points,1)
plot([Boundaries.Points(j,1);Boundaries.Points(j,3)],[Boundaries.Points(j,2);Boundaries.Points(j,4)])
hold on
end
end

function Curv = Curvature( Resamp, i )
dy = (Resamp.Length(i+1,2) - Resamp.Length(i,2)) / (Resamp.Distance(i+1) - Resamp.Distance(i));
dx = (Resamp.Length(i+1,1) - Resamp.Length(i,1)) / (Resamp.Distance(i+1) - Resamp.Distance(i));
cy = (Resamp.Length(i+1,2) - 2*Resamp.Length(i,2) + Resamp.Length(i-1,2)) / ((Resamp.Distance(i+1) - Resamp.Distance(i))^2);
cx = (Resamp.Length(i+1,1) - 2*Resamp.Length(i,1) + Resamp.Length(i-1,1)) / ((Resamp.Distance(i+1) - Resamp.Distance(i))^2);

Curv = (dx*cy - dy*cx)/(dx^2+dy^2)^(3/2);
end

function Resamp = SplineResample( Spline, i, Scale )
%% Resamples with constant arc length
u = linspace(0, 1, round( Spline.Distance(i*1000) - Spline.Distance((i-1)*1000 +1))*100)';
Resamp.Pixels = Spline.Coeff.a(i,:) + Spline.Coeff.b(i,:).* u + Spline.Coeff.c(i,:).* u.^2 + Spline.Coeff.d(i,:).* u.^3;
Resamp.Length = Resamp.Pixels .* Scale.Ratio;
Resamp.Distance = linspace( Spline.Distance((i-1)*1000 +1), Spline.Distance(i*1000), size(Resamp.Length,1));
end

function link = LinkSplines( InsideResamp, OutsideSpline, OutsidePoints, i, os, Scale, linkOld )
%% Input spline structure of outside, Resample of inside, points.Length of outside, and i/n for inside=RightSpline/LeftSpline

if i+1 <= size(InsideResamp.Length, 1)
    InsideSlope = ( InsideResamp.Length(i+1,2) - InsideResamp.Length(i,2) ) / ( InsideResamp.Length(i+1,1) - InsideResamp.Length(i,1) );
else
    InsideSlope = ( InsideResamp.Length(i,2) - InsideResamp.Length(i-1,2) ) / ( InsideResamp.Length(i,1) - InsideResamp.Length(i-1,1) );
end
LinkingSlope = -1/InsideSlope;
SplineIden = OutsidePoints;

m=1;
intersects = 0;

while m < size(SplineIden, 1)
    t = ( ( InsideResamp.Length(i,2) - LinkingSlope * InsideResamp.Length(i,1) ) - SplineIden(m,2) + LinkingSlope * SplineIden(m,1) ) / ...
        ( SplineIden(m+1,2) - SplineIden(m,2) + LinkingSlope * ( SplineIden(m,1) - SplineIden(m+1,1) ) );
    if t < 1 && t > 0
        intersects = [intersects; m];
    end
    m=m+1;
end
intersects = intersects(2:end);

if size(intersects,1) == 0 || size(intersects,2)==0
    link = false;
else
    links = [0,0,0,0];
    for j=1:size(intersects)
        m = intersects(j);
        F = @SplineIntersect;
        x = fsolve(F, [SplineIden(m,1); SplineIden(m,2); 0]);
        if x(3) > 0 && x(3) < 1
            if InsideSlope < 1 && InsideSlope > -1
                if linkOld(1) <= linkOld(3)
                    if InsideResamp.Length(i,1) < x(1)
                        links = [links; InsideResamp.Length(i,1), InsideResamp.Length(i,2), x(1), x(2)];
                    end
                else
                    if InsideResamp.Length(i,1) > x(1)
                        links = [links; InsideResamp.Length(i,1), InsideResamp.Length(i,2), x(1), x(2)];
                    end
                end
            else
                if linkOld(2) <= linkOld(4)
                    if InsideResamp.Length(i,2) < x(2)
                        links = [links; InsideResamp.Length(i,1), InsideResamp.Length(i,2), x(1), x(2)];
                    end
                else
                    if InsideResamp.Length(i,2) > x(2)
                        links = [links; InsideResamp.Length(i,1), InsideResamp.Length(i,2), x(1), x(2)];
                    end
                end
            end
        end
    end
    links = links(2:end,:);
    if size(links,1) > 1
        width = 0;
        for j=1:size(links,1)
            v = links(j,1:2) - links(j,3:4);
            width = [width; norm(v)];
        end
        width = width(2:end);
        k = min(width);
        index = find(width==k);
        disp('indexed link')
        link = links(index,:);
        disp(link)
    elseif size(links,1) == 1
        disp('single link')
        link = links;
        disp(link)
    else
        link = true;
    end
end

function F = SplineIntersect(x)
c = [x(1), x(2)];
f1 = ( OutsideSpline.Coeff.a(m,:) + OutsideSpline.Coeff.b(m,:).*x(3) + OutsideSpline.Coeff.c(m,:).*x(3)^2 + OutsideSpline.Coeff.d(m,:).*x(3)^3 ) * Scale.Ratio - c;
f2 = LinkingSlope*x(1) - LinkingSlope*InsideResamp.Length(i,1) + InsideResamp.Length(i,2) - x(2);
F = [f1'; f2];
end

end

function Median = MedianOrdering(Mid, Scale)
[my,mx] = find(Mid);
Mid = [mx,my];
[x,y] = getpts;
CurrPoint = [round(x),round(y)];
disp(Mid)
disp(CurrPoint)
Median = [0,0];
for j=1:size(Mid,1)
        if CurrPoint(end,:) == Mid(j,:)
            Index = j;
        end
end

while size(Mid,1) > 1
    
    Median = [Median; Mid(Index,:)];
    if Index - 1 >= 1 && Index + 1 <= size(Mid,1)
        Mid = [Mid(1:Index-1,:); Mid(Index+1:end,:)];
    elseif Index == 1
        Mid = Mid(2:end, :);
    elseif Index == size(Mid,1)
        Mid = Mid(1:end-1,:);
    end
    Diff = Mid - CurrPoint(end,:);
    min = norm(Diff(1,:));
    for j=1:size(Diff)
        if norm(Diff(j,:)) <= min
            Index = j;
            min = norm(Diff(j,:));
        end
    end
    
    CurrPoint = Mid(Index,:);
end

Median = Median(2:end, :) .* Scale.Ratio;

end
