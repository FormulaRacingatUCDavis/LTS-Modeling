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
            MedianOverResolve = MedianOrdering(Mid, Scale);
            Median = MedianOverResolve(1,:);
            for j=1:round(size(MedianOverResolve,1) / 10)
                Median = [Median; MedianOverResolve(j*10,:)];
            end
            MedianFit = csaps(Median(:,1),Median(:,2));
            MedianSpline = MedianSample(MedianFit);
            
            %% Linking and Discretizing
            
            Boundaries = BoundaryDefine( RightPoints, LeftPoints, RightSpline, LeftSpline, MedianSpline, Scale );
            
        case 3
            %% test
            
            %BW = roipoly(Image.Raw);
            %Mid = bwskel(BW);
            %imshow(labeloverlay(Image.Raw, Mid))
            %MedianOverResolve = MedianOrdering(Mid);
            %Median = MedianOverResolve(1,:);
            %for j=1:floor(size(MedianOverResolve,1) / 10)
            %    Median = [Median; MedianOverResolve(j*10,:)];
            %end
            %t = linspace( 1, size(Median, 1), size(Median, 1) );
            %MedianFit = csaps(t, Median');
            MedianSpline = MedianSample(MedianFit, Scale);
            Boundaries = BoundaryDefine( RightPoints, LeftPoints, RightSpline, LeftSpline, MedianSpline, Scale );
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

function Boundaries = BoundaryDefine( RightPoints, LeftPoints, RightSpline, LeftSpline, MedianSpline, Scale )
%% Generates n X 4 boundary points (x,y,x,y) with reduced resolution for pathing procedure

LinkCount=0;
LoopCount=0;
DistanceFindFail=0;
DistanceFindBounds=0;
DistanceFindOscillate=0;
Anomolies=0;

RDistanceOld = 0;
LDistanceOld = 0;
Distance = 1;
Increment = 6;

Boundaries.Points = [RightPoints.Length(1,:), LeftPoints.Length(1,:)];

i = 1; % Index of MedianSpline

while Distance <= MedianSpline.Distance(end)
    LoopCount=LoopCount+1;
    
    while Distance > MedianSpline.Distance(i)
        i=i+1;
    end
    
%    if i+1 <= size(MedianResamp.Length,1) && i-1 > 0
%        % Curvature to determine turn direction
%        Curvature = Curvature(MedianResamp, i);
%    end
    
    [link, rs, ls] = LinkSplines( MedianSpline, i, RightSpline, LeftSpline, RightPoints, LeftPoints, Scale );
    
    if link == false
        disp('no intersect found')
        Distance = Distance + 1;
    elseif link == true
        disp('no point found')
        Distance = Distance + 1;
    elseif size(link) ~= [1,4]
        disp('wrong link size')
        disp(link)
        Distance = Distance + 1;
    elseif norm(link(1:2)-link(3:4)) > 1.5 * norm(Boundaries.Points(end,1:2) - Boundaries.Points(end,3:4))
        disp('anomolous link generated')
        Distance = Distance + 1;
        Anomolies = Anomolies + 1;
    else
        RightResamp = BoundaryResample(RightSpline, rs, Scale);
        LeftResamp = BoundaryResample(LeftSpline, ls, Scale);
        RDiff = RightResamp.Length - link(1:2);
        LDiff = LeftResamp.Length - link(3:4);
        Rmin = norm(RDiff(1,:),2);
        Rindex = 1;
        for j=1:size(RDiff,1)
            if norm(RDiff(j,:),2) < Rmin
                Rmin = norm(RDiff(j,:),2);
                Rindex = j;
            end
        end
        Lmin = norm(LDiff(1,:),2);
        Lindex = 1;
        for j=1:size(LDiff,1)
            if norm(LDiff(j,:),2) < Lmin
                Lmin = norm(LDiff(j,:),2);
                Lindex = j;
            end
        end
        RDistance = RightResamp.Distance(Rindex);
        LDistance = LeftResamp.Distance(Lindex);
        if RDistance < RDistanceOld || LDistance < LDistanceOld
            disp('links intersect')
            DistanceFindBounds = DistanceFindBounds + 1;
            Distance = Distance + 1;
        else
            Boundaries.Points = [Boundaries.Points; link];
            Distance = Distance + Increment;
            LinkCount = LinkCount + 1;
            RDistanceOld = RDistance;
            LDistanceOld = LDistance;
        end
    end
end
Boundaries.Points = Boundaries.Points(2:end,:);
disp1=['points found: ',num2str(LinkCount)];
disp2=['number of loops: ',num2str(LoopCount)];
disp3=['distance find failures: ',num2str(DistanceFindFail)];
disp4=['distance found < distance old: ',num2str(DistanceFindBounds)];
disp5=['distance found + 1 < distance old: ',num2str(DistanceFindOscillate)]; 
disp6=['anomolous links ignored: ', num2str(Anomolies)];
disp(disp1)
disp(disp2)
disp(disp3)
disp(disp4)
disp(disp5)
disp(disp6)
for j=1:size(Boundaries.Points,1)
plot([Boundaries.Points(j,1);Boundaries.Points(j,3)],[Boundaries.Points(j,2);Boundaries.Points(j,4)])
hold on
end
end

function Sample = MedianSample( MedianFit, Scale )
t = linspace( MedianFit.breaks(1), MedianFit.breaks(end), 100 * MedianFit.pieces );
v = fnval(MedianFit, t);
Sample.Pixels = [ v(1,:)', v(2,:)'];
Sample.Length = Sample.Pixels .* Scale.Ratio;
Sample.Breaks = MedianFit.breaks;

Sample.Distance = 0;
for j=2:size(Sample.Length,1)
    Sample.Distance = [Sample.Distance; norm( Sample.Length(j-1,:) - Sample.Length(j,:) ) + Sample.Distance(j-1)];
end

Sample.Coeff.a = [0,0];
Sample.Coeff.b = [0,0];
Sample.Coeff.c = [0,0];
Sample.Coeff.d = [0,0];
for j=1 : MedianFit.pieces
    Sample.Coeff.a = [Sample.Coeff.a; MedianFit.coefs((j*2)-1:(j*2),4)'];
    Sample.Coeff.b = [Sample.Coeff.b; MedianFit.coefs((j*2)-1:(j*2),3)'];
    Sample.Coeff.c = [Sample.Coeff.c; MedianFit.coefs((j*2)-1:(j*2),2)'];
    Sample.Coeff.d = [Sample.Coeff.d; MedianFit.coefs((j*2)-1:(j*2),1)'];
end
Sample.Coeff.a = Sample.Coeff.a(2:end, : );
Sample.Coeff.b = Sample.Coeff.b(2:end, : );
Sample.Coeff.c = Sample.Coeff.c(2:end, : );
Sample.Coeff.d = Sample.Coeff.d(2:end, : );

end

function Curv = Curvature( Resamp, i )
dy = (Resamp.Length(i+1,2) - Resamp.Length(i,2)) / (Resamp.Distance(i+1) - Resamp.Distance(i));
dx = (Resamp.Length(i+1,1) - Resamp.Length(i,1)) / (Resamp.Distance(i+1) - Resamp.Distance(i));
cy = (Resamp.Length(i+1,2) - 2*Resamp.Length(i,2) + Resamp.Length(i-1,2)) / ((Resamp.Distance(i+1) - Resamp.Distance(i))^2);
cx = (Resamp.Length(i+1,1) - 2*Resamp.Length(i,1) + Resamp.Length(i-1,1)) / ((Resamp.Distance(i+1) - Resamp.Distance(i))^2);

Curv = (dx*cy - dy*cx)/(dx^2+dy^2)^(3/2);
end

function Resamp = BoundaryResample( Spline, i, Scale )
%% Resamples with constant arc length
u = linspace(0, 1, round( Spline.Distance(i*1000) - Spline.Distance((i-1)*1000 +1))*100)';
Resamp.Pixels = Spline.Coeff.a(i,:) + Spline.Coeff.b(i,:).* u + Spline.Coeff.c(i,:).* u.^2 + Spline.Coeff.d(i,:).* u.^3;
Resamp.Length = Resamp.Pixels .* Scale.Ratio;
Resamp.Distance = linspace( Spline.Distance((i-1)*1000 +1), Spline.Distance(i*1000), size(Resamp.Length,1));
end

function [link, rs, ls] = LinkSplines( MedianResamp, i, RightSpline, LeftSpline, RightPoints, LeftPoints, Scale )
rs = 0;
ls = 0;
if i+1 <= size(MedianResamp.Length, 1)
    MedianSlope = ( MedianResamp.Length(i+1,2) - MedianResamp.Length(i,2) ) ...
        / ( MedianResamp.Length(i+1,1) - MedianResamp.Length(i,1) );
else
    MedianSlope = ( MedianResamp.Length(i,2) - MedianResamp.Length(i-1,2) ) ...
        / ( MedianResamp.Length(i,1) - MedianResamp.Length(i-1,1) );
end
LinkingSlope = -1/MedianSlope;
MedianPoint = MedianResamp.Length(i,:);

RightIntersects = FindCubics( MedianPoint, LinkingSlope, RightPoints.Length );
LeftIntersects = FindCubics( MedianPoint, LinkingSlope, LeftPoints.Length );

if size(RightIntersects,1) == 0 || size(RightIntersects,2) == 0 ...
        || size(LeftIntersects,1) == 0 || size(LeftIntersects,2) == 0
    link = false;
else
    RightLinkPoints = [0,0];
    rs = 0;
    for j=1:size(RightIntersects)
        m = RightIntersects(j);
        F = @RightSplineIntersect;
        x = fsolve(F, [RightPoints.Length(m,1);RightPoints.Length(m,2); 0]);
        if x(3) > 0 && x(3) < 1
            RightLinkPoints = [RightLinkPoints; x(1), x(2)];
            rs = [rs; m];
        end
    end
    RightLinkPoints = RightLinkPoints(2:end,:);
    rs = rs(2:end);
    if size(RightLinkPoints,1) > 1
        width = 0;
        for j=1:size(RightLinkPoints,1)
            v = MedianResamp.Length(i,:) - RightLinkPoints(j,:);
            width = [width; norm(v)];
        end
        width = width(2:end);
        min = width(1);
        index = 1;
        for j=1:size(width)
            if min > width(j)
                min = width(j);
                index = j;
            end 
        end
        disp('indexed right link')
        disp(index)
        link = RightLinkPoints(index,:);
        rs = rs(index);
    elseif size(RightLinkPoints,1) == 1
        disp('single right link')
        link = RightLinkPoints;
    else
        link = true;
    end
end

disp(link)

if size(link, 1) ~= 1 || size(link, 2) ~= 2
    link = true;
else
    LeftLinkPoints = [0,0];
    ls = 0;
    for j=1:size(LeftIntersects)
        m = LeftIntersects(j);
        F = @LeftSplineIntersect;
        x = fsolve(F, [LeftPoints.Length(m,1);LeftPoints.Length(m,2); 0]);
        if x(3) > 0 && x(3) < 1
            LeftLinkPoints = [LeftLinkPoints; x(1), x(2)];
            ls = [ls; m];
        end
    end
    LeftLinkPoints = LeftLinkPoints(2:end,:);
    ls = ls(2:end);
    if size(LeftLinkPoints,1) > 1
        width = 0;
        for j=1:size(LeftLinkPoints,1)
            v = MedianResamp.Length(i,:) - LeftLinkPoints(j,:);
            width = [width; norm(v)];
        end
        width = width(2:end);
        min = width(1);
        index = 1;
        for j=1:size(width)
            if min > width(j)
                min = width(j);
                index = j;
            end 
        end
        disp('indexed left link')
        disp(index)
        disp(LeftLinkPoints(index,:))
        link = [link, LeftLinkPoints(index,:)];
        ls = ls(index);
        if norm(MedianPoint - link(1:2)) > norm(link(1:2) - link(3:4)) || norm(MedianPoint - link(3:4)) > norm(link(1:2) - link(3:4))
            link = true;
        end
    elseif size(LeftLinkPoints,1) == 1
        disp('single left link')
        disp(LeftLinkPoints)
        link = [link, LeftLinkPoints];
        if norm(MedianPoint - link(1:2)) > norm(link(1:2) - link(3:4)) || norm(MedianPoint - link(3:4)) > norm(link(1:2) - link(3:4))
            link = true;
        end
    else
        link = true;
    end
end


    function intersects = FindCubics( MedianPoint, LinkingSlope, Points )
        % Returns vector of all indeces of Points where intersects between point slope line and
        % line segments between Points(m,:) and Points(m+1,:) occur
        m=1;
        intersects = 0;
        
        while m < size(Points, 1)
            t = ( ( MedianPoint(2) - LinkingSlope * MedianPoint(1) ) - Points(m,2) + LinkingSlope * Points(m,1) ) / ...
                ( Points(m+1,2) - Points(m,2) + LinkingSlope * ( Points(m,1) - Points(m+1,1) ) );
            if t < 1 && t > 0
                intersects = [intersects; m];
            end
            m=m+1;
        end
        intersects = intersects(2:end);
        
    end

    function F = RightSplineIntersect(x)
        c = [x(1), x(2)];
        f1 = ( RightSpline.Coeff.a(m,:) + RightSpline.Coeff.b(m,:).*x(3) + RightSpline.Coeff.c(m,:).*x(3)^2 + RightSpline.Coeff.d(m,:).*x(3)^3 ) * Scale.Ratio - c;
        f2 = LinkingSlope*x(1) - LinkingSlope*MedianResamp.Length(i,1) + MedianResamp.Length(i,2) - x(2);
        F = [f1'; f2];
    end

    function F = LeftSplineIntersect(x)
        c = [x(1), x(2)];
        f2 = LinkingSlope*x(1) - LinkingSlope*MedianResamp.Length(i,1) + MedianResamp.Length(i,2) - x(2);
        f1 = ( LeftSpline.Coeff.a(m,:) + LeftSpline.Coeff.b(m,:).*x(3) + LeftSpline.Coeff.c(m,:).*x(3)^2 + LeftSpline.Coeff.d(m,:).*x(3)^3 ) * Scale.Ratio - c;
        F = [f1'; f2];
    end

end

function Median = MedianOrdering( Mid )
[my,mx] = find(Mid);
Mid = [mx,my];
[x,y] = getpts;
CurrPoint = [round(x),round(y)];
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

Median = Median(2:end, :);

end
