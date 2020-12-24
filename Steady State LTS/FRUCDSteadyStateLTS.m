clc; clear; close all;

Event = load( ['C:\Users\Authorized User\Documents\Clubs\GitHub\LTS-Modeling\', ...
               'Track Profiling\Analyses\2015-18_Lincoln_AutoX.mat'] );


Apex = islocalmin( abs(Event.Spline.Radius) );

figure;
plot( Event.Spline.Distance, abs(Event.Spline.Radius) ); hold on;
plot( Event.Spline.Distance(Apex), abs(Event.Spline.Radius(Apex)), 'ro' );
ylim([-250 250]);

figure
imshow( Event.Image.Raw ); hold on;
scatter( Event.Spline.Pixels( Event.Spline.Radius>100,1 ), Event.Spline.Pixels( Event.Spline.Radius>100,2 ), 2, 'k' )
scatter( Event.Spline.Pixels( Event.Spline.Radius<100,1 ), Event.Spline.Pixels( Event.Spline.Radius<100,2 ), 2, 'r' )
diff( find( Event.Spline.Radius > 250 ) )


