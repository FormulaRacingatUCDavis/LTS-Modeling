clc; clear; close all;

Event = load( ['C:\Users\Authorized User\Documents\Clubs\GitHub\LTS-Modeling\', ...
               'Track Profiling\Analyses\2015-18_Lincoln_AutoX.mat'] );

Apex = islocalmin( abs(Event.Spline.Radius(:,2)), 'MinProminence', 2 );
Apex = Apex & Event.Spline.IsCorner;

figure;
plot( Event.Spline.Distance, abs(Event.Spline.Radius(:,2)) ); hold on;
plot( Event.Spline.Distance(Apex), abs(Event.Spline.Radius(Apex ,2)), 'ro' );
ylim([0 150]);

