clc; clear; close all;

%% Triangle Parameterization Test
% Parameterizing triangle by two edges

p = [-2, 4, 7; ...
     -3, 1,-4];

v = p(:,2:end) - p(:,1);

[X1(1,:,:), X2(1,:,:)] = meshgrid( 0:0.1:1, 0:0.1:1 );

Data = p(:,1) + X1 .* v(:,1) + X2 .* v(:,2); 

X_Data = squeeze(Data(1,:,:));
Y_Data = squeeze(Data(2,:,:));

scatter( X_Data(X1+X2<=1), Y_Data(X1+X2<=1), 'k.' )