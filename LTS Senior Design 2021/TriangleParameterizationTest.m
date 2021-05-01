clc; clear; close all;

%% Simplex Parameterization Test
% Parameterizing triangle by two edges

%% 2D Test (Triangle)
p = [-2, 4, 7; ...
     -3, 1,-4];

v = p(:,2:end) - p(:,1);

[X1(1,:,:), X2(1,:,:)] = meshgrid( 0:0.1:1, 0:0.1:1 );

Data = p(:,1) + X1 .* v(:,1) + X2 .* v(:,2); 

X_Data = squeeze(Data(1,:,:));
Y_Data = squeeze(Data(2,:,:));

figure;
scatter( X_Data(X1+X2<=1), Y_Data(X1+X2<=1), 'k.' );

%% 3D Test (Tetrahedron)
p = [-2, 4, 7, 5; ...
     -3, 1,-4, 2; ...
      0,-3, 4, 2];

v = p(:,2:end) - p(:,1);

n = 5;
X1 = ones( 1,n,n,n );
X2 = ones( 1,n,n,n );
X3 = ones( 1,n,n,n );

[X1(1,:,:,:), X2(1,:,:,:), X3(1,:,:,:)] = ndgrid( linspace(0,1,n), linspace(0,1,n), linspace(0,1,n) );

Data = p(:,1) + X1 .* v(:,1) + X2 .* v(:,2) + X3 .* v(:,3); 

X_Data = squeeze(Data(1,:,:));
Y_Data = squeeze(Data(2,:,:));
Z_Data = squeeze(Data(3,:,:));

figure;
scatter3( X_Data(X1+X2+X3<=1), Y_Data(X1+X2+X3<=1), Z_Data(X1+X2+X3<=1), 'k.' );