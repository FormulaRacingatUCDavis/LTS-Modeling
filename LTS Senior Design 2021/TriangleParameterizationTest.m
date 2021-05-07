clc; clear; close all;

%% Simplex Parameterization Test
% Parameterizing triangle by two edges

% - n-d simplex is defined by n+1 points which generates n edges.
% - There are n chi's which represent the distance along each of n edges.
% - The sum of the chi's is constrained to be <= 1 to produce the simplex and
% not the parallelogram (2d) / parallelopiped (3d) / ??? (n-d)
% - I suggest formatting:
%       P = points as [n,n+1] matrix
%       E = edges as [n,n] matrix 
%       Chi = chi as [n,1] vector
% 

%% 2D Test (Triangle)
P = [-2, 4, 7; ...
     -3, 1,-4];

E = P(:,2:end) - P(:,1);

k = 4; 
Chi = zeros(2,k,k);
[Chi(1,:,:), Chi(2,:,:)] = meshgrid( linspace(0,1,k), linspace(0,1,k) );

Eval = P(:,1) + Chi(1,:,:) .* E(:,1) + X2 .* E(:,2); 

X_Data = squeeze(Data(1,:,:));
Y_Data = squeeze(Data(2,:,:));

figure;
scatter( X_Data(X1+X2<=1), Y_Data(X1+X2<=1), 'k.' );

%% 3D Test (Tetrahedron)
p = [-2, 4, 7, 5; ...
     -3, 1,-4, 2; ...
      0,-3, 4, 2];

E = p(:,2:end) - p(:,1);

n = 5;
X1 = ones( 1,n,n,n );
X2 = ones( 1,n,n,n );
X3 = ones( 1,n,n,n );

[X1(1,:,:,:), X2(1,:,:,:), X3(1,:,:,:)] = ndgrid( linspace(0,1,n), linspace(0,1,n), linspace(0,1,n) );

Data = p(:,1) + X1 .* E(:,1) + X2 .* E(:,2) + X3 .* E(:,3); 

X_Data = squeeze(Data(1,:,:));
Y_Data = squeeze(Data(2,:,:));
Z_Data = squeeze(Data(3,:,:));

figure;
scatter3( X_Data(X1+X2+X3<=1), Y_Data(X1+X2+X3<=1), Z_Data(X1+X2+X3<=1), 'k.' );