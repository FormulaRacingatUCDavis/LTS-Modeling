%% Octahedral Test MMM
% Defines an octahedral Milliken Moment Diagram that scales with speed for
% testing lap time simulation solvers.
%
% Blake Christierson - bechristierson@ucdavis.edu
%
% Version 1 - 1/14/2021

%% Corner Conditions
v = linspace(0,50,11); % Array of Speeds [m/s]

a_drive =  1.4 + ( 0.2-1.4).*(v./50).^2; % Peak Traction Acceleration  [g]
a_brake = -1.7 + (-2.0+1.7).*(v./50).^2; % Peak Braking  Acceleration  [g]

a_bound = -0.1 + (-0.2+0.1).*(v./50).^2; % Longitudinal Boundary Level [g] 

r_limit = -2.0 + (-0.5+2.0).*(v./50).^2; % Limit Yaw Acceleration      [rad/s^2]
a_limit =  1.5 + ( 1.7-1.5).*(v./50).^2; % Limit Lateral Acceleration  [g]

r_peak  =  25  + ( 23 -25 ).*(v./50).^2; % Peak Yaw Acceleration       [rad/s^]
a_peak  = -0.15.*ones(size(v));          % Peak Lateral Acceleration   [g]

%% Form MMM Point Cloud
MMM = zeros(4,6*length(v));

for i = 1:length(v)
   MMM(:,6*(i-1)+1) = [v(i); a_drive(i);  0         ;  0         ]; 
   MMM(:,6*(i-1)+2) = [v(i); a_bound(i);  a_peak(i) ;  r_peak(i) ];
   MMM(:,6*(i-1)+3) = [v(i); a_bound(i);  a_limit(i);  r_limit(i)];
   MMM(:,6*(i-1)+4) = [v(i); a_bound(i); -a_peak(i) ; -r_peak(i) ];
   MMM(:,6*(i-1)+5) = [v(i); a_bound(i); -a_limit(i); -r_limit(i)];
   MMM(:,6*i      ) = [v(i); a_brake(i);  0         ;  0         ];
end

figure;
scatter3( MMM(2,:), MMM(3,:), MMM(4,:), 10, MMM(1,:), 'filled' )

title( 'Octahedral MMM Corner Movement' )
xlabel( 'Longitudinal Acceleration [g]' )
ylabel( 'Lateral Acceleration [g]' )
zlabel( 'Yaw Acceleration [rad/s^2]' )

h = colorbar;
h.Label.String = 'Speed [m/s]';

%% Solve for GGV Surface from MMM (Local repositiory example)
