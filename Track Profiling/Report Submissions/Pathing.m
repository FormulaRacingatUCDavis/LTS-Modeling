function [Spline, CtrPoints] = Pathing(Boundaries)
%% Pathing script finds control points defining the minimum curvature path through a track represented by n X 4 vehicle midline bounds matrix [x,y,x,y]

dx = Boundaries(:,3) - Boundaries(:,1);
dy = Boundaries(:,4) - Boundaries(:,2);
dx = diag(dx);
dy = diag(dy);

RSpline = DefineSpline(Boundaries(:,1:2));
LSpline = DefineSpline(Boundaries(:,3:4));

% generate right spline, d2x(rightctrlpts)/dt2 = 2 C_xi
d2xdt2r = 2 * RSpline.Coeff.c(:,1);
d2xdt2r = [d2xdt2r; 2 * RSpline.Coeff.c(end,1) + 6 * RSpline.Coeff.d(end,1)];
d2xdt2l = 2 * LSpline.Coeff.c(:,1);
d2xdt2l = [d2xdt2l; 2 * LSpline.Coeff.c(end,1) + 6 * LSpline.Coeff.d(end,1)];

D = diag( -2*ones( size(Boundaries,1)  ,1 )   ) + ...
    diag(   ones( size(Boundaries,1)-1,1 ),-1) + ...
    diag(   ones( size(Boundaries,1)-1,1 ), 1);
D(1,1) = 1;
D(1,2) = -2;
D(1,3) = 1;
D(end,end) = 1;
D(end,end-1) = -2;
D(end,end-2) = 1;

d2ydt2r = 2 * RSpline.Coeff.c(:,2);
d2ydt2r = [d2ydt2r; 2 * RSpline.Coeff.c(end,2) + 6 * RSpline.Coeff.d(end,2)];
d2xdt2l = 2 * LSpline.Coeff.c(:,2);
d2xdt2l = [d2xdt2l; 2 * LSpline.Coeff.c(end,2) + 6 * LSpline.Coeff.d(end,2)];

G = D;

H = (dx') * (D') * D * dx + (dy') * (G') * G * dy;
B = (Boundaries(:,1)') * (D') * D * dx + (Boundaries(:,2)') * (G') * G * dy ;
alpha0 = zeros(size(Boundaries(:,1)));
alpha1 = ones(size(Boundaries(:,1)));
Aeq = [];
A = [];
beq = [];
b = [];
options = optimoptions('quadprog', 'OptimalityTolerance', 10^-15, 'StepTolerance', 10^-30);
alpha = quadprog(H, B', A, b, Aeq, beq, alpha0, alpha1, alpha1 ./ 2, options);

x = Boundaries(:,1) + dx * alpha;
y = Boundaries(:,2) + dy * alpha;
CtrPoints = [x,y];
Spline = DefineSpline(CtrPoints);

    function F = DefineDMatrix(D)
        f1 = D * Boundaries(:,1) - d2xdt2r;
        f2 = D * Boundaries(:,3) - d2xdt2l;
        F = [f1; f2];
    end

    function F = DefineGMatrix(G)
        f1 = G * Boundaries(:,2) - d2ydt2r;
        f2 = G * Boundaries(:,4) - d2ydt2l;
        F = [f1; f2];
    end

    function Spline = DefineSpline(Points)
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
        Spline.Length = [];
        d             = [];
        c             = [];

        for i = 1 : size(Points,1)-1
            pi = Spline.Coeff.a(i,:)       + Spline.Coeff.b(i,:).*u   + ...
                 Spline.Coeff.c(i,:).*u.^2 + Spline.Coeff.d(i,:).*u.^3;  

            di =    Spline.Coeff.b(i,:) + 2.*Spline.Coeff.c(i,:) .* u + 3.*Spline.Coeff.d(i,:) .* u.^2;
            ci = 2.*Spline.Coeff.c(i,:) + 6.*Spline.Coeff.d(i,:) .* u;

            Spline.Length = [Spline.Length; pi];
            d = [d; di]; %#ok<AGROW>
            c = [c; ci]; %#ok<AGROW>
        end
        
        Spline.Distance = zeros( size(Spline.Length,1), 1 );
        for i = 2 : size(Spline.Length,1)
            Spline.Distance(i) = Spline.Distance(i-1) + norm( Spline.Length(i,:)-Spline.Length(i-1,:), 2 );
        end
    end

end

