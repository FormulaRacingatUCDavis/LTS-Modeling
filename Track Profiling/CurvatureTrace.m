function Curv = CurvatureTrace( Spline )
resolution = 5;
t=resolution;
s=1;

cy = 2 * Spline.Coeff.c(s, 2); % d^2y/du^2
cx = 2 * Spline.Coeff.c(s, 1); % d^2x/du^2
c = sqrt( cx^2 + cy^2 ); % Curvature
Curv = [c, 0];
while t <= Spline.Distance(end)
    while t > Spline.Distance(s*1000)
        s=s+1;
    end
    u = (t - Spline.Distance((s-1)*1000 +1))/...
        (Spline.Distance(s*1000) - Spline.Distance((s-1)*1000 +1));
    cy = 2 * Spline.Coeff.c(s, 2) + 6 * Spline.Coeff.d(s,2) * u;
    cx = 2 * Spline.Coeff.c(s, 1) + 6 * Spline.Coeff.d(s,1) * u;
    c = sqrt( cx^2 + cy^2 );
    
    Curv = [Curv; c, t];
    t = t + resolution;
end
end