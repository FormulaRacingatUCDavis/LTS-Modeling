function Curv = CurvatureTrace( Spline )
resolution = 1;
t=0;
s=1;

cy = 2 * Spline.Coeff.c(s, 2); % d^2y/du^2
cx = 2 * Spline.Coeff.c(s, 1); % d^2x/du^2
c = cy/cx; % d^2y/dx^2
Curv = [c, t];
while t < Spline.Distance(end)
    while t > Spline.Distance(s*1000)
        s=s+1;
    end
    u = (t - Spline.Distance((s-1)*1000 +1))/...
        (Spline.Distance(s*1000) - Spline.Distance((s-1)*1000 +1));
    cy = 2 * Spline.Coeff.c(s, 2) + 6 * Spline.Coeff.d(s,2) * u;
    cx = 2 * Spline.Coeff.c(s, 1) + 6 * Spline.Coeff.d(s,1) * u;
    c = cy/cx;
    
    Curv = [Curv; c, t];
    t = t + resolution;
end
end