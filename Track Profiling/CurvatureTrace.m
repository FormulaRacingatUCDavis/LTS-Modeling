function Track = CurvatureTrace( Spline )
resolution = 5;
t=resolution;
s=1;

dy = Spline.Coeff.b(s,2); % dy/du
dx = Spline.Coeff.b(s,1); % dx/du
cy = 2 * Spline.Coeff.c(s, 2); % d^2y/du^2
cx = 2 * Spline.Coeff.c(s, 1); % d^2x/du^2
Track.Curvature = (dx*cy - dy*cx)/(dx^2+dy^2)^(3/2); % Curvature
Track.Distance = 0;
while t <= Spline.Distance(end)
    while t > Spline.Distance(s*1000)
        s=s+1;
    end
    u = (t - Spline.Distance((s-1)*1000 +1))/...
        (Spline.Distance(s*1000) - Spline.Distance((s-1)*1000 +1));
    
    dy = Spline.Coeff.b(s,2) + 2 * Spline.Coeff.c(s,2) * u + 3 * Spline.Coeff.d(s,2) * u^2;
    dx = Spline.Coeff.b(s,1) + 2 * Spline.Coeff.c(s,1) * u + 3 * Spline.Coeff.d(s,1) * u^2;
    cy = 2 * Spline.Coeff.c(s, 2) + 6 * Spline.Coeff.d(s,2) * u;
    cx = 2 * Spline.Coeff.c(s, 1) + 6 * Spline.Coeff.d(s,1) * u;
    
    Track.Curvature = [Track.Curvature; (dx*cy - dy*cx)/(dx^2+dy^2)^(3/2)];
    Track.Distance = [Track.Distance; t];
    t = t + resolution;
end
end