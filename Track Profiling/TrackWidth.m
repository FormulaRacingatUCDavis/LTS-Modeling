function Boundaries = TrackWidth(Boundaries, Width)
%% Function takes input boundary point array in meters and vehicle width in meters, outputs bounds for vehicle centerline
for j = 1 : size(Boundaries, 1)
    MidPoint = [ (Boundaries(j, 1) + Boundaries(j,3)) / 2, (Boundaries(j,2) + Boundaries(j,4)) / 2 ];
    Dist = norm(Boundaries(j,1:2) - Boundaries(j,3:4));
    ySens = (Boundaries(j,2) - Boundaries(j,4)) / Dist;
    xSens = (Boundaries(j,1) - Boundaries(j,3)) / Dist;
    Boundaries(j,1) = MidPoint(1) + ( ( Dist - Width ) / 2 ) * xSens;
    Boundaries(j,3) = MidPoint(1) - ( ( Dist - Width ) / 2 ) * xSens;
    Boundaries(j,2) = MidPoint(2) + ( ( Dist - Width ) / 2 ) * ySens;
    Boundaries(j,4) = MidPoint(2) - ( ( Dist - Width ) / 2 ) * ySens;
end

end

