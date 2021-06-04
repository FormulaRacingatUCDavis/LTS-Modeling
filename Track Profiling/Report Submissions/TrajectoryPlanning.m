width = 1.5;
Boundaries = TrackWidth(Boundaries.Points, width);
[PathSpline, ControlPoints] = Pathing(Boundaries);
Track = CurvatureTrace(PathSpline,Image);