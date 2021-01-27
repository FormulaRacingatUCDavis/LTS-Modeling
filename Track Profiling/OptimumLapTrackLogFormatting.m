%% Optimum Lap from Track Analysis Splines

v = 15; % Constant Speed [m/s]
f = 8;  % Sampling Rate [Hz]

Raw = [Spline.Distance, v*ones(size(Spline.Distance)) .* 3.6, v^2./Spline.Radius(:,1) ./ 9.81]; % [m, kmh, g]

[Sorted, Idx] = unique( Raw(:,1) );
Sorted(:,2) = Raw(Idx,2);
Sorted(:,3) = Raw(Idx,3);

Track = (0 : v/f : Sorted(end,1))';
Track(:,2) = interp1( Sorted(:,1), Sorted(:,2), Track(:,1) );
Track(:,3) = interp1( Sorted(:,1), Sorted(:,3), Track(:,1) );

figure
yyaxis left
scatter( Sorted(:,1), Sorted(:,2), '.' ); hold on;
plot( Track(:,1), Track(:,2) )

yyaxis right
scatter( Sorted(:,1), Sorted(:,3), '.' ); hold on;
plot( Track(:,1), Track(:,3) )

xlswrite('2016-19_Lincoln_Endurance_OL.xlsx', Track);