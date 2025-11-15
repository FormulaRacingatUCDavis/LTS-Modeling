function [v,tps,bps] = vehicle_model_lat(veh,r,pp_Ay)
     % speed solution
     if abs(r) < 1e-3 % At strait
         % max v given ax > 0
         v = max(veh.ggv(veh.ggv(:, 1) > 0, 3));
     else % Cornering
         r = abs(r);
         f = @(v) (v < 0)*1e6 + ppval(pp_Ay, v) .* 9.81 - v.^2 .* r;
         % disp("r=" + r)
         % opts = optimset('Display', 'iter', 'FunValCheck', 'on');
         v = fzero(f, 30);
     end
     tps = 0;
     bps = 0;
end