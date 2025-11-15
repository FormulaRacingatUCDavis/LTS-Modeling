function [v_next,ax,ay,tps,bps,overshoot] = vehicle_model_comb(veh,tr,v,v_max_next,j,mode)
    
    %% initialisation
    
    % assuming no overshoot
    overshoot = false ;
    % getting track data
    dx = tr.dx(j) ;
    r = tr.r(j) ;
    incl = tr.incl(j) ;
    bank = tr.bank(j) ;
    g = 9.81 ;
    
    %% current lat acc
    
    ay = v^2*r+g*sind(bank) ;

    %% overshoot acceleration
    
    % maximum allowed long acc to not overshoot at next point
    ax_max = mode*(v_max_next^2-v^2)/2/dx ;

    %% find ax

    if mode == 1
        ax = veh.max_drive(double(v), double(ay/g))*g;
        ax = min(ax_max, ax);

    else
        ax = veh.max_brake(double(v), double(ay/g))*g;
        ax = max(ax_max, ax);
    end

    %% final results
    
    tps = 0;
    bps = 0;

    % next speed value
    v_next = sqrt(v^2+2*mode*ax*tr.dx(j)) ;
    % correcting tps for full throttle when at v_max on straights
    % if tps>0 && v/veh.v_max>=0.999
    %     tps = 1 ;
    % end
    
    %% checking for overshoot
    
    % if v_next/v_max_next>1
    %     % setting overshoot flag
    %     overshoot = true ;
    %     % resetting values for overshoot
    %     v_next = inf ;
    %     ax = 0 ;
    %     ay = 0 ;
    %     tps = -1 ;
    %     bps = -1 ;
    %     return
    % end
    
end
