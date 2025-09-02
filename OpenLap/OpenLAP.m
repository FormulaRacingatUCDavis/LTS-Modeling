%% OpenLAP Laptime Simulation Project
%
% OpenLAP
%
% Lap time simulation using a simple point mass model for a racing vehicle.
% Instructions:
% 1) Select a vehicle file created by OpenVEHICLE by assigning the full
%    path to the variable "vehiclefile".
% 2) Select a track file created by OpenTRACK by assigning the full path to
%    the variable "trackfile".
% 3) Select an export frequency in [Hz] by setting the variable "freq" to
%    the desired value.
% 4) Run the script.
% 5) The results will appear on the command window and inside the folder
%    "OpenLAP Sims". You can choose to include the date and time of each
%    simulation in the result file name by changing the
%    "use_date_time_in_name" variable to true.
%
% More information can be found in the "OpenLAP Laptime Simulator"
% videos on YouTube.
%
% This software is licensed under the GPL V3 Open Source License.
%
% Open Source MATLAB project created by:
%
% Michael Halkiopoulos
% Cranfield University MSc Advanced Motorsport Engineer
% National Technical University of Athens MEng Mechanical Engineer
%
% LinkedIn: https://www.linkedin.com/in/michael-halkiopoulos/
% email: halkiopoulos_michalis@hotmail.com
% MATLAB file exchange: https://uk.mathworks.com/matlabcentral/fileexchange/
% GitHub: https://github.com/mc12027
%
% April 2020.

%% Clearing memory

clear
clc
% close all force
diary('off')
fclose('all') ;

%% Starting timer

tic

%% Filenames

trackfile = 'OpenTRACK_Paul Ricard_Closed_Forward.mat' ;
vehiclefile = 'OpenVEHICLE Vehicles/OpenVEHICLE_Formula 1_Open Wheel.mat' ;
ggv = load("GGV_Data.mat").dataPoints;

%% Loading circuit

tr = load(trackfile) ;

%% Loading car

% veh = load(vehiclefile) ;
% FE12 constants
% Chasis/suspension constants
carParams.m = 270;                        % Total Mass [kg]
carParams.PFront = 53.4/100;              % Percent Mass Front [0-1]
carParams.WB = 1.582;                     % Wheelbase [m]
carParams.TWf = 1.240;                    % Trackwidth [m]
carParams.TWr = 1.240;
carParams.toe_f = -0.5 * (pi/180);        % Toe Angles [radians] (positive is inwards)
carParams.toe_r = 0.5 * (pi/180);
carParams.hCG = 0.314;                    % CG height [m]
carParams.TireInclinationFront = -1.3; % deg 
carParams.TireInclinationRear = -1;    % deg   

% Aero constants
carParams.Cl = 3.215;
carParams.Cd = 1.468; 
carParams.CoP = 45/100;                   % front downforce distribution (%)
carParams.rho = 1.165;                    % kg/m^3
carParams.crossA = 0.9237;                % m^2

% braking system
carParams.B_FBB = 55/45;                    % Front brake bias

% Tire
tire = load('Hoosier_R20_16(18)x75(60)-10x8(7).mat');
tire.Idx = 1;                     % Moment of Inertia in x for wheel
tire.TirePressure = 70;           % kPa
tire.Model = struct( 'Pure', 'Pacejka', 'Combined', 'MNC' );
carParams.tire = tire;
veh = carParams;
veh.name = "FE12";

mask = ggv(3, :) > 0;

x = ggv(1, mask);
y = ggv(2, mask);
z = ggv(3, mask);

veh.max_drive = scatteredInterpolant(x', y', z', "linear", "nearest");

x = ggv(1, ~mask);
y = ggv(2, ~mask);
z = ggv(3, ~mask);

veh.max_brake = scatteredInterpolant(x', y', z', "linear", "nearest");

%% Export frequency

freq = 50 ; % [Hz]

%% Simulation name

use_date_time_in_name = false ;
if use_date_time_in_name
    date_time = "_"+datestr(now,'yyyy_mm_dd')+"_"+datestr(now,'HH_MM_SS') ; %#ok<UNRCH>
else
    date_time = "" ;
end
simname = "OpenLAP Sims/OpenLAP_"+char(veh.name)+"_"+tr.info.name+date_time ;
logfile = simname+".log" ;

%% HUD

[folder_status,folder_msg] = mkdir('OpenLAP Sims') ;
delete(simname+".log") ;
logid = fopen(logfile,'w') ;
disp_logo(logid)
disp('=================================================')
disp("Vehicle: "+veh.name)
disp("Track:   "+tr.info.name)
disp("Date:    "+datestr(now,'dd/mm/yyyy'))
disp("Time:    "+datestr(now,'HH:MM:SS'))
disp('=================================================')
fprintf(logid,'%s\n','=================================================') ;
fprintf(logid,'%s\n',"Vehicle: "+veh.name) ;
fprintf(logid,'%s\n',"Track:   "+tr.info.name) ;
fprintf(logid,'%s\n',"Date:    "+datestr(now,'dd/mm/yyyy')) ;
fprintf(logid,'%s\n',"Time:    "+datestr(now,'HH:MM:SS')) ;
fprintf(logid,'%s\n','=================================================') ;

%% Lap Simulation

[sim] = simulate(veh,tr,simname,logid) ;

%% Displaying laptime

disp(['Laptime:  ',num2str(sim.laptime.data,'%3.3f'),' [s]'])
fprintf(logid,'%s','Laptime   : ') ;
fprintf(logid,'%7.3f',sim.laptime.data) ;
fprintf(logid,'%s\n',' [s]') ;
for i=1:max(tr.sector)
    disp(['Sector ',num2str(i),': ',num2str(sim.sector_time.data(i),'%3.3f'),' [s]'])
    fprintf(logid,'%s','Sector ') ;
    fprintf(logid,'%3d',i) ;
    fprintf(logid,'%s',': ') ;
    fprintf(logid,'%7.3f',sim.sector_time.data(i)) ;
    fprintf(logid,'%s\n',' [s]') ;
end

%% Ploting results

% figure window
set(0,'units','pixels') ;
SS = get(0,'screensize') ;
H = 900-90 ;
W = 900 ;
Xpos = floor((SS(3)-W)/2) ;
Ypos = floor((SS(4)-H)/2) ;
f = figure('Name','OpenLAP Simulation Results','Position',[Xpos,Ypos,W,H]) ;
figname = ["OpenLAP: "+char(veh.name)+" @ "+tr.info.name,"Date & Time: "+datestr(now,'yyyy/mm/dd')+" "+datestr(now,'HH:MM:SS')] ;
sgtitle(figname)

% setting rows & columns
rows = 7 ;
cols = 2 ;
% x axis limits
xlimit = [tr.x(1),tr.x(end)] ;
% xlimit = [4000,4500] ;
% setting legend location
loc = 'east' ;

% speed
subplot(rows,cols,[1,2])
hold on
plot(tr.x,sim.speed.data*3.6)
legend({'Speed'},'Location',loc)
xlabel('Distance [m]')
xlim(xlimit)
ylabel('Speed [m/s]')
ylabel('Speed [km/h]')
grid on

% elevation and curvature
subplot(rows,cols,[3,4])
yyaxis left
plot(tr.x,tr.Z)
xlabel('Distance [m]')
xlim(xlimit)
ylabel('Elevation [m]')
grid on
yyaxis right
plot(tr.x,tr.r)
legend({'Elevation','Curvature'},'Location',loc)
ylabel('Curvature [m^-^1]')

% accelerations
subplot(rows,cols,[5,6])
hold on
plot(tr.x,sim.long_acc.data)
plot(tr.x,sim.lat_acc.data)
plot(tr.x,sim.sum_acc.data,'k:')
legend({'LonAcc','LatAcc','GSum'},'Location',loc)
xlabel('Distance [m]')
xlim(xlimit)
ylabel('Acceleration [m/s^2]')
grid on

% drive inputs
subplot(rows,cols,[7,8])
hold on
plot(tr.x,sim.throttle.data*100)
plot(tr.x,sim.brake_pres.data/10^5)
legend({'tps','bps'},'Location',loc)
xlabel('Distance [m]')
xlim(xlimit)
ylabel('input [%]')
grid on
ylim([-10,110])

% steering inputs
subplot(rows,cols,[9,10])
hold on
plot(tr.x,sim.steering.data)
plot(tr.x,sim.delta.data)
plot(tr.x,sim.beta.data)
legend({'Steering wheel','Steering \delta','Vehicle slip angle \beta'},'Location',loc)
xlabel('Distance [m]')
xlim(xlimit)
ylabel('angle [deg]')
grid on

% ggv circle
subplot(rows,cols,[11,13])
hold on
scatter3(sim.lat_acc.data,sim.long_acc.data,sim.speed.data*3.6,50,'ro','filled','MarkerEdgeColor',[0,0,0])
surf(veh.GGV(:,:,2),veh.GGV(:,:,1),veh.GGV(:,:,3)*3.6,'EdgeAlpha',0.3,'FaceAlpha',0.8)
legend('OpenLAP','GGV','Location','northeast')
xlabel('LatAcc [m/s^2]')
ylabel('LonAcc [m/s^2]')
zlabel('Speed [km/h]')
grid on
set(gca,'DataAspectRatio',[1 1 3])
axis tight

% track map
subplot(rows,cols,[12,14])
hold on
scatter(tr.X,tr.Y,5,sim.speed.data*3.6)
plot(tr.arrow(:,1),tr.arrow(:,2),'k','LineWidth',2)
legend('Track Map','Location','northeast')
xlabel('X [m]')
ylabel('Y [m]')
colorbar
grid on
axis equal

% saving figure
savefig(simname+".fig")

% HUD
disp('Plots created and saved.')
fprintf(logid,'%s\n','Plots created and saved.') ;

%% Report generation

% csv report generation
export_report(veh,tr,sim,freq,logid) ;
% saving .mat file
save(simname+".mat",'veh','tr','sim')
% HUD
toc
fprintf(logid,'%s','Elapsed time is: ') ;
fprintf(logid,'%f',toc) ;
fprintf(logid,'%s\n',' [s]') ;
fclose('all') ;

%% Functions

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [sim] = simulate(veh,tr,simname,logid)
    
    %% initialisation
    
    % solver timer
    timer_solver_start = tic ;
    
    % HUD
    disp('Simulation started.')
    fprintf(logid,'%s\n','Simulation started.') ;
    
    %% maximum speed curve (assuming pure lateral condition)

    dataPoints = load("GGV_Data.mat").dataPoints;
    mask = dataPoints(3, :) == 0 & dataPoints(2, :) > 0;
    
    v_samples = dataPoints(1, mask);
    Fy_samples = dataPoints(2, mask) .* 9.81 .* veh.m;
    
    %%

    pp_Fy = pchip(v_samples, Fy_samples);
    
    v_max = single(zeros(tr.n,1)) ;
    bps_v_max = single(zeros(tr.n,1)) ;
    tps_v_max = single(zeros(tr.n,1)) ;
    for i=1:tr.n
        [v_max(i),tps_v_max(i),bps_v_max(i)] = vehicle_model_lat(veh,tr.r(i),pp_Fy) ;
    end
    
    % HUD
    disp('Maximum speed calculated at all points.')
    fprintf(logid,'%s\n','Maximum speed calculated at all points.') ;
    
    %% finding apexes
    
    [v_apex,apex] = findpeaks(-v_max) ; % findpeaks works for maxima, so need to flip values
    v_apex = -v_apex ; % flipping to get positive values
    % setting up standing start for open track configuration
    if strcmp(tr.info.config,'Open')
        if apex(1)~=1 % if index 1 is not already an apex
            apex = [1;apex] ; % inject index 1 as apex
            v_apex = [0;v_apex] ; % inject standing start
        else % index 1 is already an apex
            v_apex(1) = 0 ; % set standing start at index 1
        end
    end
    % checking if no apexes found and adding one if needed
    if isempty(apex)
        [v_apex,apex] = min(v_max) ;
    end
    % reordering apexes for solver time optimisation
    apex_table = sortrows([v_apex,apex],1) ;
    v_apex = apex_table(:,1) ;
    apex = apex_table(:,2) ;
    % getting driver inputs at apexes
    tps_apex = tps_v_max(apex) ;
    bps_apex = bps_v_max(apex) ;
    
    % HUD
    disp('Found all apexes on track.')
    fprintf(logid,'%s\n','Found all apexes on track.') ;
    
    %% simulation
    
    % memory preallocation
    N = uint32((length(apex))) ; % number of apexes
    flag = false(tr.n,2) ; % flag for checking that speed has been correctly evaluated
    % 1st matrix dimension equal to number of points in track mesh
    % 2nd matrix dimension equal to number of apexes
    % 3rd matrix dimension equal to 2 if needed (1 copy for acceleration and 1 for deceleration)
    v = single(inf*ones(tr.n,N,2)) ;
    ax = single(zeros(tr.n,N,2)) ;
    ay = single(zeros(tr.n,N,2)) ;
    tps = single(zeros(tr.n,N,2)) ;
    bps = single(zeros(tr.n,N,2)) ;
    
    % HUD
    disp('Starting acceleration and deceleration.')
    fprintf(logid,'%s\n','Starting acceleration and deceleration.') ;
    prg_size = 30 ;
    prg_pos = ftell(logid) ;
    fprintf(['Running: [',repmat(' ',1,prg_size),'] '])
    fprintf('% 3.0f',0)
    fprintf(' [%%]')
    fprintf(logid,'%s',['Running: [',repmat(' ',1,prg_size),'] ']) ;
    fprintf(logid,'% 3.0f',0) ;
    fprintf(logid,'%s\n',' [%]') ;
    fprintf(logid,'________________________________________________\n') ;
    fprintf(logid,'|_Apex__|_Point_|_Mode__|___x___|___v___|_vmax_|\n') ;
    
    % running simulation
    for i=1:N % apex number
        for k=uint8(1:2) % mode number
            switch k
                case 1 % acceleration
                    mode = 1 ;
                    k_rest = 2 ;
                case 2 % deceleration
                    mode = -1 ;
                    k_rest = 1 ;
            end
            if ~(strcmp(tr.info.config,'Open') && mode==-1 && i==1) % does not run in decel mode at standing start in open track
                % getting other apex for later checking
                [i_rest] = other_points(i,N) ;
                if isempty(i_rest)
                    i_rest = i ;
                end
                % getting apex index
                j = uint32(apex(i)) ;
                % saving speed & latacc & driver inputs from presolved apex
                v(j,i,k) = v_apex(i) ;
                ay(j,i,k) = v_apex(i)^2*tr.r(j) ;
                tps(j,:,1) = tps_apex(i)*ones(1,N) ;
                bps(j,:,1) = bps_apex(i)*ones(1,N) ;
                tps(j,:,2) = tps_apex(i)*ones(1,N) ;
                bps(j,:,2) = bps_apex(i)*ones(1,N) ;
                % setting apex flag
                flag(j,k) = true ;
                % getting next point index
                [~,j_next] = next_point(j,tr.n,mode,tr.info.config) ;
                if ~(strcmp(tr.info.config,'Open') && mode==1 && i==1) % if not in standing start
                    % assuming same speed right after apex
                    v(j_next,i,k) = v(j,i,k) ;
                    % moving to next point index
                    [j_next,j] = next_point(j,tr.n,mode,tr.info.config) ;
                end
                while 1
                    % writing to log file
                    fprintf(logid,'%7d\t%7d\t%7d\t%7.1f\t%7.2f\t%7.2f\n',i,j,k,tr.x(j),v(j,i,k),v_max(j)) ;
                    % calculating speed, accelerations and driver inputs from vehicle model
                    [v(j_next,i,k),ax(j,i,k),ay(j,i,k),tps(j,i,k),bps(j,i,k),overshoot] = vehicle_model_comb(veh,tr,v(j,i,k),v_max(j_next),j,mode) ;
                    % checking for limit
                    if overshoot
                        break
                    end
                    % checking if point is already solved in other apex iteration
                    if flag(j,k) || flag(j,k_rest)
                        if max(v(j_next,i,k)>=v(j_next,i_rest,k)) || max(v(j_next,i,k)>v(j_next,i_rest,k_rest))
                            break
                        end
                    end
                    % updating flag and grogress bar
                    flag = flag_update(flag,j,k,prg_size,logid,prg_pos) ;
                    % moving to next point index
                    [j_next,j] = next_point(j,tr.n,mode,tr.info.config) ;
                    % checking if lap is completed
                    switch tr.info.config
                        case 'Closed'
                            if j==apex(i) % made it to the same apex
                                break
                            end
                        case 'Open'
                            if j==tr.n % made it to the end
                                flag = flag_update(flag,j,k,prg_size,logid,prg_pos) ;
                                break
                            end
                            if j==1 % made it to the start
                                break
                            end
                    end
                end
            end
        end
    end
    
    % HUD
    progress_bar(max(flag,[],2),prg_size,logid,prg_pos) ;
    fprintf('\n')
    disp('Velocity profile calculated.')
    disp(['Solver time is: ',num2str(toc(timer_solver_start)),' [s]']) ;
    disp('Post-processing initialised.')
    fprintf(logid,'________________________________________________\n') ;
    if sum(flag)<size(flag,1)/size(flag,2)
        fprintf(logid,'%s\n','Velocity profile calculation error.') ;
        fprintf(logid,'%s\n','Points not calculated.') ;
        p = (1:tr.n)' ;
        fprintf(logid,'%d\n',p(min(flag,[],2))) ;
    else
        fprintf(logid,'%s\n','Velocity profile calculated successfully.') ;
    end
    fprintf(logid,'%s','Solver time is: ') ;
    fprintf(logid,'%f',toc(timer_solver_start)) ;
    fprintf(logid,'%s\n',' [s]') ;
    fprintf(logid,'%s\n','Post-processing initialised.') ;
    
    %% post-processing resutls
    
    % result preallocation
    V = zeros(tr.n,1) ;
    AX = zeros(tr.n,1) ;
    AY = zeros(tr.n,1) ;
    TPS = zeros(tr.n,1) ;
    BPS = zeros(tr.n,1) ;
    % solution selection
    for i=1:tr.n
        IDX = length(v(i,:,1)) ;
        [V(i),idx] = min([v(i,:,1),v(i,:,2)]) ; % order of k in v(i,:,k) inside min() must be the same as mode order to not miss correct values
        if idx<=IDX % solved in acceleration
            AX(i) = ax(i,idx,1) ;
            AY(i) = ay(i,idx,1) ;
            TPS(i) = tps(i,idx,1) ;
            BPS(i) = bps(i,idx,1) ;
        else % solved in deceleration
            AX(i) = ax(i,idx-IDX,2) ;
            AY(i) = ay(i,idx-IDX,2) ;
            TPS(i) = tps(i,idx-IDX,2) ;
            BPS(i) = bps(i,idx-IDX,2) ;
        end
    end
    % HUD
    disp('Correct solution selected from modes.')
    fprintf(logid,'%s\n','Correct solution selected from modes.') ;
    
    % laptime calculation
    if strcmp(tr.info.config,'Open')
        time = cumsum([tr.dx(2)./V(2);tr.dx(2:end)./V(2:end)]) ;
    else
        time = cumsum(tr.dx./V) ;
    end
    sector_time = zeros(max(tr.sector),1) ;
    for i=1:max(tr.sector)
        sector_time(i) = max(time(tr.sector==i))-min(time(tr.sector==i)) ;
    end
    laptime = time(end) ;
    % HUD
    disp('Laptime calculated.')
    fprintf(logid,'%s\n','Laptime calculated.') ;
    
    %%
    % calculating forces
    M = veh.m ;
    g = 9.81 ;
    A = sqrt(AX.^2+AY.^2) ;
    Fz_mass = -M*g*cosd(tr.bank).*cosd(tr.incl) ;
    Fz_aero = 1/2*veh.rho*veh.factor_Cl*veh.Cl*veh.A*V.^2 ;
    Fz_total = Fz_mass+Fz_aero ;
    Fx_aero = 1/2*veh.rho*veh.factor_Cd*veh.Cd*veh.A*V.^2 ;
    Fx_roll = veh.Cr*abs(Fz_total) ;
    % HUD
    disp('Forces calculated.')
    fprintf(logid,'%s\n','Forces calculated.') ;

    % calculating yaw motion, vehicle slip angle and steering input
    yaw_rate = V.*tr.r ;
    delta = zeros(tr.n,1) ;
    beta = zeros(tr.n,1) ;
    for i=1:tr.n
        B = [M*V(i)^2*tr.r(i)+M*g*sind(tr.bank(i));0] ;
        sol = veh.C\B ;
        delta(i) = sol(1)+atand(veh.L*tr.r(i)) ;
        beta(i) = sol(2) ;
    end
    steer = delta*veh.rack ;
    % HUD
    disp('Yaw motion calculated.')
    disp('Steering angles calculated.')
    disp('Vehicle slip angles calculated.')
    fprintf(logid,'%s\n','Yaw motion calculated.') ;
    fprintf(logid,'%s\n','Steering angles calculated.') ;
    fprintf(logid,'%s\n','Vehicle slip angles calculated.') ;

    % calculating engine metrics
    wheel_torque = TPS.*interp1(veh.vehicle_speed,veh.wheel_torque,V,'linear','extrap') ;
    Fx_eng = wheel_torque/veh.tyre_radius ;
    engine_torque = TPS.*interp1(veh.vehicle_speed,veh.engine_torque,V,'linear','extrap') ;
    engine_power = TPS.*interp1(veh.vehicle_speed,veh.engine_power,V,'linear','extrap') ;
    engine_speed = interp1(veh.vehicle_speed,veh.engine_speed,V,'linear','extrap') ;
    gear = interp1(veh.vehicle_speed,veh.gear,V,'nearest','extrap') ;
    fuel_cons = cumsum(wheel_torque/veh.tyre_radius.*tr.dx/veh.n_primary/veh.n_gearbox/veh.n_final/veh.n_thermal/veh.fuel_LHV) ;
    fuel_cons_total = fuel_cons(end) ;
    % HUD
    disp('Engine metrics calculated.')
    fprintf(logid,'%s\n','Engine metrics calculated.') ;

    % calculating kpis
    percent_in_corners = sum(tr.r~=0)/tr.n*100 ;
    percent_in_accel = sum(TPS>0)/tr.n*100 ;
    percent_in_decel = sum(BPS>0)/tr.n*100 ;
    percent_in_coast = sum(and(BPS==0,TPS==0))/tr.n*100 ;
    percent_in_full_tps = sum(tps==1)/tr.n*100 ;
    percent_in_gear = zeros(veh.nog,1) ;
    for i=1:veh.nog
        percent_in_gear(i) = sum(gear==i)/tr.n*100 ;
    end
    energy_spent_fuel = fuel_cons*veh.fuel_LHV ;
    energy_spent_mech = energy_spent_fuel*veh.n_thermal ;
    gear_shifts = sum(abs(diff(gear))) ;
    [~,i] = max(abs(AY)) ;
    ay_max = AY(i) ;
    ax_max = max(AX) ;
    ax_min = min(AX) ;
    sector_v_max = zeros(max(tr.sector),1) ;
    sector_v_min = zeros(max(tr.sector),1) ;
    for i=1:max(tr.sector)
        sector_v_max(i) = max(V(tr.sector==i)) ;
        sector_v_min(i) = min(V(tr.sector==i)) ;
    end
    % HUD
    disp('KPIs calculated.')
    disp('Post-processing finished.')
    fprintf(logid,'%s\n','KPIs calculated.') ;
    fprintf(logid,'%s\n','Post-processing finished.') ;
    
    %% saving results in sim structure
    sim.sim_name.data = simname ;
    sim.distance.data = tr.x ;
    sim.distance.unit = 'm' ;
    sim.time.data = time ;
    sim.time.unit = 's' ;
    sim.N.data = N ;
    sim.N.unit = [] ;
    sim.apex.data = apex ;
    sim.apex.unit = [] ;
    sim.speed_max.data = v_max ;
    sim.speed_max.unit = 'm/s' ;
    sim.flag.data = flag ;
    sim.flag.unit = [] ;
    sim.v.data = v ;
    sim.v.unit = 'm/s' ;
    sim.Ax.data = ax ;
    sim.Ax.unit = 'm/s/s' ;
    sim.Ay.data = ay ;
    sim.Ay.unit = 'm/s/s' ;
    sim.tps.data = tps ;
    sim.tps.unit = [] ;
    sim.bps.data = bps ;
    sim.bps.unit = [] ;
    sim.elevation.data = tr.Z ;
    sim.elevation.unit = 'm' ;
    sim.speed.data = V ;
    sim.speed.unit = 'm/s' ;
    % sim.yaw_rate.data = yaw_rate ;
    % sim.yaw_rate.unit = 'rad/s' ;
    sim.long_acc.data = AX ;
    sim.long_acc.unit = 'm/s/s' ;
    sim.lat_acc.data = AY ;
    sim.lat_acc.unit = 'm/s/s' ;
    sim.sum_acc.data = A ;
    sim.sum_acc.unit = 'm/s/s' ;
    sim.throttle.data = TPS ;
    sim.throttle.unit = 'ratio' ;
    sim.brake_pres.data = BPS ;
    sim.brake_pres.unit = 'Pa' ;
    sim.brake_force.data = BPS*veh.phi ;
    sim.brake_force.unit = 'N' ;
    sim.steering.data = steer ;
    sim.steering.unit = 'deg' ;
    sim.delta.data = delta ;
    sim.delta.unit = 'deg' ;
    sim.beta.data = beta ;
    sim.beta.unit = 'deg' ;
    sim.Fz_aero.data = Fz_aero ;
    sim.Fz_aero.unit = 'N' ;
    sim.Fx_aero.data = Fx_aero ;
    sim.Fx_aero.unit = 'N' ;
    sim.Fx_eng.data = Fx_eng ;
    sim.Fx_eng.unit = 'N' ;
    sim.Fx_roll.data = Fx_roll ;
    sim.Fx_roll.unit = 'N' ;
    sim.Fz_mass.data = Fz_mass ;
    sim.Fz_mass.unit = 'N' ;
    sim.Fz_total.data = Fz_total ;
    sim.Fz_total.unit = 'N' ;
    sim.wheel_torque.data = wheel_torque ;
    sim.wheel_torque.unit = 'N.m' ;
    sim.engine_torque.data = engine_torque ;
    sim.engine_torque.unit = 'N.m' ;
    sim.engine_power.data = engine_power ;
    sim.engine_power.unit = 'W' ;
    sim.engine_speed.data = engine_speed ;
    sim.engine_speed.unit = 'rpm' ;
    sim.gear.data = gear ;
    sim.gear.unit = [] ;
    sim.fuel_cons.data = fuel_cons ;
    sim.fuel_cons.unit = 'kg' ;
    sim.fuel_cons_total.data = fuel_cons_total ;
    sim.fuel_cons_total.unit = 'kg' ;
    sim.laptime.data = laptime ;
    sim.laptime.unit = 's' ;
    sim.sector_time.data = sector_time ;
    sim.sector_time.unit = 's' ;
    sim.percent_in_corners.data = percent_in_corners ;
    sim.percent_in_corners.unit = '%' ;
    sim.percent_in_accel.data = percent_in_accel ;
    sim.percent_in_accel.unit = '%' ;
    sim.percent_in_decel.data = percent_in_decel ;
    sim.percent_in_decel.unit = '%' ;
    sim.percent_in_coast.data = percent_in_coast ;
    sim.percent_in_coast.unit = '%' ;
    sim.percent_in_full_tps.data = percent_in_full_tps ;
    sim.percent_in_full_tps.unit = '%' ;
    sim.percent_in_gear.data = percent_in_gear ;
    sim.percent_in_gear.unit = '%' ;
    sim.v_min.data = min(V) ;
    sim.v_min.unit = 'm/s' ;
    sim.v_max.data = max(V) ;
    sim.v_max.unit = 'm/s' ;
    sim.v_ave.data = mean(V) ;
    sim.v_ave.unit = 'm/s' ;
    sim.energy_spent_fuel.data = energy_spent_fuel ;
    sim.energy_spent_fuel.unit = 'J' ;
    sim.energy_spent_mech.data = energy_spent_mech ;
    sim.energy_spent_mech.unit = 'J' ;
    sim.gear_shifts.data = gear_shifts ;
    sim.gear_shifts.unit = [] ;
    sim.lat_acc_max.data = ay_max ;
    sim.lat_acc_max.unit = 'm/s/s' ;
    sim.long_acc_max.data = ax_max ;
    sim.long_acc_max.unit = 'm/s/s' ;
    sim.long_acc_min.data = ax_min ;
    sim.long_acc_min.unit = 'm/s/s' ;
    sim.sector_v_max.data = sector_v_max ;
    sim.sector_v_max.unit = 'm/s' ;
    sim.sector_v_min.data = sector_v_min ;
    sim.sector_v_min.unit = 'm/s' ;
    % HUD
    disp('Simulation results saved.')
    disp('Simulation completed.')
    fprintf(logid,'%s\n','Simulation results saved.') ;
    fprintf(logid,'%s\n','Simulation completed.') ;
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [v,tps,bps] = vehicle_model_lat(veh,r,pp_Fy)
     % speed solution
     if false%abs(r) < 1e-6 % At strait
         v = inf;
     else % Cornering
         r = abs(r);
         f = @(v) (v < 5)*1e6 + ppval(pp_Fy, v) - veh.m*r*v.^2;
         disp("r=" + r)
         opts = optimset('Display', 'iter', 'FunValCheck', 'on');
         v = fzero(f, 50);
     tps = 0;
     bps = 0;
end
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

    % ax = interpulateGGV(veh.GGV, v, ay);
    % [ax, ~, ~] = vehicle_model_comb_test(veh, v, ay, mode==1, [0 0 0]);
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
    
    if v_next/v_max_next>1
        % setting overshoot flag
        overshoot = true ;
        % resetting values for overshoot
        v_next = inf ;
        ax = 0 ;
        ay = 0 ;
        tps = -1 ;
        bps = -1 ;
        return
    end
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [j_next,j] = next_point(j,j_max,mode,tr_config)
    switch mode
        case 1 % acceleration
            switch tr_config
                case 'Closed'
                    if j==j_max-1
                        j = j_max ;
                        j_next = 1 ;
                    elseif j==j_max
                        j = 1 ;
                        j_next = j+1 ;
                    else
                        j = j+1 ;
                        j_next = j+1 ;
                    end
                case 'Open'
                    j = j+1 ;
                    j_next = j+1 ;
            end
        case -1 % deceleration
            switch tr_config
                case 'Closed'
                    if j==2
                        j = 1 ;
                        j_next = j_max ;
                    elseif j==1
                        j = j_max ;
                        j_next = j-1 ;
                    else
                        j = j-1 ;
                        j_next = j-1 ;
                    end
                case 'Open'
                    j = j-1 ;
                    j_next = j-1 ;
            end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [i_rest] = other_points(i,i_max)
    i_rest = (1:i_max)' ;
    i_rest(i) = [] ;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [flag] = flag_update(flag,j,k,prg_size,logid,prg_pos)
    % current flag state
    p = sum(flag,'all')/size(flag,1)/size(flag,2) ;
    n_old = floor(p*prg_size) ; % old number of lines
    % new flag state
    flag(j,k) = true ;
    p = sum(flag,'all')/size(flag,1)/size(flag,2) ;
    n = floor(p*prg_size) ; % new number of lines
    % checking if state has changed enough to update progress bar
    if n>n_old
        progress_bar(flag,prg_size,logid,prg_pos) ;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = progress_bar(flag,prg_size,logid,prg_pos)
    % current flag state
    p = sum(flag,'all')/size(flag,1)/size(flag,2) ; % progress percentage
    n = floor(p*prg_size) ; % new number of lines
    e = prg_size-n ; % number of spaces
    % updating progress bar in command window
    fprintf(repmat('\b',1,prg_size+1+8)) % backspace to start of bar
    fprintf(repmat('|',1,n)) % writing lines
    fprintf(repmat(' ',1,e)) % writing spaces
    fprintf(']') % closing bar
    fprintf('%4.0f',p*100) % writing percentage
    fprintf(' [%%]') % writing % symbol
    % updating progress bar in log file
    fseek(logid,prg_pos,'bof') ; % start of progress bar position in log file
    fprintf(logid,'%s','Running: [') ;
    fprintf(logid,'%s',repmat('|',1,n)) ;
    fprintf(logid,'%s',repmat(' ',1,e)) ;
    fprintf(logid,'%s','] ') ;
    fprintf(logid,'%3.0f',p*100) ;
    fprintf(logid,'%s\n',' [%]') ;
    fseek(logid,0,'eof') ; % continue at end of file
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = disp_logo(logid)
    lg = [...
        '_______                    _____________________ ';...
        '__  __ \______________________  /___    |__  __ \';...
        '_  / / /__  __ \  _ \_  __ \_  / __  /| |_  /_/ /';...
        '/ /_/ /__  /_/ /  __/  / / /  /___  ___ |  ____/ ';...
        '\____/ _  .___/\___//_/ /_//_____/_/  |_/_/      ';...
        '       /_/                                       '...
        ] ;
    disp(lg) % command window
    fprintf(logid,'%s',[lg,repmat(newline,size(lg,1),1)].') ; % log file
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = export_report(veh,tr,sim,freq,logid)
    % frequency
    freq = round(freq) ;
    % channel names
    all_names = fieldnames(sim) ;
    % number of channels to export
    S = 0 ;
    % channel id vector
    I = (1:length(all_names))' ;
    % getting only vector channels (excluding matrices)
    for i=1:length(all_names)
        % getting size for each channel
        s = size(eval(['sim.',all_names{i},'.data'])) ;
        % checking if channel is a vector
        if length(s)==2 && s(1)==tr.n && s(2)==1 % is vector
            S = S+1 ;
        else % is not vector
            I(i) = 0 ;
        end
    end
    % keeping only vector channel ids
    I(I==0) = [] ;
    % getting channel names
    channel_names = all_names(I)' ;
    % memory preallocation
    % data matrix
    data = single(zeros(tr.n,S)) ;
    % units vector
    channel_units = cell(1,length(I)) ;
    % getting data and units
    for i=1:length(I)
        data(:,i) = eval(['sim.',all_names{I(i)},'.data']) ;
        channel_units(i) = eval(['{sim.',all_names{I(i)},'.unit}']) ;
    end
    % new time vector for specified frequency
    t = (0:1/freq:sim.laptime.data)' ;
    % getting time channel id vector
    j = strcmp(string(channel_names),"time") ;
    % time data memory preallocation
    time_data = single(zeros(length(t),length(I))) ;
    % getting 
    for i=1:length(I)
         % checking if channel corresponds to time
        if i==j % time channel
            time_data(:,i) = t ;
        else % all other channels
            % checking for integer channel
            if strcmp(string(channel_names(i)),"gear") % gear needs to be integer
                time_data(:,i) = interp1(data(:,j),data(:,i),t,'nearest','extrap') ;
            else % all other channels are linearly interpolated
                time_data(:,i) = interp1(data(:,j),data(:,i),t,'linear','extrap') ;
            end
        end
    end
    % opening and writing .csv file
    % HUD
    disp('Export initialised.')
    fprintf(logid,'%s\n','Export initialised.') ;
    % filename
    filename = sim.sim_name.data+".csv" ;
    % opening file
    fid = fopen(filename,'w') ;
    % writing file header
    fprintf(fid,'%s,%s\n',["Format","OpenLAP Export"]) ;
    fprintf(fid,'%s,%s\n',["Venue",tr.info.name]) ;
    fprintf(fid,'%s,%s\n',["Vehicle",veh.name]) ;
    fprintf(fid,'%s,%s\n',["Driver",'OpenLap']) ;
    fprintf(fid,'%s\n',"Device") ;
    fprintf(fid,'%s\n',"Comment") ;
    fprintf(fid,'%s,%s\n',["Date",datestr(now,'dd/mm/yyyy')]) ;
    fprintf(fid,'%s,%s\n',["Time",datestr(now,'HH:MM:SS')]) ;
    fprintf(fid,'%s,%s\n',["Frequency",num2str(freq,'%d')]) ;
    fprintf(fid,'\n') ;
    fprintf(fid,'\n') ;
    fprintf(fid,'\n') ;
    fprintf(fid,'\n') ;
    fprintf(fid,'\n') ;
    % writing channels
    form = [repmat('%s,',1,length(I)-1),'%s\n'] ;
    fprintf(fid,form,channel_names{:}) ;
    fprintf(fid,form,channel_names{:}) ;
    fprintf(fid,form,channel_units{:}) ;
    fprintf(fid,'\n') ;
    fprintf(fid,'\n') ;
    form = [repmat('%f,',1,length(I)-1),'%f\n'] ;
    for i=1:length(t)
        fprintf(fid,form,time_data(i,:)) ;
    end
    % closing file
    fclose(fid) ;
    % HUD
    disp('Exported .csv file successfully.')
    fprintf(logid,'%s\n','Exported .csv file successfully.') ;
end
