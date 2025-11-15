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

% trackfile = 'OpenTRACK_FSAE Skidpad_Closed_Forward.mat' ;
trackfile = 'bluemax' ;
% trackfile = 'OpenTRACK_FSAE Skidpad_Closed_Forward.mat' ;
% trackfile = 'OpenTRACK_Paul Ricard_Closed_Forward.mat' ;
vehiclefile = 'FE12_max_aero' ;

%% Loading circuit

tr = load(trackfile) ;
tr = tr.TrackInfo;
tr.info.name = "blue max";
tr.info.config = "Closed";
tr.r = 1 ./ tr.r;
tr.dx = [tr.dx; 0.5] .* 0.3048;
tr.x = tr.x .* 0.3048;
tr.bank = zeros(size(tr.r));
tr.incl = zeros(size(tr.r));
tr.Z = zeros(size(tr.r));
tr.X = tr.coords(:, 1);
tr.Y = tr.coords(:, 2);

%% Loading car

veh = load(vehiclefile).carParams;
ggv = load(vehiclefile).GGV_data;
veh.ggv = ggv;
veh.name = "FE12";

v_max = max(ggv(:, 3));
ggv = [ggv; [zeros(10, 1), linspace(-2, 2, 10)', ones(10, 1)*(v_max+0.1)]];

mask = ggv(:, 1) >= 0;

v = ggv(mask, 3);
ay = ggv(mask, 2);
ax = ggv(mask, 1);

veh.max_drive = scatteredInterpolant(v, ay, ax, "linear", "nearest");

v = ggv(~mask, 3);
ay = ggv(~mask, 2);
ax = ggv(~mask, 1);

veh.max_brake = scatteredInterpolant(v, ay, ax, "linear", "nearest");

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

[sim] = OpenlapSim(veh,tr,simname,logid) ;

%% Displaying laptime

disp(['Laptime:  ',num2str(sim.laptime.data,'%3.3f'),' [s]'])
fprintf(logid,'%s','Laptime   : ') ;
fprintf(logid,'%7.3f',sim.laptime.data) ;
fprintf(logid,'%s\n',' [s]') ;
% for i=1:max(tr.sector)
%     disp(['Sector ',num2str(i),': ',num2str(sim.sector_time.data(i),'%3.3f'),' [s]'])
%     fprintf(logid,'%s','Sector ') ;
%     fprintf(logid,'%3d',i) ;
%     fprintf(logid,'%s',': ') ;
%     fprintf(logid,'%7.3f',sim.sector_time.data(i)) ;
%     fprintf(logid,'%s\n',' [s]') ;
% end

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
% subplot(rows,cols,[7,8])
% hold on
% plot(tr.x,sim.throttle.data*100)
% plot(tr.x,sim.brake_pres.data/10^5)
% legend({'tps','bps'},'Location',loc)
% xlabel('Distance [m]')
% xlim(xlimit)
% ylabel('input [%]')
% grid on
% ylim([-10,110])
% 
% % steering inputs
% subplot(rows,cols,[9,10])
% hold on
% plot(tr.x,sim.steering.data)
% plot(tr.x,sim.delta.data)
% plot(tr.x,sim.beta.data)
% legend({'Steering wheel','Steering \delta','Vehicle slip angle \beta'},'Location',loc)
% xlabel('Distance [m]')
% xlim(xlimit)
% ylabel('angle [deg]')
% grid on

% ggv circle
subplot(rows,cols,[11,13])
hold on
scatter3(sim.lat_acc.data,sim.long_acc.data,sim.speed.data*3.6,50,'ro','filled','MarkerEdgeColor',[0,0,0])
% surf(veh.GGV(:,:,2),veh.GGV(:,:,1),veh.GGV(:,:,3)*3.6,'EdgeAlpha',0.3,'FaceAlpha',0.8)
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
% plot(tr.arrow(:,1),tr.arrow(:,2),'k','LineWidth',2)
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
