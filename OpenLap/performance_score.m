clear; clc; close all;

% configs
using_feet = false;
vehicle_file = "FE13_HDF_MB";

set_baseline = false;
baseline = load("baselineFE13_HDF_HB.mat").baseline;

carParams = load(vehicle_file).carParams;
carParams.Cl = carParams.Cl(0);
GGV_data = load(vehicle_file).GGV_data;
veh = carParams;
ggv = GGV_data;
veh.ggv = ggv;
veh.name = "FE13";

tracks = ["2004_JPN_END" "2008_FSUK_END"];


% initialization
if set_baseline
    baseline = [];
end
sims = {};

simname = "OpenLAP Sims/OpenLAP_"+char(veh.name)+"_performance_score";
logfile = simname+".log" ;
[folder_status,folder_msg] = mkdir('OpenLAP Sims') ;
delete(simname+".log") ;
logid = fopen(logfile,'w') ;
disp_logo(logid)
disp('=================================================')
disp("Vehicle: "+veh.name)
disp("Track:   "+tracks)
disp("Date:    "+datestr(now,'dd/mm/yyyy'))
disp("Time:    "+datestr(now,'HH:MM:SS'))
disp('=================================================')
fprintf(logid,'%s\n','=================================================') ;
fprintf(logid,'%s\n',"Vehicle: "+veh.name) ;
fprintf(logid,'%s\n',"Track:   "+tracks) ;
fprintf(logid,'%s\n',"Date:    "+datestr(now,'dd/mm/yyyy')) ;
fprintf(logid,'%s\n',"Time:    "+datestr(now,'HH:MM:SS')) ;
fprintf(logid,'%s\n','=================================================') ;

%%% Load car
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

for i = 1:length(tracks)
    %%% Loading the track
    trackfile = tracks(i) ;
    tr = load(trackfile) ;
    tr = tr.TrackInfo;
    tr.info.name = tracks(i);
    tr.info.config = "Closed";
    
    % Some nessesary modifications if the track is created by the
    % RacingLineManualGeneration.m script
    
    % convert radias to curvature
    tr.r = 1 ./ tr.r;
    
    tr.dx = [tr.dx; 0.5];
    
    % The bluemax track is in feet, convert it to meters
    % The newer tracks are in meters
    if using_feet
        tr.dx = tr.dx .* 0.3048;
        tr.x = tr.x .* 0.3048;
    end
    
    tr.bank = zeros(size(tr.r));
    tr.incl = zeros(size(tr.r));
    tr.Z = zeros(size(tr.r));
    tr.X = tr.coords(:, 1);
    tr.Y = tr.coords(:, 2);

    %%% Run the sim
    sims(i) = {OpenlapSim(veh,tr,simname,logid)};

    if set_baseline
        baseline(i) = sims{i}.laptime.data;
    end
end

scores = zeros(length(sims), 1);
for i = 1:length(sims)
    laptime = sims{i}.laptime.data;
    disp(laptime)
    scores(i) = calc_autocross_score(laptime, baseline(i));
end

if set_baseline
    save("baseline"+vehicle_file, "baseline")
end

% Calculate the average score across all simulations
disp(scores)
averageScore = mean(scores);
disp(averageScore);

function score = calc_autocross_score(laptime, Tmin)
       Tmax = 1.45 * Tmin;
       score = 118.5 * ((Tmax/laptime) - 1) / ((Tmax/Tmin) - 1) + 6.5;
end