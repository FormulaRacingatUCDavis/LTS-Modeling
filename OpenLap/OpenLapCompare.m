VehicleFiles = ["FE13_HDF_HB" "FE13_HDF_MB" "FE13_HDF_LB" "FE13_LDF_HB" "FE13_LDF_MB" "FE13_LDF_LB"];
sims = {};
track = "2008_FSUK_END";

using_feet = false;
%%% Loading the track
tr = load(track) ;
tr = tr.TrackInfo;
tr.info.name = track;
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

parfor i = 1:length(VehicleFiles)
    carParams = load(VehicleFiles(i)).carParams;
    carParams.Cl = carParams.Cl(0);
    GGV_data = load(VehicleFiles(i)).GGV_data;
    veh = carParams;
    ggv = GGV_data;
    veh.ggv = ggv;
    veh.name = "VehicleFiles(i)";

    simname = "OpenLAP Sims/OpenLAP_"+char(veh.name)+"_performance_score";
    logfile = simname+".log" ;
    [folder_status,folder_msg] = mkdir('OpenLAP Sims') ;
    delete(simname+".log") ;
    logid = fopen(logfile,'w') ;
    disp_logo(logid)
    disp('=================================================')
    disp("Vehicle: "+veh.name)
    disp("Track:   "+track)
    disp("Date:    "+datestr(now,'dd/mm/yyyy'))
    disp("Time:    "+datestr(now,'HH:MM:SS'))
    disp('=================================================')
    fprintf(logid,'%s\n','=================================================') ;
    fprintf(logid,'%s\n',"Vehicle: "+veh.name) ;
    fprintf(logid,'%s\n',"Track:   "+track) ;
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

    sims(i) = {OpenlapSim(veh,tr,simname,logid)};
end

% tiledlayout(3, 1)
% nexttile
figure;
hold on
for i = 1:length(VehicleFiles)
    plot(sims{i}.distance.data, sims{i}.speed.data)
end
xlabel("distance")
ylabel("speed")
legend(VehicleFiles)

% nexttile
% hold on
% for i = 1:length(VehicleFiles)
%     plot(sims{i}.distance.data, sims{i}.long_acc.data ./ 9.81)
% end
% xlabel("distance")
% ylabel("Ax")
% legend(VehicleFiles)
% 
% nexttile
% hold on
% for i = 1:length(VehicleFiles)
%     plot(sims{i}.distance.data, sims{i}.lat_acc.data ./ 9.81)
% end
% xlabel("distance")
% ylabel("Ay")
% legend(VehicleFiles)