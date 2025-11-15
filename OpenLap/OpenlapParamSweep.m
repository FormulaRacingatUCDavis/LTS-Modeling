clear; clc; close all;

m = 1:10;
motor_pwr = 1:10;
carParamSweep = repmat(Cars.FE12, size(m));

for i = 1:length(m)
    carParamSweep(i).m = m(i);
    carParamSweep(1).drivetrain.Power = motor_pwr(i);
end

V = linspace(7, 40, 2);
%%

for i = 1:length(m)
    carParams = carParamSweep(i);
    mmd = MMD.MMD(carParams);
    GGV_data = generate_GGV(mmd, V);

    save("LTS_Sweep_GGV_Data" + i, "GGV_data", "carParams")
end
    
%% Loading circuit

trackfile = 'bluemax' ;
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
    
for i = 1:length(m)
    tic
    
    vehiclefile = "LTS_Sweep_GGV_Data" + i ;
    
    %% Loading car
    veh = load(vehiclefile).carParams;
    ggv = load(vehiclefile).GGV_data;
    veh.ggv = ggv;
    veh.name = "FE12" + i;
    
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
    save("LTS_param_sweep_sim_" + i)
end