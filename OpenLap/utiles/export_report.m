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
