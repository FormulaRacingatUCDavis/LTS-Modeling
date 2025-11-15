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
