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
