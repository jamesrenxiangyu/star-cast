function input_data = get_myData(csvfile)
opts = detectImportOptions(csvfile);
% select columns in the .csv file for process
% "time, v_id (type+id), speed, x_loc, y_loc"
opts.SelectedVariableNames = [1, 3, 9, 10, 7];
data = readtable(csvfile, opts);


max_time = data{end,1};
title = {'v_id', 'loc_x', 'loc_y', 'spd'};
data_all = cell(max_time, 3);

for t = 1: max_time
    % get vehicle data at timeslot t
    tempData = data(data{:,1} == t, 2:end);
    tempData = table2cell(tempData);
    % find if target vehicle within the network
    flag = 0;
    for i = 1:size(tempData,1)
        tempName = char(tempData{i,1});
        % parse vehicle name
        vflag = strfind(tempName, '.');
        if isempty(vflag)
            vtype = tempName;
            vid = 1;
        else
            vtype = tempName(1:vflag-1);
            tempvid = tempName(vflag+1:end);
            tempvid = str2double(tempvid);
        end
        switch vtype
            case "SlowCar"
                vid = 1000 + tempvid;
            case "Car"
                vid = 2000 + tempvid;
            case "FastCar"
                vid = 3000 + tempvid;
            case "Target"
                vid = vid  + 0;
                flag = 1;
            case "V_id"
                vid = 0;
            otherwise
                warning('Unexpected type!')
        end
        tempData{i,1} = vid;
    end
    tempData = sortrows(tempData,2);
    tempData = [title; tempData];
    
    data_all{t,1} = t;
    data_all{t,2} = tempData;
    data_all{t,3} = flag;

end

input_data = data_all;
    
        
    
        



        
    
        


