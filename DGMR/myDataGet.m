function [Location_x, Location_y, Speed] = myDataGet(csvfile, numVehicle)
opts = detectImportOptions(csvfile);
% select columns in the .csv file for process
% "time, v_id (type+id), speed, x_loc, y_loc"
opts.SelectedVariableNames = [1, 3, 7, 9, 10];
data = readtable(csvfile, opts);
%  dictionary to store vehicle id
types = ["V_id"]; 
flag = 0;
num_vehicle = numVehicle;

max_time = data{end,1};
Location_x = zeros(num_vehicle + 1, max_time + 1);
Location_y = zeros(num_vehicle + 1, max_time + 1);
Location_x(1, :) = (0:max_time);
Location_y(1, :) = (0:max_time);

Speed = zeros(num_vehicle + 1, max_time + 1);
Speed(1, :) = (0:max_time);

for i = 2: size(data,1)
    if isnan(data{i,3})
        continue;
    end
    tempTime = data{i,1}; % each timestep is 0.1s
    tempName = char(data{i,2});
    % parse vehicle name
    vflag = strfind(tempName, '.');
    if isempty(vflag)
        vtype = tempName;
        vid = 1;
    else
        vtype = tempName(1:vflag-1);
        vid = tempName(vflag+1:end);
        vid = str2double(vid);
    end
    switch vtype
        case "SlowCar"
            vid = 1000 + vid;
        case "Car"
            vid = 2000 + vid;
        case "FastCar"
            vid = 3000 + vid;
        case "Target"
            vid = vid  + 0;
        case "V_id"
            vid = 0;
        otherwise
            warning('Unexpected type!')
    end
    % Get vehicle id 
    flag = find(types == tempName); 
    if isempty(flag)
        % Create a position in array
        types = [types; tempName];
        flag = length(types);
        Location_x(flag, 1) = vid;
        Location_y(flag, 1) = vid;
        Speed(flag, 1) = vid;
    end
%     if data{i,3} >= 5e3
%         temp_loc = 0;
%         temp_spe = 0;
%     else
%         temp_loc = data{i,3};
%         temp_spe = data{i,4};
%     end
    temp_loc_x = data{i,4};
    temp_loc_y = data{i,5};
    temp_spe = data{i,3};
    Location_x(flag, tempTime + 1) = temp_loc_x;
    Location_y(flag, tempTime + 1) = temp_loc_y;
    Speed(flag, tempTime + 1) = temp_spe;
end

    
        
    
        


