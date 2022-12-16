%% Organize csv.file to .mat file and sample data from dataset
clear; clc;

path = 'SUMO/';
cd(path)
csvfile = [path, 'mobility.csv'];
[Pos_x, Pos_y, Speed] = myDataGet(csvfile);
save('Locations.mat', 'Pos_x', 'Pos_y');
save('Speed.mat', 'Speed');


% sample data every 10s
sampRate = 10;

% Reserve data with target vehicle (id=1)
tar_id1 = find(Pos_x(:,1) == 1); % locate target vehicle
tar_id2 = find(Pos_x(:,1) ~= 0, 1, 'Last'); % count total vehicle number
tar_begin = find(Pos_x(tar_id1, 2:end) ~= 0, 1); % time target vehicle enters the network
tar_end = find(Pos_x(tar_id1, :) ~= 0, 1, 'Last'); % time target vehicle exits the network

% retrieve gloable network information over time involving target vehicle
loc_x = [Pos_x(1:tar_id1, 1) Pos_x(1:tar_id1, tar_begin:tar_end)];
loc_y = [Pos_y(1:tar_id1, 1) Pos_y(1:tar_id1, tar_begin:tar_end)];
mob = [Speed(1:tar_id1, 1) Speed(1:tar_id1, tar_begin:tar_end)]; 


Sloc_x = loc_x(:,1:sampRate:end);
Sloc_y = loc_y(:,1:sampRate:end);
Smob = mob(:,1:sampRate:end);
save('SampledLocations.mat', 'Sloc_x', 'Sloc_y');
save('SampledSpeed.mat', 'Smob');