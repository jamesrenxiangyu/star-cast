% compute the inter-vehilce
function dist_table = get_distTable(Locations)
% Input: Locations structure
% id, loc_x, loc_y, spd
% Output: dist_table structure
% layer_id, v_idx, v_loc, v_spd, dist

l = Locations;
layer_idx = unique(l(:,2));
num = length(layer_idx);
temp_table = cell(num,5);
dist_table = cell(num,5);
gap_th = 5; % thershold to merge vehicles

% initialize table
for i = 1:num
    % label each layer
    layer_flag = layer_idx(i);
    temp_table{i,1} = layer_flag;
    % find vehilces within the same layer
    v_idx = l(l(:,2) == layer_flag,1);
    temp_table{i,2} = v_idx;
    temp_loc_con = [];
    temp_spd_con = [];
    for j = 1:length(v_idx)
        temp_loc_con = [temp_loc_con; l(l(:,1) == v_idx(j), 2:3)];
        temp_spd_con = [temp_spd_con; l(l(:,1) == v_idx(j), 4)];
    end
    temp_table{i,3} = temp_loc_con;
    temp_table{i,4} = temp_spd_con;
end

% merge close layers
merge_flag = 0;
for i = 1:num-1
    if merge_flag == 0
        gap = temp_table{i+1,1} - temp_table{i,1};
        if gap > gap_th
            dist_table{i,1} = temp_table{i,1};
            dist_table{i,2} = temp_table{i,2};
            dist_table{i,3} = temp_table{i,3};
            dist_table{i,4} = temp_table{i,4};
        else
            % merge layer
            layer_flag_new = (temp_table{i+1,1} + temp_table{i,1})/2;
            dist_table{i,1} = layer_flag_new;
            dist_table{i,2} = [temp_table{i,2}; temp_table{i+1,2}];
            dist_table{i,3} = [temp_table{i,3}; temp_table{i+1,3}];
            dist_table{i,4} = [temp_table{i,4}; temp_table{i+1,4}];
            merge_flag = 1;
        end
    elseif merge_flag == 1
        merge_flag = 0;
    else
        disp('Error!');
    end
end
dist_table = dist_table(~cellfun(@isempty, dist_table(:,1)), :);

% compute distance between layers: minimum inter-vehicle distance
num_layers = size(dist_table,1);
for i = 1:num_layers-1
    num_v_pre = size(dist_table{i,2},1);
    num_v_lat = size(dist_table{i+1,2},1);
    
    % compute inter-vehicle distance for each vehicle in layer i and i+1
    dist_flag = 1000;
    for j = 1:num_v_pre
        v_pre_idx = dist_table{i,2}(j);
        v_pre_loc = l(l(:,1) == v_pre_idx, 2:3);
        v_pre_dis_con = zeros(num_v_lat,1);
        for k = 1:num_v_lat
            v_lat_idx = dist_table{i+1,2}(k);
            v_lat_loc = l(l(:,1) == v_lat_idx, 2:3);
            v_pre_dis_con(k) = norm(v_lat_loc - v_pre_loc);
        end
        % use minimum distance as per-vehicle inter-layer distance
        v_pre_dis = min(v_pre_dis_con);
        % use minimum distance as per-layer inter-layer distance
        dist_flag = min(v_pre_dis, dist_flag);
    end
    dist_table{i,5} = dist_flag;
end

dist_table{num_layers,5} = 0;


        
        
        
        
    













