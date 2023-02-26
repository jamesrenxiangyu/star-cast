% Cluster vehicles by inter-vehicle distance and kmeans
function [num_Cluster, Cluster] = get_Cluster(locTable, time, distRef, covRef, mode, src_id)
%%
% Input:    locTable: node locations, speed
%           time: time gap between each input
%           distRef: per-hop transmission distance reference i.e., dmin
%           covRef: required cluster size 
%           mode: cluster strategy option used in myKMeans
%           src_id: v_id of source vehicle
% 
% Output:   Num_cluster: number of clusters (K)
%           Cluster: Include locations, speeds, and indices of cluster memebers and
%           specify CH. In the form: 'Cluster; CH'; CH has the form: v_id,
%           v_loc, v_spd;

temp_cluster = cell(size(locTable,1),2);
Cluster = cell(1,2);
Cluster{1,1} = 'Cluster';
Cluster{1,2} = 'CH';

if size(temp_cluster,1) == 1
    Cluster{2,1} = locTable;
    Cluster{2,2} = locTable;
else
% first-round cluster: divide by inter-layer distance
inter_dist = cell2mat(locTable(:,end));
split_flag_out = find(inter_dist >= distRef);
pre_flag = 1;
if ~isempty(split_flag_out)
    for i = 1:length(split_flag_out)
        temp_cluster{i,1} = locTable(pre_flag:split_flag_out(i),:);
        pre_flag = split_flag_out(i) + 1;
        if i == length(split_flag_out)
            temp_cluster{i+1,1} = locTable(pre_flag:end,:);
            break
        end
    end
end
temp_cluster = temp_cluster(~cellfun(@isempty, temp_cluster(:,1)), :);

num_Cluster = size(Cluster,1);
num_temp_cluster = size(temp_cluster,1);
src_flag = 0;

% second-round cluster: cluster each subgroup by cluster coverage
count = 0;
for i = 1:num_temp_cluster
    % compute size of subgroup
    temp_cluster_cov = sum(cell2mat(temp_cluster{i,1}(1:end-1,end)));
    % organize subgroup data:
    % vid, loc_x, loc_y, spd
    subgroup_data = cell2mat(temp_cluster{i,1}(:,2:end-1));
    % cluster subgroup
    if temp_cluster_cov > covRef
        split_num = ceil(temp_cluster_cov/covRef);
        cluster = get_myKMeans(split_num, subgroup_data, time, mode);
        Cluster = [Cluster; cluster];
    else
        split_num = 1;
        cluster = get_myKMeans(split_num, subgroup_data, time, mode);
        if src_flag == 0
        for k = 1:size(cluster,1)
            % replace cluster head by the src vehicle if exists
            if ~isempty(find(cluster{k,1} == src_id, 1))
                src_data = cluster{k,1}(cluster{k,1} == src_id, :);
                cluster{k,2} = src_data;
                src_flag = 1;
            end        
        end
        end
        Cluster = [Cluster; cluster];
    end
    
end
end

num_Cluster = size(Cluster, 1);

