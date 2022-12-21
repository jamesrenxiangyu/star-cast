% Cluster vehicles by inter-vehicle distance and kmeans
function [Num_Cluster, Cluster_mem, Cluster_head] = myClusterV3(locations, speed, time, distRef, covRef, mode)
%%
% Input:    locations: node locations
%           speed: node speed
%           time: time gap between each input
%           distRef: per-hop transmission distance reference
%           covRef: required cluster size 
%           mode: cluster strategy option used in myKMeans
% 
% Output:   Num_cluster: number of clusters (K)
%           Cluster_mem: cell, member locations and ids
%           Cluster_head: cell, CH locations and ids



l = locations(2:end, :); % n node locations array of size n x 3 (id, x, y),
s = speed(2,:); % speed of each node, put in n x 2 shape (id, spd),
src_loc = l(1,1); % assume first vehicle as source
d = distTable(l(1,:), src_loc); % compute the distance between adjacent vehicles
dmax = distRef; % maximum per-hop transmission distance
cmax = covRef; % maximum coverage of a cluster

% init paramters
fsrc = 1;  % source vehilce flag
fend = 1; % end vehilce flag
Num_Cluster = 0;
Cluster_ind = []; % final cluster member index
Cluster_loc = []; % final cluster member locations
Cluster_head = []; % final cluster member locations
CH_ind = []; % final cluster head index
% first-round cluster
clu_out_loc = [];
clu_out_spd = [];
% clu_in_ind = []; % second-round cluster
% clu_in_loc = [];


% divide network into subgroups by inter-vehicle distance
for i = 1: size(d,1)
    % reset container
    tempMem = zeros(2, size(l,2)+1);
    tempSpd = zeros(1, size(l,2)+1);
    if d(i) > dmax
        fend = i-1; % put the previous vehilce in the subgroup
        temp1 = l(:,fsrc:fend); % locations of vehilces
        temp2 = s(:,fsrc:fend); % speed of vehilces
        tempMem(:, 1:size(temp1,2)) = temp1;
        tempSpd(:, 1:size(temp1,2)) = temp2;
        clu_out_loc = [clu_out_loc; tempMem];
        clu_out_spd = [clu_out_spd; tempSpd];
        fsrc = i;
    end
    if i == size(d,1) && fsrc ~= i % put the last vehilce in the subgroup
        fend = i;
        temp1 = l(:, fsrc:fend);
        temp2 = s(:,fsrc:fend); % speed of vehilces
        tempMem(:, 1:size(temp1,1)) = temp1;
        tempSpd(:, 1:size(temp1,2)) = temp2;
        clu_out_loc = [clu_out_loc; tempMem];
        clu_out_spd = [clu_out_spd; tempSpd];
    end
end

Num_Cluster = size(clu_out_spd,1);

% cluster each subgroup by cluster coverage
for j = 1 : Num_Cluster
    flag1 = find(clu_out_loc(2*j-1, 2:end) == 0, 1); % edge of each subgroup
    tempLoc = clu_out_loc(2*j-1 : 2*j, 1:flag1);
    tempSpd = clu_out_spd(j, 1:flag1);
    if flag1 == 1 % if single member in a cluster
        tempMem = zeros(2, size(l,2)+1);
        tempMem(:,1) = tempLoc;
        Cluster_loc = [Cluster_loc; tempMem];
        Cluster_head = [Cluster_head, tempLoc];
        continue;
    end
    clu_cov = clu_out_loc(2*j-1, flag1) - clu_out_loc(2*j-1, 1);
    split_num = ceil(clu_cov/cmax);
    if split_num >= 1
        [cidx, chead] = myKMeans(split_num, tempLoc, tempSpd, time, mode);
        % cluster member locations container
        tempMem = zeros(2*split_num, size(l,2)+1);
        for c = 1:split_num
            mem = tempLoc(:, cidx == c);
            tempMem(2*c-1: 2*c, 1:size(mem,2)) = mem;
        end
        Cluster_loc = [Cluster_loc; tempMem];
        Cluster_head = [Cluster_head, chead];
    else
        warning("No members. Check your inputs")
    end
end

Num_Cluster = size(Cluster_head, 2);

