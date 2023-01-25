% Cluster vehicles by inter-vehicle distance, cluster size constraints
function [Num_Cluster, Cluster_ind, Cluster_loc] = myCluster(locations,  distRef, covRef)
l = locations; % inter-vehicle distance
src_loc = l(1);
d = distTable(l, src_loc);
dmax = distRef; % maximum per-hop transmission distance
cmax = covRef; % maximum coverage of a cluster
fsrc = 1;
fend = 1;
Num_Cluster = 0;
Cluster_ind = [];
Cluster_loc = [];
clu_out_ind = [];
clu_out_loc = [];
clu_in_ind = [];
clu_in_loc = [];

tempMem = zeros(1, size(l,1)+1);
tempIndex = zeros(1, size(l,1)+1);

% cluster network by inter-vehicle distance
for i = 1: size(d,1)
    % reset container
    tempMem = zeros(1, size(l,1)+1);
    tempIndex = zeros(1, size(l,1)+1);
    if d(i) > dmax
        fend = i-1;
        tempIndex(1:fend - fsrc + 1) = (fsrc:fend);
        temp1 = l(fsrc:fend);
        tempMem(1:size(temp1,1)) = temp1;
        clu_out_ind = [clu_out_ind; tempIndex];
        clu_out_loc = [clu_out_loc; tempMem];
        fsrc = i;
    end
    if i == size(d,1) && fsrc ~= i
        fend = size(d,1);
        tempIndex(1:fend - fsrc + 1) = (fsrc:fend);
        temp1 = l(fsrc:fend);
        tempMem(1:size(temp1,1)) = temp1;
        clu_out_ind = [clu_out_ind; tempIndex];
        clu_out_loc = [clu_out_loc; tempMem];
    end
end

Num_Cluster = size(clu_out_ind,1);

% cluster each cluster by cluster coverage
for j = 1: Num_Cluster
    % reset container
    tempMem = zeros(1, size(l,1)+1);
    tempIndex = zeros(1, size(l,1)+1);
    flag1 = find(clu_out_loc(j,2:end) == 0, 1); % edge of outter-cluster
    clu_cov = clu_out_loc(j,flag1) - clu_out_loc(j, 1);
    split_num = ceil(clu_cov/cmax);
    if split_num > 1
        sp_loc = linspace(clu_out_loc(j, 1), clu_out_loc(j,flag1), split_num + 1);
        sp_loc = sp_loc(2:end-1);
        fsrc = 1;
        for k = 1:length(sp_loc)
            fend = find(clu_out_loc(j,2:end) > sp_loc(k), 1); % edge of inner-cluster
            flag2 = fend - fsrc + 1; % size of inner-cluster
            tempIndex(1:flag2) = clu_out_ind(j, fsrc:fend);
            tempMem(1:flag2) = clu_out_loc(j, fsrc:fend);
            Cluster_ind = [Cluster_ind; tempIndex];
            Cluster_loc = [Cluster_loc; tempMem];
            fsrc = fend + 1;
            % reset container
            tempMem = zeros(1, size(l,1)+1);
            tempIndex = zeros(1, size(l,1)+1);
            if k == length(sp_loc)
                flag2 = flag1 - fsrc + 1; % size of inner-cluster
                tempIndex(1:flag2) = clu_out_ind(j, fsrc:flag1);
                tempMem(1:flag2) = clu_out_loc(fsrc:flag1);
                Cluster_ind = [Cluster_ind; tempIndex];
                Cluster_loc = [Cluster_loc; tempMem];
                % reset container
                tempMem = zeros(1, size(l,1)+1);
                tempIndex = zeros(1, size(l,1)+1);
            end
        end
    else
        Cluster_ind = [Cluster_ind; clu_out_ind(j,:)];
        Cluster_loc = [Cluster_loc; clu_out_loc(j,:)];
    end
end

Num_Cluster = size(Cluster_ind,1);

                
            
        
                
                
            
            
        
        
            
            
        
    
    
    
    
    
    
        
        
        
        
        
    
