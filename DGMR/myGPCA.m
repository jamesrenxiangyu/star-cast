%% GPCA part
function [Cluster_idx, CH_idx, loc_con, rel_con, del_con, cos_con, idx_con] = myGPCA(loc, nCluster, Cluster_loc, Cluster_head, refDis, Ch_par, QoS_th)

DEL_th = QoS_th(5);
RAT_th = QoS_th(2);
Tau = Ch_par(9);
Pkt = Ch_par(8);


%% Cluster network 
% init cluster member and cluster head indices
% Cluster_idx: member index, array of size K x N
Cluster_idx = zeros(nCluster, size(Cluster_loc,2));
% CH_idx: CH index, array of size K x 1
CH_idx = zeros(nCluster, 1);


%% Routing algorithm
% Set relay location container and performance metrics container
NUM = nCluster; % total number of clusters
HOP_MAX = ceil(DEL_th/Tau); % max number of hops per cluster
loc_con = zeros(2*NUM, HOP_MAX); % store lcoations of each relay per cluster
rel_con = zeros(NUM, 1); % store e2e reliability
del_con = zeros(NUM, 1); % store e2e delay
cos_con = zeros(NUM, 1); % store e2e cost
idx_con = zeros(NUM, HOP_MAX); % store vehilce index of each relay per cluster
rat_con = zeros(NUM, 1); % store minimum e2e rate

 % V2N2V delay metric
Cdelay = 2 * Pkt/RAT_th * 1e3; % in ms
Ccost = 10;

% Relay selection for each cluster
for i = 1:NUM
   
    fg = find(Cluster_loc(2*i-1, 2:end) == 0, 1); % find edge vehicle
    if fg == 1 % Cluster size == 1
        disp('Use V2I2V unicast');
        dLoc = Cluster_loc(2*i-1: 2*i, fg);
        rel = 1.0;
        delay = Cdelay;
        cost =  Ccost;
        fidx = find(ismember(loc(:,2:3), dLoc','rows'));
        indx = loc(fidx,1);
        Cluster_idx(i,1:fg) = indx;
        CH_idx(i,1) = indx;
    else
        front = Cluster_loc(2*i-1: 2*i, 1);
        back = Cluster_loc(2*i-1: 2*i, fg);
        ch = Cluster_head(:, i);
        % retrieve member and CH indices
        for v = 1:fg
            fidx = find(ismember(loc(:,2:3), Cluster_loc(2*i-1: 2*i, v)','rows'),1);
            indx = loc(fidx,1);
            Cluster_idx(i,v) = indx;
        end
        hidx = find(ismember(loc(:,2:3), ch','rows'),1);
        CH_idx(i,1) = loc(hidx,1);
            
        if isequal(Cluster_head(:,i), front) % CH is the front vehicle of the cluster
            ctype = 1;
            tempLoc = Cluster_loc(2*i-1, 1:fg) - ch(1); % scalarize location of each cluster
            tempCov = back(1) - front(1);
        elseif isequal(Cluster_head(:,i), back) % CH is the back vehicle of the cluster
            ctype = 2;
            tempLoc = abs(Cluster_loc(2*i-1, 1:fg) - ch(1));
            tempLoc = flip(tempLoc);
            tempCov = back(1) - front(1);
        else % CH is in the middel of the cluster
            ctype = 3;
            tempLoc = abs(Cluster_loc(2*i-1, 1:fg) - ch(1));
            fch = find(tempLoc == 0);
            tempLoc1 = tempLoc(fch:end); % forward
            tempLoc2 = flip(tempLoc(1:fch)); % backward
            tempCov1 = back(1) - ch(1); % forward coverage
            tempCov2 = ch(1) - front(1); % backward coverage
        end
        
        switch ctype
            case 1
                % Method 1 dijkstra.m
                graph = myGraph(tempLoc, QoS_th, Ch_par, refDis);
                [~, rCan] = dijkstra(graph, 1, length(tempLoc)); % cost is a e2e weight cost
                % Method 2 dijkstra_v2.m
    %             [g1, g2] = myGraph_v2(tempLoc, QoS_th, Ch_par, refDis);
    %             [cost, rCan] = dijkstra_v2(g1, g2, 1, length(tempLoc), QoS_th);
                dTable = distTable(tempLoc(rCan(2:end)), tempLoc(rCan(1)));
                [pdr, rate] = linkCal(dTable, Ch_par);
                rel = prod(pdr);
                delay = Cdelay + sum(Pkt./rate + Tau * ones(length(pdr),1)) * 1e3; % e2e delay
                dLoc = Cluster_loc(2*i-1: 2*i, rCan);
                cost = size(dLoc,2) + Ccost;
                indx = zeros(1,length(rCan));
                for v=1:size(dLoc,2)
                    fidx = find(ismember(loc(:,2:3),dLoc(:,v)','rows'),1);
                    indx(v) = loc(fidx,1);
                end
                dRate = min(rate);
                
            case 2
                % Method 1 dijkstra.m
                graph = myGraph(tempLoc, QoS_th, Ch_par, refDis);
                [~, rCan] = dijkstra(graph, 1, length(tempLoc)); % cost is a e2e weight cost
                % Method 2 dijkstra_v2.m
    %             [g1, g2] = myGraph_v2(tempLoc, QoS_th, Ch_par, refDis);
    %             [cost, rCan] = dijkstra_v2(g1, g2, 1, length(tempLoc), QoS_th);
                dTable = distTable(tempLoc(rCan(2:end)), tempLoc(rCan(1)));
                [pdr, rate] = linkCal(dTable, Ch_par);
                rel = prod(pdr);
                delay = Cdelay + sum(Pkt./rate + Tau * ones(length(pdr),1)) * 1e3; % e2e delay
                rCan = length(tempLoc)+1 - rCan; % reverse relay index
%                 rCan = sort(rCan);
                dLoc = Cluster_loc(2*i-1: 2*i, rCan);
                indx = zeros(1,length(rCan));
                cost = size(dLoc,2) + Ccost;
                for v=1:size(dLoc,2)
                    fidx = find(ismember(loc(:,2:3),dLoc(:,v)','rows'),1);
                    indx(v) = loc(fidx,1);
                end
                PP = [dLoc; indx]';
                PP = sortrows(PP,1);
                dLoc = PP(:,1:2)';
                indx = PP(:,3)';
                dRate = min(rate);
                
            case 3
                % Forward part
                 % Method 1 dijkstra.m
                graph1 = myGraph(tempLoc1, QoS_th, Ch_par, refDis);
                [~, rCan1] = dijkstra(graph1, 1, length(tempLoc1)); % cost is a e2e weight cost
                % Method 2 dijkstra_v2.m
    %             [g1, g2] = myGraph_v2(tempLoc1, QoS_th, Ch_par, refDis);
    %             [cost1, rCan1] = dijkstra_v2(g1, g2, 1, length(tempLoc1), QoS_th);
                dTable1 = distTable(tempLoc1(rCan1(2:end)), tempLoc1(rCan1(1)));
                [pdr1, rate1] = linkCal(dTable1, Ch_par);
                rel1 = prod(pdr1);
                delay1 = Cdelay + sum(Pkt./rate1 + Tau * ones(length(pdr1),1)) * 1e3; % e2e delay
                dLoc1 = Cluster_loc(2*i-1: 2*i, rCan1(2:end) + fch -1);
                indx1 = zeros(1,size(dLoc1,2));
                cost1 = size(dLoc1,2);
                for v=1:size(dLoc1,2)
                    fidx = find(ismember(loc(:,2:3),dLoc1(:,v)','rows'),1);
                    indx1(v) = loc(fidx,1);
                end
                
                % Backward part (involve CH)
                 % Method 1 dijkstra.m
                graph2 = myGraph(tempLoc2, QoS_th, Ch_par, refDis);
                [~, rCan2] = dijkstra(graph2, 1, length(tempLoc2)); % cost is a e2e weight cost
                % Method 2 dijkstra_v2.m
    %             [g1, g2] = myGraph_v2(tempLoc2, QoS_th, Ch_par, refDis);
    %             [cost2, rCan2] = dijkstra_v2(g1, g2, 1, length(tempLoc1), QoS_th);
                dTable2 = distTable(tempLoc2(rCan2(2:end)), tempLoc2(rCan2(1)));
                [pdr2, rate2] = linkCal(dTable2, Ch_par);
                rel2 = prod(pdr2);
                delay2 = Cdelay + sum(Pkt./rate2 + Tau * ones(length(pdr2),1)) * 1e3; % e2e delay
                rCan2 = length(tempLoc2)+1 - rCan2; % reverse relay index
                dLoc2 = Cluster_loc(2*i-1: 2*i, rCan2);
                indx2 = zeros(1,size(dLoc2,2));
                cost2 = size(dLoc2,2);
                for v=1:size(dLoc2,2)
                    fidx = find(ismember(loc(:,2:3), dLoc2(:,v)','rows'),1);
                    indx2(v) = loc(fidx,1);
                end
                
                % Aggregate
                dLoc = [dLoc2, dLoc1];
                delay = max(delay1, delay2);
                rel = min(rel1, rel2);
                cost = cost1 + cost2 + Ccost;
                indx = [indx2, indx1];
                PP = [dLoc; indx]';
                PP = sortrows(PP,1);
                dLoc = PP(:,1:2)';
                indx = PP(:,3)';
                dRate = min([rate1; rate2]);
        end
    end
    loc_con(2*i-1: 2*i, 1:size(dLoc,2)) = dLoc;
    rel_con(i,1) = rel;
    if i == 1
        delay = delay-Cdelay; % no V2N delay in the first cluster
        cost = cost - Ccost; % no V2N cost in the first cluster
    end
    del_con(i,1) = delay;
    cos_con(i,1) = cost;
    idx_con(i,1:length(indx)) = indx;  % relay indices
    rat_con(i,1) = dRate;
end
