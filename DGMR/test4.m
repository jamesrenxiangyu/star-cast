%% Version with kmeans cluster algorihtm and LPCA without time series

%% Generate mobility trace
clear; clc;
close all;

path = 'SUMO/';
file = 'sampledenseTrace.mat';
% mobfile = 'Speed.mat';

load([path file])

%% MBS sample vehicle mobility information 
timeGap = 10;
flag = 3;
X = [Sloc_x(2:end, 1), Sloc_x(2:end, flag),  Sloc_y(2:end, flag), Smob(2:end, flag)];
X = sortrows(X,2);
loc = X(:, 1:3);
spd = X(:, [1 4]);

fX = [Sloc_x(2:end, 1), Sloc_x(2:end, flag+1),  Sloc_y(2:end, flag+1), Smob(2:end, flag+2)];
fX = sortrows(fX,2);
floc = fX(:, 1:3);
fspd = fX(:, [1 4]);
% %% Generate test data
% timeGap = 10;
% Vnum = 100;
% cov_begin = 200;
% cov_end = 5000;
% 
% Locations = [randi([cov_begin cov_end], [Vnum, 1]), 4*randi([0, 3], [Vnum, 1])];
% Speed = randi([30 40], [Vnum, 1]);
% Idx = 10 + (1:size(Locations,1))';
% X = [Idx, Locations, Speed];
% X = sortrows(X, 2);
% loc = X(:, 1:3);
% spd = X(:, [1 4]);
% % CH_idx = Locations(round(Vnum/2),1);
% % idx = [1 14 23 50 63 75 89 95];
% % refRelay = Locations(idx,1);



%% Network setting
% QoS settings
DEL_th = 20e-3; % E2E delay in sec
SNR_th = 20; % SNR in dB
RAT_th = 20e6; % Data rate in bps
PDR_th = 1 - 1e-4; % Per-hop reliability
REL_th = 1 - 1e-3; % End-to-end reliability

QoS_th = [SNR_th; RAT_th; PDR_th; REL_th; DEL_th];

% Channel settings
Pkt = 1600; % packet size
B = 5e6; % bandwidth in Hz
M = 4; % M-PSK
F = 5e9; % Radio frequency 5GHz
alpha = 3.6; % Path loss component
Xg = 3.7; % Shadowing component in dB
N0 = 2e-9; % Noise power level in mW
Pt_max = 23; % in mW;
Tau = 2e-3; % Delay constant in sec

Ch_par = [M, Pt_max, B, alpha, N0, Xg, F, Pkt, Tau];

% Initial Reference distance
dSNR = distSNR(SNR_th, M, Pt_max, alpha, N0, Xg, F);
dRate = distRate(RAT_th, B, M, Pt_max, alpha, N0, Xg, F);
dmax = min(dSNR, dRate);
covRef = dmax * (DEL_th/(Tau+1e-3));
dDel = covRef * (Pkt + Tau * RAT_th)/(DEL_th * RAT_th);
dmin = dDel;

if dmin > dmax
    disp('Service Error, adjust QoS');
end
refDis = [dmin, dmax];

%% Cluster network 
mode = 2; % cluster algorithm mode
[nCluster, Cluster_loc, Cluster_head, Cluster_spd] = myClusterV2(loc, spd, timeGap, refDis, covRef, mode);

% init cluster member and cluster head indices
% Cluster_idx: member index, array of size K x N
Cluster_idx = zeros(nCluster, size(Cluster_loc,2));
% CH_idx: CH index, array of size K x 1
CH_idx = zeros(nCluster, 1);

% % Visualize cluster results
% figure(9);
% for i = 1:size(Cluster_head,2)
%     temp = Cluster_loc(2*i-1:2*i, :);
%     ff = find(temp(1, 2:end) == 0, 1);
%     subplot(2,1,1)
%     scatter(temp(1, 1:ff), temp(2, 1:ff), 'o');
%     hold on;
%     scatter(Cluster_head(1,i), Cluster_head(2,i), 'kx', 'LineWidth', 1.5)
% end
% grid on;
% title("Kmeans with revised distance");
% subplot(2,1,2)
% scatter(floc(:,1), floc(:,2));
% grid on;

%% Routing algorithm
% Set relay location container and performance metrics container
NUM = nCluster; % total number of clusters
HOP_MAX = ceil(DEL_th/Tau); % max number of hops per cluster
loc_con = zeros(2*NUM, HOP_MAX); % store lcoations of each relay per cluster
rel_con = zeros(NUM, 1); % store e2e reliability
del_con = zeros(NUM, 1); % store e2e delay
cos_con = zeros(NUM, 1); % store e2e cost
idx_con = zeros(NUM, HOP_MAX); % store relay index
rat_con = zeros(NUM, 1); % store minimum e2e rate

% Select GPCA = 1 or LPCA = 0
PCA = 1;

 % V2N2V delay metric
Cdelay = 2 * Pkt/RAT_th * 1e3; % in ms
Ccost = 10;

% GPCA part
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
%% Results visualization
figure(1)
for j = 1:NUM
    jj = find(Cluster_loc(2*j-1, 2:end) == 0, 1);
    plot(linspace(Cluster_loc(2*j-1, 1), Cluster_loc(2*j-1, jj), length(Cluster_loc(2*j-1, 1:jj))), 'r-');
    hold on;
    a = Cluster_loc(2*j-1, 2:end); % Cluster members
    flag = find(a == 0, 1);
    plot(Cluster_loc(2*j-1,1: flag), 'o');
    hold on; 
    b = find(loc_con(2*j-1, 2:end) == 0, 1); % Relay vehicles
    if isempty(b)
        b = length(loc_con(2*j-1,:));
    end
    for r = 1:b
        flag = find(Cluster_loc(2*j-1,:) == loc_con(2*j-1,r));
        plot(flag, loc_con(2*j-1,r), 'r*');
        hold on;
    end
    flag = find(Cluster_loc(2*j-1,:) == Cluster_head(1,j));
    plot(flag, Cluster_head(1,j), 'pentagram', 'MarkerSize', 10, 'LineWidth', 1.5);
end

% figure(2);
% bar(rel_con);
% ylim([REL_th 1]);
% ylabel("Probability");
% xlabel("Cluster number");
% title("E2E relibility");

figure(3);
bar(del_con);
ylabel("Delay in (ms)");
xlabel("Cluster number");
title("E2E delay");

% figure(4);
% bar(rat_con);
% ylabel("Data rate in (bps)");
% xlabel("Cluster number");
% title("Minimum data rate");
% 
% figure(5);
% bar(cos_con);
% ylabel("Cost");
% xlabel("Cluster number");
% title("Transmission cost");


%% LPCA part
% Init containers
rloc_con = zeros(2*NUM, HOP_MAX); % store lcoations of each relay per cluster
ridx_con = zeros(NUM, HOP_MAX); % store relay index
rrel_con = zeros(NUM, 1); % store e2e reliability
rdel_con = zeros(NUM, 1); % store e2e delay
rcos_con = zeros(NUM, 1); % store e2e cost
rrat_con = zeros(NUM, 1); % store minimum e2e rate

% Relay selection for each cluster
for i = 1:NUM
    fg = find(Cluster_loc(2*i-1, 2:end) == 0, 1); % find edge vehicle
    input_loc = [Cluster_idx(i, 1:fg)', Cluster_loc(2*i-1:2*i, 1:fg)'];
    input_spd = [Cluster_idx(i, 1:fg)', Cluster_spd(i,1:fg)'];
    if fg == 1 % Cluster size == 1
        disp('Use V2I2V unicast');
        rloc = input_loc(1:fg, 2:3)';
        ridx = input_loc(1:fg, 1);
        Rel = 1.0;
        delay = Cdelay;
        cost =  Ccost;
    else
        ch = Cluster_head(:, i);
        fp = find(idx_con(i,:) == 0, 1); % find edge relay
        if isempty(fp)
            fp = length(idx_con(i,:));
        end
        % organize input data
        [ridx, Del, Rel] = DGMR_V3(QoS_th, input_loc, input_spd, idx_con(i,1:fp-1)', CH_idx(i,1)', Ch_par, refDis);
        ridx = unique(ridx, 'stable');
        rloc = [];
        for k = 1:length(ridx)
            gg = find(input_loc(:,1) == ridx(k));
            rloc = [rloc; input_loc(gg, 2:3)];
        end
        Rel = prod(Rel);
        delay = sum(Del*1e3) + Cdelay;
        cost = length(ridx) + Ccost;
    end
    rloc_con(2*i-1: 2*i, 1:length(ridx)) = rloc';
    ridx_con(i, 1:length(ridx)) = ridx;
    rrel_con(i,1) = Rel;
    if i == 1
        delay = delay-Cdelay; % no V2N delay in the first cluster
        cost = cost - Ccost; % no V2N cost in the first cluster
    end
    rdel_con(i,1) = delay;
    rcos_con(i,1) = cost;
end

%% Results visualization
figure(6)
for j = 1:NUM
    jj = find(Cluster_loc(2*j-1, 2:end) == 0, 1);
    plot(linspace(Cluster_loc(2*j-1, 1), Cluster_loc(2*j-1, jj), length(Cluster_loc(2*j-1, 1:jj))), 'r-');
    hold on;
    a = Cluster_loc(2*j-1, 2:end); % Cluster members
    flag = find(a == 0, 1);
    plot(Cluster_loc(2*j-1,1: flag), 'o');
    hold on; 
    b = find(rloc_con(2*j-1, 2:end) == 0, 1); % Relay vehicles
    if isempty(b)
        b = length(rloc_con(2*j-1,:));
    end
    for r = 1:b
        flag = find(Cluster_loc(2*j-1,:) == rloc_con(2*j-1,r));
        plot(flag, rloc_con(2*j-1,r), 'r*');
        hold on;
    end
    flag = find(Cluster_loc(2*j-1,:) == Cluster_head(1,j));
    plot(flag, Cluster_head(1,j), 'pentagram', 'MarkerSize', 10, 'LineWidth', 1.5);
end

% figure(7);
% bar(rrel_con);
% ylim([REL_th 1]);
% ylabel("Probability");
% xlabel("Cluster number");
% title("E2E relibility");

figure(8);
bar(rdel_con);
ylabel("Delay in (ms)");
xlabel("Cluster number");
title("E2E delay");

% figure(4);
% bar(rat_con);
% ylabel("Data rate in (bps)");
% xlabel("Cluster number");
% title("Minimum data rate");
% 
% figure(9);
% bar(rcos_con);
% ylabel("Cost");
% xlabel("Cluster number");
% title("Transmission cost");
