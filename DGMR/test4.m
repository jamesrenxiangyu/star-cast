%% Version with kmeans cluster algorihtm and GPCA

%% Generate mobility trace
clear; clc;
close all;

path = 'SUMO/';
locfile = 'Locations.mat';
mobfile = 'Speed.mat';

load([path locfile])
load([path mobfile])

% Reserve data with target vehicle (id=1)
tar_id1 = find(Pos_x(:,1) == 1); % locate target vehicle
tar_id2 = find(Pos_x(:,1) ~= 0, 1, 'Last'); % count total vehicle number
tar_begin = find(Pos_x(tar_id1, 2:end) ~= 0, 1); % time target vehicle enters the network
tar_end = find(Pos_x(tar_id1, :) ~= 0, 1, 'Last'); % time target vehicle exits the network

% retrieve gloable network information over time involving target vehicle
loc_x = [Pos_x(1:tar_id1, 1) Pos_x(1:tar_id1, tar_begin:tar_end)];
loc_y = [Pos_y(1:tar_id1, 1) Pos_y(1:tar_id1, tar_begin:tar_end)];
mob = [Speed(1:tar_id1, 1) Speed(1:tar_id1, tar_begin:tar_end)]; 


%% MBS sample vehicle mobility information 

sampRate = 10; % sample data every 10s
LOC_x = loc_x(:,1:sampRate:end);
LOC_y = loc_y(:,1:sampRate:end);
MOB = mob(:,1:sampRate:end);

flag = 60;
locRaw = [LOC_x(2:end, flag), LOC_y(2:end, flag)]; % Use flag's timestep
loc = sortrows(locRaw,1);
spd = MOB(2:end, flag);
% xmin = min(loc(:,1));
floc = [LOC_x(2:end, flag+1), LOC_y(2:end, flag+1)]; % Test with next timestep
floc = sortrows(floc,1);


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
disRef = min(dSNR, dRate);
covRef = disRef * (DEL_th/(Tau+1e-3));
dDel = covRef * (Pkt + Tau * RAT_th)/(DEL_th * RAT_th);

dmin = dDel;
dmax = min(dSNR, dRate);
if dmin > dmax
    disp('Service Error, adjust QoS');
end
refDis = [dmin, dmax];

%% Cluster network 
mode = 1; % cluster algorithm mode
[nCluster, Cluster_loc, Cluster_head] = myClusterV2(loc', spd, 10, disRef, covRef, 2);

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

% Select GPCA = 1 or LPCA = 0
PCA = 1;

 % V2N2V delay metric
Cdelay = 2 * Pkt/RAT_th * 1e3; % in ms
Ccost = 10;

% Relay selection for each cluster
for i = 1:NUM
   
    fg = find(Cluster_loc(2*i-1, 2:end) == 0, 1); % find edge vehicle
    if fg == 1 % Cluster size == 1
        disp('Use V2I2V unicast');
        dLoc = Cluster_loc(2*i-1: 2*i, :);
        rel = 1.0;
        delay = Cdelay;
        cost =  Ccost;
    else
        front = Cluster_loc(2*i-1: 2*i, 1);
        back = Cluster_loc(2*i-1: 2*i, fg);
        ch = Cluster_head(:, i);
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
                [cost, rCan] = dijkstra(graph, 1, length(tempLoc)); % cost is a e2e weight cost
                % Method 2 dijkstra_v2.m
    %             [g1, g2] = myGraph_v2(tempLoc, QoS_th, Ch_par, refDis);
    %             [cost, rCan] = dijkstra_v2(g1, g2, 1, length(tempLoc), QoS_th);
                dTable = distTable(tempLoc(rCan(2:end)), tempLoc(rCan(1)));
                [pdr, rate] = linkCal(dTable, Ch_par);
                rel = prod(pdr);
                delay = Cdelay + sum(Pkt./rate + Tau * ones(length(pdr),1)) * 1e3; % e2e delay
                dLoc = Cluster_loc(2*i-1: 2*i, rCan);
                cost = cost + Ccost;
            case 2
                % Method 1 dijkstra.m
                graph = myGraph(tempLoc, QoS_th, Ch_par, refDis);
                [cost, rCan] = dijkstra(graph, 1, length(tempLoc)); % cost is a e2e weight cost
                % Method 2 dijkstra_v2.m
    %             [g1, g2] = myGraph_v2(tempLoc, QoS_th, Ch_par, refDis);
    %             [cost, rCan] = dijkstra_v2(g1, g2, 1, length(tempLoc), QoS_th);
                dTable = distTable(tempLoc(rCan(2:end)), tempLoc(rCan(1)));
                [pdr, rate] = linkCal(dTable, Ch_par);
                rel = prod(pdr);
                delay = Cdelay + sum(Pkt./rate + Tau * ones(length(pdr),1)) * 1e3; % e2e delay
                rCan = length(tempLoc)+1 - rCan; % reverse relay index
                rCan = sort(rCan);
                dLoc = Cluster_loc(2*i-1: 2*i, rCan);
                cost = cost + Ccost;
            case 3
                % Forward part
                 % Method 1 dijkstra.m
                graph1 = myGraph(tempLoc1, QoS_th, Ch_par, refDis);
                [cost1, rCan1] = dijkstra(graph1, 1, length(tempLoc1)); % cost is a e2e weight cost
                % Method 2 dijkstra_v2.m
    %             [g1, g2] = myGraph_v2(tempLoc1, QoS_th, Ch_par, refDis);
    %             [cost1, rCan1] = dijkstra_v2(g1, g2, 1, length(tempLoc1), QoS_th);
                dTable1 = distTable(tempLoc1(rCan1(2:end)), tempLoc1(rCan1(1)));
                [pdr1, rate1] = linkCal(dTable1, Ch_par);
                rel1 = prod(pdr1);
                delay1 = Cdelay + sum(Pkt./rate1 + Tau * ones(length(pdr1),1)) * 1e3; % e2e delay
                dLoc1 = Cluster_loc(2*i-1: 2*i, rCan1(2:end) + fch -1);
                
                % Backward part
                 % Method 1 dijkstra.m
                graph2 = myGraph(tempLoc2, QoS_th, Ch_par, refDis);
                [cost2, rCan2] = dijkstra(graph2, 1, length(tempLoc2)); % cost is a e2e weight cost
                % Method 2 dijkstra_v2.m
    %             [g1, g2] = myGraph_v2(tempLoc2, QoS_th, Ch_par, refDis);
    %             [cost2, rCan2] = dijkstra_v2(g1, g2, 1, length(tempLoc1), QoS_th);
                dTable2 = distTable(tempLoc2(rCan2(2:end)), tempLoc2(rCan2(1)));
                [pdr2, rate2] = linkCal(dTable2, Ch_par);
                rel2 = prod(pdr2);
                delay2 = Cdelay + sum(Pkt./rate2 + Tau * ones(length(pdr2),1)) * 1e3; % e2e delay
                rCan2 = length(tempLoc2)+1 - rCan2; % reverse relay index
                dLoc2 = Cluster_loc(2*i-1: 2*i, rCan2);
                
                % Aggregate
                dLoc = [flip(dLoc2,2), dLoc1];
                delay = max(delay1, delay2);
                rel = min(rel1, rel2);
                cost = cost1 + cost2 + Ccost;
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
end




%% Results visualization
% figure(1)
% for j = 1:NUM
%     jj = find(Cluster_loc(2*j-1, 2:end) == 0, 1);
%     plot(linspace(Cluster_loc(2*j-1, 1), Cluster_loc(2*j-1, jj), length(Cluster_loc(2*j-1, 1:jj))), 'r-');
%     hold on;
%     a = loc_con(2*j-1, 2:end);
%     flag = find(a == 0, 1);
%     plot(loc_con(2*j-1,1: flag), '*');
%     hold on;
% end
% 
% figure(2);
% bar(rel_con);
% ylim([REL_th 1]);
% ylabel("Probability");
% xlabel("Cluster number");
% title("E2E relibility");
% 
% figure(3);
% bar(del_con);
% ylabel("Delay in (ms)");
% xlabel("Cluster number");
% title("E2E delay");
