%% Version with V2N2V unicast

clear; clc; 
close all;

path = 'SUMO/';
locfile = 'test_loc.mat';
mobfile = 'test_mob.mat';

load([path locfile])
load([path mobfile])

% Reserve data with target vehicle (id=1)
tar_id1 = find(L(:,1) == 1); 
tar_id2 = find(L(:,1) ~= 0, 1, 'Last');
tar_begin = find(L(tar_id1, 2:end) ~= 0, 1);
tar_end = find(L(tar_id1, :) ~= 0, 1, 'Last');

% retrieve all network information over time involving target vehicle
loc = [L(1:tar_id2, 1) L(1:tar_id2, tar_begin:tar_end)];
mob = [S(1:tar_id2, 1) S(1:tar_id2, tar_begin:tar_end)]; 

% QoS settings
DEL_th = 20e-3; % E2E delay in sec
SNR_th = 18; % SNR in dB
RAT_th = 20e6; % Data rate in bps
PDR_th = 1 - 1e-4; % Per-hop reliability
REL_th = 1 - 1e-3; % End-to-end reliability

QoS_th = [SNR_th; RAT_th; PDR_th; REL_th; DEL_th];

% Channel settings
Pkt = 1600; % packet size
B = 5e6; % bandwidth in Hz
M = 4; % M-PSK
F = 5e9; % Radio frequency 5GHz
alpha = 3.2; % Path loss component
Xg = 3.5; % Shadowing component in dB
N0 = 1e-9; % Noise power level in mW
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

% Cluster network 
Loc = sort(loc(2:33,2));
Loc = unique(Loc);
[nCluster, Cluster_ind, Cluster_loc] = myCluster(Loc, disRef, covRef);

       
% Set relay location container and performance metrics container
NUM = nCluster; % total number of clusters
HOP_MAX = ceil(DEL_th/Tau); % max number of hops per cluster
loc_con = zeros(NUM, HOP_MAX); % store lcoations of each relay per cluster
rel_con = zeros(NUM, 1); % store e2e reliability
del_con = zeros(NUM, 1); % store e2e delay
cos_con = zeros(NUM, 1); % store e2e cost

% Select GPCA = 1 or LPCA = 0
PCA = 1;

% Relay selection for each cluster
for i = 1:NUM
    fg = find(Cluster_loc(i, 2:end) == 0, 1);
    if fg == 1 % Cluster size == 1
        dLoc = Cluster_loc(i,1);
        disp('Use V2I2V unicast');
        rel = 1.0;
        rate = RAT_th;
        delay = 2 * Pkt/rate * 1e3;
    else
        tempLoc = Cluster_loc(i, 1:fg) - Cluster_loc(i, 1); % scalarize location of each cluster
        tempCov = Cluster_loc(i, fg) - Cluster_loc(i, 1);
        % PCA selection
        if PCA == 1 % DGMR-GPCA
            % Method 1 dijkstra.m
            graph = myGraph(tempLoc, QoS_th, Ch_par, refDis);
            [cost, rCan] = dijkstra(graph, 1, length(tempLoc)); % cost is a e2e weight cost
            % Method 2 dijkstra_v2.m
    %         [g1, g2] = myGraph_v2(tempLoc, QoS_th, Ch_par, refDis);
    %         [cost, rCan] = dijkstra_v2(g1, g2, 1, length(tempLoc), QoS_th);
            dTable = distTable(tempLoc(rCan(2:end)), tempLoc(rCan(1)));
            dLoc = Cluster_loc(i,1) + tempLoc(rCan);
            [pdr, rate] = linkCal(dTable, Ch_par);
            rel = prod(pdr);
            delay = sum(Pkt./rate + Tau * ones(length(pdr),1)) * 1e3; % e2e delay
        elseif PCA == 0 % DGMR-LPCA
            % DGMR-LPCA
            [rCan, rPDR, rRATE, rRel] = DGMR_v2(QoS_th, tempLoc, tempCov, Ch_par, refDis);
            dLoc = [Cluster_loc(i,1); Cluster_loc(i,1) + rCan]';
            rel = rRel; % e2e reliability
            delay = sum(Pkt./rRATE + Tau * ones(size(rRATE,1),1)) * 1e3; % e2e delay in ms
            cost = 0;
        end
    end
    loc_con(i,1:length(dLoc)) = dLoc;
    rel_con(i,1) = rel;
    del_con(i,1) = delay;
    cos_con(i,1) = cost;
end

figure(1)
for j = 1:NUM
    jj = find(Cluster_loc(j, 2:end) == 0, 1);
    plot(linspace(Cluster_loc(j, 1), Cluster_loc(j, jj), length(Cluster_loc(j, 1:jj))), 'r-');
    hold on;
    a = loc_con(j,2:end);
    flag = find( a == 0, 1);
    plot(loc_con(j,1: flag), '*');
    hold on;
end
legend();

figure(2);
bar(rel_con);
ylim([REL_th 1]);
ylabel("Probability");
xlabel("Cluster number");
title("E2E relibility");

figure(3);
bar(del_con);
ylabel("Delay in (ms)");
xlabel("Cluster number");
title("E2E delay");
