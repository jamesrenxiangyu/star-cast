close all;
clear; clc;

% QoS settings
DEL_th = 20e-3; % E2E delay in sec
SNR_th = 18; % SNR in dB
RAT_th = 20e6; % Data rate in bps
PDR_th = 1 - 1e-4; % Per-hop reliability
REL_th = 1 - 1e-3; % End-to-end reliability

QoS_th = [SNR_th; RAT_th; PDR_th; REL_th; DEL_th];

% Traffic settings
Coverage = 5000;   % Coverage in meters
Veh_num = 100; % Number of vehicles on road
Lane_num = 3; % Number of lanes
% Traffic locations
Loc = 0;

s = RandStream('dsfmt19937', 'Seed', 3);
for i = 1:Lane_num
   loc = randi(s,[0 Coverage], 1, Veh_num); % Vehicle locations
   Loc = [Loc, loc];
end
Loc = sort(Loc);

Loc_test = (1:20:1000);

% Channel settings
Pkt = 1600 * 8; % packet size
B = 5e6; % bandwidth in Hz
M = 4; % M-PSK
F = 5e9; % Radio frequency 5GHz
alpha = 3.2; % Path loss component
Xg = 3.5; % Shadowing component in dB
N0 = 1e-8; % Noise power level in mW
Pt_max = 23; % in mW;
Tau = 2e-3; % Delay constant in sec

Ch_par = [M, Pt_max, B, alpha, N0, Xg, F, Pkt, Tau];

% Initial Reference distance
dSNR = distSNR(SNR_th, M, Pt_max, alpha, N0, Xg, F);
dRate = distRate(RAT_th, B, M, Pt_max, alpha, N0, Xg, F);
% disRef = max(dSNR, dRate);
dmax = min(dSNR, dRate);
covRef = 0.8 * dmax * (DEL_th/(Tau+1e-3));
dDel = covRef * (Pkt + Tau * RAT_th)/(DEL_th * RAT_th);
dmin = dDel;

if dmin > dmax
    disp('Service Error, adjust QoS');
end
refDis = [dmin, dmax];

% test
% [rCan, rPDR, rRATE, rRel] = DGMR_v2(QoS_th, Loc_test, Loc_test(end), Ch_par, refDis);


% Cluster network 
[nCluster, Cluster_ind, Cluster_loc] = myCluster(Loc', dmax, covRef);

% Select GPCA = 1 or LPCA = 0
PCA = 1;

% Set relay location container and performance metrics container
NUM = nCluster;
HOP_MAX = ceil(DEL_th/Tau); % max number of hops per cluster
loc_con = zeros(NUM, HOP_MAX); % store lcoations of each relay per cluster
rel_con = zeros(NUM, 1); % store e2e reliability
del_con = zeros(NUM, 1); % store e2e delay
cos_con = zeros(NUM, 1); % store e2e cost

% Relay selection for each cluster
for i =1: nCluster
    fg = find(Cluster_loc(i, 2:end) == 0, 1);
    tempLoc = Cluster_loc(i, 1:fg) - Cluster_loc(i, 1); % scalarize location of each cluster
    tempCov = Cluster_loc(i, fg) - Cluster_loc(i, 1);
    
    % PCA selection
    if PCA == 1
        % Use logarithm-weight to find the shortest path
        graph = myGraph(tempLoc, QoS_th, Ch_par, refDis);
        [cost, rCan] = dijkstra(graph, 1, length(tempLoc));
        % Use modified Dijkstra (filter out unavailable path wrt e2e reliability)
%         [g1, g2] = myGraph_v2(tempLoc, QoS_th, Ch_par, refDis);
%         [cost, rCan] = dijkstra_v2(g1, g2, 1, length(tempLoc), QoS_th);
        dTable = distTable(tempLoc(rCan(2:end)), tempLoc(rCan(1)));
        dLoc = Cluster_loc(i,1) + tempLoc(rCan);
        [pdr, rate] = linkCal(dTable, Ch_par);
        rel = prod(pdr);
        delay = sum(Pkt./rate + Tau * ones(length(pdr),1)) * 1e3; % e2e delay
    else
        % DGMR-LPCA
        [rCan, rPDR, rRATE, rRel] = DGMR_v2(QoS_th, tempLoc, tempCov, Ch_par, refDis);
        dLoc = [Cluster_loc(i,1); Cluster_loc(i,1) + rCan]';
        rel = rRel; % e2e reliability
        delay = sum(Pkt./rRATE + Tau * ones(size(rRATE,1),1)) * 1e3; % e2e delay in ms
        cost = 0;
    end
    loc_con(i,1:length(dLoc)) = dLoc;
    rel_con(i,1) = rel;
    del_con(i,1) = delay;
    cos_con(i,1) = cost;
end


figure(1)
for j = 1:NUM
    jj = find(loc_con(j, 2:end) == 0, 1);
    if isempty(jj)
        jj = size(loc_con,2);
    end
    plot(linspace(loc_con(j, 1), loc_con(j, jj), length(loc_con(j, 1:jj))), 'r-');
    hold on;
    plot(loc_con(j,1: jj), '*');
    hold on;
end



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




 


