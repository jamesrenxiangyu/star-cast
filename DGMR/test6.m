%% Version with kmeans cluster algorihtm without LPCA overtime

%% Generate mobility trace
clear; clc;
close all;

path = 'SUMO/';
MBSfile = 'sampledenseTrace.mat';
Vehiclefile = 'denseTrace.mat';
% mobfile = 'Speed.mat';

load([path MBSfile])
load([path Vehiclefile])

%% Network setting
% QoS settings
DEL_th = 20e-3; % E2E delay in sec
SNR_th = 20; % SNR in dB
RAT_th = 20e6; % Data rate in bps
PDR_th = 1 - 1e-4; % Per-hop reliability
REL_th = 1 - 1e-3; % End-to-end reliability

QoS_th = [SNR_th; RAT_th; PDR_th; REL_th; DEL_th];

% Channel settings
Pkt = 1600 * 8; % packet size
B = 5e6; % bandwidth in Hz
M = 4; % M-PSK
F = 5e9; % Radio frequency 5GHz
alpha = 3.6; % Path loss component
Xg = 3.7; % Shadowing component in dB
N0 = 2e-9; % Noise power level in mW
Pt_max = 23; % in mW;
Tau = 2e-3; % Delay constant in sec

Ch_par = [M, Pt_max, B, alpha, N0, Xg, F, Pkt, Tau];

 % V2N2V delay metric
Cdelay = 2 * Pkt/RAT_th; % in ms
Ccost = 10;

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


%% MBS sample vehicle mobility information 
timeGap = 10;
flag = 3;
time = Sloc_x(1,flag) + 1; % timestamp

X = [Sloc_x(2:end, 1), Sloc_x(2:end, flag),  Sloc_y(2:end, flag), Smob(2:end, flag)];
X = sortrows(X,2);
Gloc = X(:, 1:3);
Gspd = X(:, [1 4]);

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



%% Cluster network 
mode = 2;
[nCluster, Cluster_loc, Cluster_head, Cluster_spd] = myClusterV2(Gloc, Gspd, timeGap, refDis, covRef, mode);

%% GPCA part
[Cluster_idx, CH_idx, loc_con, rel_con, del_con, cos_con, idx_con] = myGPCA(Gloc, nCluster, Cluster_loc, Cluster_head, refDis, Ch_par, QoS_th);

NUM = length(rel_con);
HOP_MAX = size(loc_con,2);

%% LPCA part

Relay_loc = cell(timeGap,1);
Relay_idx = cell(timeGap,1);
Relay_del = cell(timeGap,1);
Relay_cos = cell(timeGap,1);
Relay_rat = cell(timeGap,1);
Relay_rel = cell(timeGap,1);

for t = 0:timeGap-1
    Y = [Pos_x(2:end, 1), Pos_x(2:end, time+t),  Pos_y(2:end, time+t), Speed(2:end, time+t)];
    Y = sortrows(Y,2);
    target = find(Y(:,1) == 1);
    Lloc = Y(target:end, 1:3);
    Lspd = Y(target:end, [1 4]);

    % Init containers
    rrel_con = zeros(NUM, 1); % store e2e reliability
    rdel_con = zeros(NUM, 1); % store e2e delay
    rcos_con = zeros(NUM, 1); % store e2e cost
    rrat_con = zeros(NUM, 1); % store minimum e2e rate

    % Relay selection for each cluster
    for i = 1:NUM
        fg = find(Cluster_idx(i, 2:end) == 0, 1); % find size of the cluster
        input_loc = zeros(fg,3);
        for k = 1:fg
            input_loc(k,:) = Lloc(find(Lloc(:,1) == Cluster_idx(i,k)), :);
        end
        % organize data
        input_loc = sortrows(input_loc,2);
        input_idx = input_loc(:,1);
        % find the edge vehicle in cluster
        fedge_idx = input_loc(end,1);
        fedge_loc = input_loc(end,2:3);
        bedge_idx = input_loc(1,1);
        bedge_loc = input_loc(1,2:3);
        if fg == 1 % Cluster size == 1
            disp('Use V2I2V unicast');
            rloc = input_loc(1:fg, 2:3)';
            ridx = input_loc(1:fg, 1);
            Rel = 1.0;
            delay = Cdelay;
            cost =  Ccost;
        else
            ch = CH_idx(i);    
            gfg = find(idx_con(i,:) == 0, 1);
            if isempty(gfg)
                gfg = length(idx_con(i,:));
            end
            gpath = idx_con(i,1:gfg-1);           
            % Compute e2e QoS metrics
            ch_fg = find(gpath == ch);
%             cluster_pdr = zeros(1,length(gpath)+2);
%             cluster_rat = zeros(1,length(gpath)+2);
%             cluster_del = zeros(1,length(gpath)+2);
            delay = 0;
            rel = 1;
            cost = 0;
            if ch_fg == 1 % CH at back edge
                now_idx = ch;
                now_loc = input_loc(find(input_loc(:,1)==now_idx),2:3);
                for p = 2:length(gpath)
                    nxt_idx = gpath(p);
                    nxt_loc = input_loc(find(input_loc(:,1)==nxt_idx),2:3);
                    dd = norm(nxt_loc-now_loc);
                    [pdr, rate] = linkCal(dd,Ch_par);
                    del = Pkt/rate + Tau;
                    delay = delay + del;
                    rel = rel * pdr;
                    cost = cost + 1;
%                     cluster_pdr(p) = pdr;
%                     cluster_rat(p) = rate;
%                     cluster_del(p) = delay;
                    now_idx = nxt_idx;
                    now_loc = input_loc(find(input_loc(:,1)==now_idx),2:3);
                end
                if gpath(end) ~= fedge_idx
                    dd = norm(now_loc - fedge_loc);
                    [pdr, rate] = linkCal(dd,Ch_par);
                    del = Pkt/rate + Tau;
                    delay = delay + del;
                    rel = rel * pdr;
                    cost = cost + 1;
%                     cluster_pdr(end) = pdr;
%                     cluster_rat(end) = rate;
%                     cluster_del(end) = delay;   
                end
            elseif ch_fg == length(gpath) % CH at front edge 
                now_idx = ch;
                now_loc = input_loc(find(input_loc(:,1)==now_idx),2:3);
                gpath = flip(gpath);
                for p = 2:length(gpath)
                    nxt_idx = gpath(p);
                    nxt_loc = input_loc(find(input_loc(:,1)==nxt_idx),2:3);
                    dd = norm(nxt_loc-now_loc);
                    [pdr, rate] = linkCal(dd,Ch_par);
                    del = Pkt/rate + Tau;
                    delay = delay + del;
                    rel = rel * pdr;
                    cost = cost + 1;
%                     cluster_pdr(p) = pdr;
%                     cluster_rat(p) = rate;
%                     cluster_del(p) = delay;
                    now_idx = nxt_idx;
                    now_loc = input_loc(find(input_loc(:,1)==now_idx),2:3);
                end
                if gpath(end) ~= bedge_idx
                    dd = norm(now_loc - bedge_loc);
                    [pdr, rate] = linkCal(dd,Ch_par);
                    del = Pkt/rate + Tau;
                    delay = delay + del;
                    rel = rel * pdr;
                    cost = cost + 1;
%                     cluster_pdr(p) = pdr;
%                     cluster_rat(p) = rate;
%                     cluster_del(p) = delay; 
                end
            else % CH at middle
                now_idx = ch;
                now_loc = input_loc(find(input_loc(:,1)==now_idx),2:3);
                delay1 = 0;
                rel1 = 1;
                cost1 = 0;
                delay2 = 0;
                rel2 = 1;
                cost2 = 0;
                for p1 = ch_fg+1:length(gpath)
                    nxt_idx = gpath(p1);
                    nxt_loc = input_loc(find(input_loc(:,1)==nxt_idx),2:3);
                    dd = norm(nxt_loc-now_loc);
                    [pdr, rate] = linkCal(dd,Ch_par);
                    del = Pkt/rate + Tau;
                    delay1 = delay1 + del;
                    rel1 = rel1 * pdr;
                    cost1 = cost1 + 1;
%                     cluster_pdr(p) = pdr;
%                     cluster_rat(p) = rate;
%                     cluster_del(p) = delay;
                    now_idx = nxt_idx;
                    now_loc = input_loc(find(input_loc(:,1)==now_idx),2:3);
                end
                if gpath(end) ~= fedge_idx
                    dd = norm(now_loc - fedge_loc);
                    [pdr, rate] = linkCal(dd,Ch_par);
                    del = Pkt/rate + Tau;
                    delay1 = delay + del;
                    rel1 = rel * pdr;
                    cost1 = cost1 + 1;
%                     cluster_pdr(p) = pdr;
%                     cluster_rat(p) = rate;
%                     cluster_del(p) = delay;  
                end 
                now_idx = ch;
                now_loc = input_loc(find(input_loc(:,1)==now_idx),2:3);
                for p2 = ch_fg-1:-1:1
                    nxt_idx = gpath(p2);
                    nxt_loc = input_loc(find(input_loc(:,1)==nxt_idx),2:3);
                    dd = norm(nxt_loc-now_loc);
                    [pdr, rate] = linkCal(dd,Ch_par);
                    del = Pkt/rate + Tau;
                    delay2 = delay2 + del;
                    rel2 = rel2 * pdr;
                    cost2 = cost2 + 1;
%                     cluster_pdr(p) = pdr;
%                     cluster_rat(p) = rate;
%                     cluster_del(p) = delay;
                    now_idx = nxt_idx;
                    now_loc = input_loc(find(input_loc(:,1)==now_idx),2:3);
                end
                if gpath(1) ~= bedge_idx
                    dd = norm(now_loc - bedge_loc);
                    [pdr, rate] = linkCal(dd,Ch_par);
                    del = Pkt/rate + Tau;
                    delay2 = delay2 + del;
                    rel2 = rel2 * pdr;
                    cost2 = cost2 + 1;
%                     cluster_pdr(p) = pdr;
%                     cluster_rat(p) = rate;
%                     cluster_del(p) = delay; 
                end
                delay = max(delay1, delay2);
                rel = rel1 * rel2;
                cost = cost1 + cost2;
            end
        end
         rrel_con(i,1) = rel;
         if i ~= 1
            delay = delay + Cdelay; % no V2N delay in the first cluster
            cost = cost + Ccost; % no V2N cost in the first cluster
         end
            rdel_con(i,1) = delay*1e3;
            rcos_con(i,1) = cost;
    end
    Relay_del{t+1,1} = rdel_con;
    Relay_cos{t+1,1} = rcos_con;
    Relay_rat{t+1,1} = rrat_con;
    Relay_rel{t+1,1} = rrel_con;
end

%% Results visualization
delay_table = zeros(NUM, timeGap);
for i = 1:NUM
    for j = 1:size(Relay_del,1)
        delay_table(i,j) = Relay_del{j,1}(i);
    end
end
for i = 1:NUM
    plot(delay_table(i,:), '-square');
    hold on; 
end
ylim([1,10]);
xlim([1,10]);
xlabel("Timestamp (s)");
ylabel("E2E delay (ms)");
legend('C1', 'C2', 'C3', 'C4','C5', 'C6','C7', 'C8')
grid on;

%% LPCA part

Relay_loc = cell(timeGap,1);
Relay_idx = cell(timeGap,1);
Relay_del = cell(timeGap,1);
Relay_cos = cell(timeGap,1);
Relay_rat = cell(timeGap,1);
Relay_rel = cell(timeGap,1);

for t = 0:timeGap-1
    Y = [Pos_x(2:end, 1), Pos_x(2:end, time+t),  Pos_y(2:end, time+t), Speed(2:end, time+t)];
    Y = sortrows(Y,2);
    target = find(Y(:,1) == 1);
    Lloc = Y(target:end, 1:3);
    Lspd = Y(target:end, [1 4]);

    % Init containers
    rloc_con = zeros(2*NUM, HOP_MAX); % store lcoations of each relay per cluster
    ridx_con = zeros(NUM, HOP_MAX); % store relay index
    rrel_con = zeros(NUM, 1); % store e2e reliability
    rdel_con = zeros(NUM, 1); % store e2e delay
    rcos_con = zeros(NUM, 1); % store e2e cost
    rrat_con = zeros(NUM, 1); % store minimum e2e rate

    % Relay selection for each cluster
    for i = 1:NUM
        fg = find(Cluster_idx(i, 2:end) == 0, 1); % find size of the cluster
        input_loc = zeros(fg,3);
        input_spd = zeros(fg,2);
        for k = 1:fg
            input_loc(k,:) = Lloc(find(Lloc(:,1) == Cluster_idx(i,k)), :);
            input_spd(k,:) = Lspd(find(Lspd(:,1) == Cluster_idx(i,k)), :);
        end
        if fg == 1 % Cluster size == 1
            disp('Use V2I2V unicast');
            rloc = input_loc(1:fg, 2:3)';
            ridx = input_loc(1:fg, 1);
            Rel = 1.0;
            delay = Cdelay;
            cost =  Ccost;
        else
            ch = Cluster_head(:, i);            
            if t <= 1
                fp = find(idx_con(i,:) == 0, 1); % find edge relay
                input_idx = idx_con(i,1:fp-1)';
            else
                fp = find(Relay_idx{t-1,1}(i,:) == 0, 1); % find edge relay
                input_idx = Relay_idx{t-1,1}(i, 1:fp-1)';
            end
            
            if isempty(fp)
                fp = length(idx_con(i,:));
            end
            % organize input data
            [ridx, Del, Rel] = DGMR_V3(QoS_th, input_loc, input_spd, input_idx, CH_idx(i,1)', Ch_par, refDis);
            ridx = unique(ridx, 'stable');
            rloc = [];
            for k = 1:length(ridx)
                gg = find(input_loc(:,1) == ridx(k));
                rloc = [rloc; input_loc(gg, 2:3)];
            end
            Rel = prod(Rel);
            delay = sum(Del) + Cdelay;
            cost = length(ridx) + Ccost;
         end
         rloc_con(2*i-1: 2*i, 1:length(ridx)) = rloc';
         ridx_con(i, 1:length(ridx)) = ridx;
         rrel_con(i,1) = Rel;
         if i == 1
            delay = delay-Cdelay; % no V2N delay in the first cluster
            cost = cost - Ccost; % no V2N cost in the first cluster
         end
            rdel_con(i,1) = delay*1e3;
            rcos_con(i,1) = cost;
    end
    Relay_loc{t+1,1} = rloc_con;
    Relay_idx{t+1,1} = ridx_con;
    Relay_del{t+1,1} = rdel_con;
    Relay_cos{t+1,1} = rcos_con;
    Relay_rat{t+1,1} = rrat_con;
    Relay_rel{t+1,1} = rrel_con;
end

%% Results visualization
% for i = 1:NUM
%     figure(i)
%     time_delay = zeros(size(Relay_del,1),1);
%     for j = 1:size(Relay_del,1)
%         time_delay(j) = Relay_del{j,1}(i);
%     end
%     plot(time_delay,'-square');
% end
figure(2)
delay_table2 = zeros(NUM, timeGap);
for i = 1:NUM
    for j = 1:size(Relay_del,1)
        delay_table2(i,j) = Relay_del{j,1}(i);
    end
end
for i = 1:NUM
    plot(delay_table2(i,:), '-square');
    hold on; 
end
legend('C1', 'C2', 'C3', 'C4','C5', 'C6','C7', 'C8')
ylim([1,10]);
xlim([1,10]);
xlabel("Timestamp (s)");
ylabel("E2E delay (ms)");
grid on;
