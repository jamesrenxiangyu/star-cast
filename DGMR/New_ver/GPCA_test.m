%% Test GPCA-QGR
close all; clear; clc;

%% Channel Parameters Setting
% QoS settings
DEL_th = 20e-3; % E2E delay in sec
SNR_th = 20; % SNR in dB
RAT_th = 80e6; % Data rate in bps
REL_th = 1 - 1e-2; % End-to-end reliability
COV_th = 500; % Coverage requirement in m

QoS_th = [SNR_th; RAT_th; REL_th; DEL_th];

% Channel settings
Pkt = 1600 * 8; % packet size
B = 10e6; % bandwidth in Hz
M = 4; % M-PSK
F = 5e9; % Radio frequency 5GHz
alpha = 3.6; % Path loss component
Xg = 3.7; % Shadowing component in dB
N0 = -95; % Background noise in 10 MHz assuming a noise figure of 9dB (dBm)   
Pt_max = 23; % in mW;
Tau = 2e-3; % Delay constant in sec
beta = 0.06; % beta: traffic density in veh/m. Values tested: 0.06 and 0.12 veh/m
lambda = 10; % lambda: packet transmission frequency in Hz. Values tested: 10 and 25 Hz.

Ch_par = [M, Pt_max, B, alpha, N0, Xg, F, Pkt, Tau, beta, lambda];

%% Network Settings

% Read topology
Coverage = 5000;   % Coverage in meters
Veh_max = 100; % Number of vehicles on road
Lane_num = 4; % Number of lanes

Loc = [];
% s = RandStream('dsfmt19937', 'Seed', 3);
for i = 1:Lane_num
   Veh_num = randi([floor(Veh_max/2), Veh_max], 1);
   loc_y = 4.5 * i * ones(Veh_num, 1); % Vehicle y locations
   loc_x = randperm(Coverage, Veh_num); % Vehicle x locations
   spd = 25 + 10 * rand([Veh_num, 1]); % Vehicle speed
   loc = [loc_x', loc_y, spd];
   Loc = [Loc; loc];
end
Loc = sortrows(Loc);
Loc = [(1:size(Loc,1))', Loc];

% Network table
% Output: dist_table structure
% layer_id, v_idx, v_loc, v_spd, dist
Loc_table = get_distTable(Loc);


% Network density
dens_th = 30; % inter-vehicle distance
dens = mean(cell2mat(Loc_table(:,end)));
if dens > dens_th
    flag_dens = 0; % dense network
else
    flag_dens = 1; % sparse network
end
 

%% Reference Distance Calculation
dis_snr = get_SNRDist(QoS_th, Ch_par, flag_dens);
dis_rate = get_RateDist(QoS_th, Ch_par, flag_dens);

dmax = min(dis_snr, dis_rate);
dDel = COV_th * (Pkt + Tau * RAT_th)/(DEL_th * RAT_th);
dmin = dDel;

if dmin > dmax
    disp('Service Error, adjust QoS');
end
refDis = [dmin, dmax];

%% Cluster network 
sampleTime = 5;
refDistance = 50;
clusterMode = 1; % use revised kmeans

[nCluster, Cluster] = get_Cluster(Loc_table, sampleTime, refDistance, COV_th, clusterMode);

%% Routing algorithm

NUM = nCluster; % total number of clusters
HOP_MAX = ceil(DEL_th/Tau); % max number of hops per cluster

% initialize result table
% Relays (cell): v_id, loc_x, loc_y, spd; 
% Delay: per-hop delay, end-to-end delay (first hop delay V2N2V); 
% Reliability: per-hop rel, end-to-end rel (first hop reliability V2N2V); 
% Cost: per-hop cost, end-to-end cost (first hop cost V2N2V); 
Results = {'Relays', 'Delay', 'Reliability', 'Throughput', 'Cost'};
result_con = cell(NUM, size(Results,2));
result_con = [Results; result_con];

% V2N2V delay metric
Cdelay = 2 * Pkt/RAT_th * 1e3; % in ms
Ccost = 2 * 5;
Vcost = 2 * 1;

% Relay selection for each cluster
for i = 2:NUM
    temp_Cluster = Cluster{i,1}; % Nx4 array 'v_id, loc_x, loc_y, spd'
    temp_CH = Cluster{i,2}; % 1x3 vector 'v_id, loc_x, loc_y' 
    % find the size of cluster
    fg = size(temp_Cluster,1);
    % initialize containers for QoS metric per hop; last value is e2e QoS
    temp_relay_f = [];
    temp_del_f = [];
    temp_rel_f = [];
    temp_th_f = [];
    temp_cost_f = [];
    
    temp_relay_b = [];
    temp_del_b = [];
    temp_rel_b = [];
    temp_th_b = [];
    temp_cost_b = [];
    
    if fg == 1
        % single member cluster
        if i == 2
            % source is single cluster
            temp_relay_f = temp_Cluster;
            temp_rel_f = 1.0;
            temp_del_f = 0;
            temp_cost_f =  0;
            temp_th_f =  RAT_th;
        else
            % Cluster size == 1
            disp('Use V2I2V unicast');
            temp_relay_f = temp_Cluster;
            temp_rel_f = 1.0;
            temp_del_f = Cdelay;
            temp_cost_f =  Ccost;
            temp_th_f =  RAT_th;
        end
    else
        % multiple member cluster
        if i == 2
            % add first-hop delay
            temp_del_f = [temp_del_f; 0];
            temp_cost_f = [temp_cost_f; 0];
            temp_rel_f = [temp_rel_f; 1.0];
            temp_del_b = [temp_del_b; 0];
            temp_cost_b = [temp_cost_b; 0];
            temp_rel_b = [temp_rel_b; 1.0];
        else
            temp_del_f = [temp_del_f; Cdelay];
            temp_cost_f = [temp_cost_f; Ccost];
            temp_rel_f = [temp_rel_f; 1.0];
            temp_del_b = [temp_del_b; Cdelay];
            temp_cost_b = [temp_cost_b; Ccost];
            temp_rel_b = [temp_rel_b; 1.0];
        end
        front = temp_Cluster(1, :);
        back = temp_Cluster(end, :);
        ch = temp_CH;
        
        % CH is the front vehicle of the cluster
        if isequal(ch(1), front(1)) 
            ctype = 1;
            % scalarize location of each cluster
            loc = temp_Cluster(:, 2:3) - front(1, 2:3); 
            temp_Cluster(:,2:3) = loc;
            tempLoc = temp_Cluster;
            tempCov = norm(back(1,2:3) - front(1,2:3));
        % CH is the back vehicle of the cluster
        elseif isequal(ch(1), back(1)) 
            ctype = 2;
            loc = abs(temp_Cluster(:, 2:3) - back(1, 2:3));
            loc = flip(loc);
            temp_Cluster(:,2:3) = loc;
            tempLoc = temp_Cluster;
            tempCov = norm(back(1,2:3) - front(1,2:3));
        else % CH is in the middel of the cluster
            ctype = 3;
            loc = abs(temp_Cluster(:, 2:3) - back(1, 2:3));
            loc = flip(loc);
            temp_Cluster(:,2:3) = loc;
            tempLoc = temp_Cluster;
            fch = find(tempLoc(:,1) == ch(1));
            tempLoc1 = tempLoc(fch:end, :); % forward
            tempLoc2 = flip(tempLoc(1:fch, :)); % backward
            tempCov1 = norm(back(1,2:3) - ch(1, 2:3)); % forward coverage
            tempCov2 = norm(ch(1,2:3) - front(1, 2:3)); % backward coverage
        end
        
        switch ctype
            case 1
                % Forward part
                 % Method 1 dijkstra.m
                graph = get_myGraph(tempLoc, tempCov, QoS_th, Ch_par, refDis);
                [~, rCan] = get_dijkstra(graph, 1, size(tempLoc,1)); 
                % Method 2 dijkstra_v2.m
    %             [g1, g2] = myGraph_v2(tempLoc1, QoS_th, Ch_par, refDis);
    %             [cost1, rCan1] = dijkstra_v2(g1, g2, 1, length(tempLoc1), QoS_ttemp_rel_f = [temp_rel_f; e_rel_f];h);
                % Map relay routes to v_id
                rCanTable = get_route2Table(rCan, tempLoc);
                temp_relay_f = rCanTable;
                rCanDist = rCanTable(1:end-1, end);
                for rv = 1:length(rCanDist)
                    [rpdr, rrate, ~]= get_linkCal(rCanDist(rv), Ch_par, RAT_th);
                    temp_rel_f = [temp_rel_f; rpdr];
                    temp_th_f = [temp_th_f; rrate];
                    temp_cost_f = [temp_cost_f; Vcost];
                    rdelay = (Pkt/rrate + Tau) * 1e3;
                    temp_del_f = [temp_del_f; rdelay];
                end
                
                e_rel_f = prod(temp_rel_f);
                e_th_f = min(temp_th_f);
                e_cost_f = sum(temp_cost_f);
                e_del_f = sum(temp_del_f);
                
                temp_rel_f = [temp_rel_f; e_rel_f];
                temp_th_f = [temp_th_f; e_th_f];
                temp_cost_f = [temp_cost_f; e_cost_f];
                temp_del_f = [temp_del_f; e_del_f];
                
            case 2
                % Backward part (involve CH)
                 % Method 1 dijkstra.m
                graph = get_myGraph(tempLoc, tempCov, QoS_th, Ch_par, refDis);
                [~, rCan] = get_dijkstra(graph, 1, size(tempLoc,1)); 
                % Method 2 dijkstra_v2.m
    %             [g1, g2] = myGraph_v2(tempLoc1, QoS_th, Ch_par, refDis);
    %             [cost1, rCan1] = dijkstra_v2(g1, g2, 1, length(tempLoc1), QoS_th);
                % Map relay routes to v_id
                rCanTable = get_route2Table(rCan, tempLoc);
                temp_relay_b = rCanTable;
                rCanDist = rCanTable(1:end-1, end);
                for rv = 1:length(rCanDist)
                    [rpdr, rrate, ~]= get_linkCal(rCanDist(rv), Ch_par, RAT_th);
                    temp_rel_b = [temp_rel_b; rpdr];
                    temp_th_b = [temp_th_b; rrate];
                    temp_cost_b = [temp_cost_b; Vcost];
                    rdelay = (Pkt/rrate + Tau) * 1e3;
                    temp_del_b = [temp_del_b; rdelay];
                end

                e_rel_b = prod(temp_rel_b);
                e_th_b = min(temp_th_b);
                e_cost_b = sum(temp_cost_b);
                e_del_b = sum(temp_del_b);
                
                temp_rel_b = [temp_rel_b; e_rel_b];
                temp_th_b = [temp_th_b; e_th_b];
                temp_cost_b = [temp_cost_b; e_cost_b];
                temp_del_b = [temp_del_b; e_del_b];
                
            case 3
                % Forward part
                 % Method 1 dijkstra.m
                graph_f = get_myGraph(tempLoc1, tempCov1, QoS_th, Ch_par, refDis);
                [~, rCan] = get_dijkstra(graph_f, 1, size(tempLoc1,1)); 
                % Method 2 dijkstra_v2.m
    %             [g1, g2] = myGraph_v2(tempLoc1, QoS_th, Ch_par, refDis);
    %             [cost1, rCan1] = dijkstra_v2(g1, g2, 1, length(tempLoc1), QoS_ttemp_rel_f = [temp_rel_f; e_rel_f];h);
                % Map relay routes to v_id
                rCanTable_f = get_route2Table(rCan, tempLoc1);
                temp_relay_f = rCanTable_f;
                rCanDist_f = rCanTable_f(1:end-1, end);
                for rv = 1:length(rCanDist_f)
                    [rpdr, rrate, ~]= get_linkCal(rCanDist_f(rv), Ch_par, RAT_th);
                    temp_rel_f = [temp_rel_f; rpdr];
                    temp_th_f = [temp_th_f; rrate];
                    temp_cost_f = [temp_cost_f; Vcost];
                    rdelay = (Pkt/rrate + Tau) * 1e3;
                    temp_del_f = [temp_del_f; rdelay];
                end
                
                e_rel_f = prod(temp_rel_f);
                e_th_f = min(temp_th_f);
                e_cost_f = sum(temp_cost_f);
                e_del_f = sum(temp_del_f);
                
                temp_rel_f = [temp_rel_f; e_rel_f];
                temp_th_f = [temp_th_f; e_th_f];
                temp_cost_f = [temp_cost_f; e_cost_f];
                temp_del_f = [temp_del_f; e_del_f];

                % Backward part (involve CH)
                 % Method 1 dijkstra.m
                graph_b = get_myGraph(tempLoc2, tempCov1, QoS_th, Ch_par, refDis);
                [~, rCan] = get_dijkstra(graph_b, 1, size(tempLoc2,1)); 
                % Method 2 dijkstra_v2.m
    %             [g1, g2] = myGraph_v2(tempLoc1, QoS_th, Ch_par, refDis);
    %             [cost1, rCan1] = dijkstra_v2(g1, g2, 1, length(tempLoc1), QoS_th);
                % Map relay routes to v_id
                rCanTable_b = get_route2Table(rCan, tempLoc2);
                temp_relay_b = rCanTable_b;
                rCanDist_b = rCanTable_b(1:end-1, end);
                for rv = 1:length(rCanDist_b)
                    [rpdr, rrate, ~]= get_linkCal(rCanDist_b(rv), Ch_par, RAT_th);
                    temp_rel_b = [temp_rel_b; rpdr];
                    temp_th_b = [temp_th_b; rrate];
                    temp_cost_b = [temp_cost_b; Vcost];
                    rdelay = (Pkt/rrate + Tau) * 1e3;
                    temp_del_b = [temp_del_b; rdelay];
                end

                e_rel_b = prod(temp_rel_b);
                e_th_b = min(temp_th_b);
                e_cost_b = sum(temp_cost_b);
                e_del_b = sum(temp_del_b);
                
                temp_rel_b = [temp_rel_b; e_rel_b];
                temp_th_b = [temp_th_b; e_th_b];
                temp_cost_b = [temp_cost_b; e_cost_b];
                temp_del_b = [temp_del_b; e_del_b];
                
                % Aggregate forward and backward               
                
                
        end
    end
    
    result_con{i,1} = {temp_relay_f; temp_relay_b};
    result_con{i,2} = {temp_del_f; temp_del_b};
    result_con{i,3} = {temp_rel_f; temp_rel_b};
    result_con{i,4} = {temp_th_f; temp_th_b};
    result_con{i,5} = {temp_cost_f; temp_cost_b};

end


%% Results visualization
for j = 2:NUM
    
    % visualize relay locations per cluster
    figure(1)
    relay_loc = result_con{j,1};
    % delete empty cell
    relay_loc = relay_loc(~cellfun(@isempty, relay_loc(:,1)), :);
    for clu = 1:size(relay_loc, 1)
        temp_relay_loc = relay_loc{clu,1};
        plot(temp_relay_loc(:,2), temp_relay_loc(:,3), '*');
        hold on;
    end 
    
    % visualize e2e delay per cluster
    figure(2)
    relay_delay = result_con{j,2};
    % delete empty cell
    relay_delay = relay_delay(~cellfun(@isempty, relay_delay(:,1)), :);
    temp_relay_delay = 0;
    for clu = 1:size(relay_delay, 1)
        % use maximum e2e delay of front and back transmission
        temp_relay_delay = max(temp_relay_delay, relay_delay{clu,1}(end));
    end
    bar(j, temp_relay_delay);
    hold on;
    
    % visualize e2e reliability per cluster
    figure(3)
    relay_rel = result_con{j,3};
    % delete empty cell
    relay_rel = relay_rel(~cellfun(@isempty, relay_rel(:,1)), :);
    temp_relay_rel = 1;
    for clu = 1:size(relay_rel, 1)
        % use minimum e2e reliability of front and back transmission
        temp_relay_rel = min(temp_relay_rel, relay_rel{clu,1}(end));
    end
    bar(j, temp_relay_rel);
    hold on;
    
    % visualize e2e throughput per cluster
    figure(4)
    relay_th = result_con{j,4};
    % delete empty cell
    relay_th = relay_th(~cellfun(@isempty, relay_th(:,1)), :);
    temp_relay_th = inf;
    for clu = 1:size(relay_th, 1)
        % use minimum e2e throughput of front and back transmission
        temp_relay_th_temp = min(relay_th{clu,1}(:));
        temp_relay_th = min(temp_relay_th, temp_relay_th_temp);
    end
    bar(j, temp_relay_th);
    hold on;
    
    % visualize e2e cost per cluster
    figure(5)
    relay_cost = result_con{j,5};
    temp_relay_delay = 0;
    for clu = 1:size(relay_delay, 1)
        % use aggregated e2e cost of front and back transmission
        temp_relay_delay = temp_relay_delay + relay_delay{clu,1}(end);
    end
    % remove redudant V2N2V cost
    temp_relay_delay = temp_relay_delay - (clu-1) * Cdelay;
    bar(j, temp_relay_delay);
    hold on;
    
end

figure(2);
ylabel("E2e Delay (ms)");
xlabel("Cluster number");
xticks(0:NUM)

figure(3);
ylabel("Reliability");
xlabel("Cluster number");
ylim([0.9,1]);
xticks(0:NUM);

figure(4);
ylabel("E2E throughput");
xlabel("Cluster number");
xticks(0:NUM);

figure(5);
ylabel("E2E Cost");
xlabel("Cluster number");
xticks(0:NUM);

for j = 2:NUM
    % visualize relay locations per cluster
    figure(1)
    relay_loc = Cluster{j,1};
    plot(relay_loc(:,2), relay_loc(:,3), 'o');
    hold on;
    CH_loc = Cluster{j,2};
    plot(CH_loc(2), CH_loc(3), '*', 'MarkerSize', 10, 'LineWidth', 2.0);
end
grid on;
ylim([0,20]);










