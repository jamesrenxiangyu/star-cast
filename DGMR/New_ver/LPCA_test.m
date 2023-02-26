%% Test GPCA-QGR
close all; clear; clc;

%% Channel Parameters Setting
% QoS settings
DEL_th = 20; % E2E delay in ms
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

% % Read topology
% Coverage = 5000;   % Coverage in meters
% Veh_max = 100; % Number of vehicles on road
% Lane_num = 4; % Number of lanes
% 
% Loc = [];
% % s = RandStream('dsfmt19937', 'Seed', 3);
% for i = 1:Lane_num
%    Veh_num = randi([floor(Veh_max/2), Veh_max], 1);
%    loc_y = 4.5 * i * ones(Veh_num, 1); % Vehicle y locations
%    loc_x = randperm(Coverage, Veh_num); % Vehicle x locations
%    spd = 25 + 10 * rand([Veh_num, 1]); % Vehicle speed
%    loc = [loc_x', loc_y, spd];
%    Loc = [Loc; loc];
% end
% Loc = sortrows(Loc);
% Loc = [(1:size(Loc,1))', Loc];
% 
% % Network table
% % Output: dist_table structure
% % layer_id, v_idx, v_loc, v_spd, dist
% Loc_table = get_distTable(Loc);

load GPCAtest_network.mat
load GPCAtest_networkTable.mat

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

% %% Cluster network 
% sampleTime = 5;
% refDistance = 50;
% clusterMode = 1; % use revised kmeans
% 
% [nCluster, Cluster] = get_Cluster(Loc_table, sampleTime, refDistance, COV_th, clusterMode, src_id);
% 
% % load GPCAtest_cluster.mat
% % load GPCAtest_result.mat
% 
% %% GPCA routing
% NUM = nCluster; % total number of clusters
% 
% % Output: result_con contains:
% %        Relays (cell): v_id, loc_x, loc_y, spd; 
% %        Delay: per-hop delay, end-to-end delay (first hop delay V2N2V); 
% %        Reliability: per-hop rel, end-to-end rel (first hop reliability V2N2V); 
% %        Cost: per-hop cost, end-to-end cost (first hop cost V2N2V); 
% 
% result_con_gpca = get_GPCA(Cluster, NUM, Ch_par, QoS_th, refDis);

% V2N2V delay metric
Cdelay = (2 * Pkt/RAT_th + Tau) * 1e3; % in ms
Ccost = 2 * 5;
Vcost = 2 * 1;

load GPCAtest_cluster.mat
load GPCAtest_result.mat

NUM = size(Cluster,1) - 1; % total number of clusters

%% LPCA Routing

HOP_MAX = ceil(DEL_th/Tau); % max number of hops per cluster

src_id = 1;

% result_LPCA = get_LPCA(input_data, result_con, Cluster, NUM, Ch_par, QoS_th, refDis, src_id, timeGap, init_time);




% initialize result table
% Relays (cell): v_id, loc_x, loc_y, spd; 
% Delay: per-hop delay, end-to-end delay (first hop delay V2N2V); 
% Reliability: per-hop rel, end-to-end rel (first hop reliability V2N2V); 
% Cost: per-hop cost, end-to-end cost (first hop cost V2N2V); 
title = {'Relays', 'Delay', 'Reliability', 'Throughput', 'Cost'};
result_GPCA = result_con;
timeGap = 1;
time = 1;
result_LPCA = cell(timeGap, size(title,2));
for i = 1:5
    result_LPCA{1,i} = result_GPCA(:,i);
end
result_LPCA = [title; result_LPCA];

% assume cluster remain the same within timeGap time slots, 
% i.e., the same input Cluster is used for each time slot
for t = 1:timeGap
    % load vehicle information at time: time+t
%     input_data = data{time + t, :};
    input_data = Cluster;
    
    % for storing routing result for each time slot
    temp_relay = cell(NUM,1);
    temp_delay = cell(NUM,1);
    temp_relia = cell(NUM,1);
    temp_throu = cell(NUM,1);
    temp_cost = cell(NUM,1);
    
    % compute for each cluster
    for c = 2:NUM
        % cluster_ids and cluster heads are not changed
        tempCluster_id = Cluster{c, 1}(:, 1);
        % check if src vehicle is in this cluster: 1-yes, 0-no
        flag = ~isempty(find(tempCluster_id == src_id, 1));
                
        % 'v_id'
        tempCH = Cluster{c,2}(1);
        % 'v_id, loc_x, loc_y, spd'
%         tempCluster_member = input_data(input_data{time+t, :}{c,1}(:,1) == tempCluster_id, :);
        tempCluster_member = cell2mat(input_data(c,1));
        
        % sort vehicle information by loc_x
        tempCluster_member = sortrows(tempCluster_member,2);
        % initialize metrics for each 
        

        % size of each cluster
        c_size = size(tempCluster_id, 1);
        if c_size == 1 % Cluster size == 1
            new_relay = cell(2,1);
            new_relia = zeros(2,1);
            new_delay = zeros(2,1);
            new_throu = zeros(2,1);
            new_cost = zeros(2,1);
            disp('Use V2I2V unicast');
            new_relay{1,1} = tempCluster_member;
            if flag == 1
                % single + src cluster
                new_relia(1,1) = 1.0;
                new_delay(1,1) = 0.0;
                new_cost(1,1) = 0;
                new_throu(1,1) = RAT_th;
            else
                % single remote cluster
                new_relia(1,1) = 1.0;
                new_delay(1,1) = Cdelay;
                new_cost(1,1) = Ccost;
                new_throu(1,1) = RAT_th;
            end
        else % Cluster size > 1
            cell_relia = [];
            cell_delay = [];
            cell_throu = [];
            cell_cost = [];
            if flag == 1
                % multiple + src cluster
                cell_relia(:,1) = 1.0;
                cell_delay(:,1) = 0.0;
                cell_cost(:,1) = 0.0;
                cell_throu(:,1) = RAT_th;
            else
                % multiple + remote cluster
                cell_relia(:,1) = 1.0;
                cell_delay(:,1) = Cdelay;
                cell_cost(:,1) = Ccost;
                cell_throu(:,1) = RAT_th;
            end
            
          
            % retrieve relay information (v_id) from previous time slot
            if ~isempty(result_LPCA{t+1, 1}{c,1}{1})
                old_relay_f = result_LPCA{t+1, 1}{c,1}{1}(:,1);
            else
                old_relay_f = [];
            end
            
            if ~isempty(result_LPCA{t+1, 1}{c,1}{2})
                old_relay_b = result_LPCA{t+1, 1}{c,1}{2}(:,1);
            else
                old_relay_b = [];
            end
            old_relay = unique([old_relay_f; old_relay_b]); 
                      
            % re-routing
            [new_relay, new_delay, new_relia, new_throu, new_cost] = get_reRouting(QoS_th, tempCluster_member, old_relay, tempCH, Ch_par, refDis);
            
            for i = 1:2
                if ~isempty(new_delay{i,1})
                    cell_delay_e2e = sum([cell_delay, new_delay{i,1}]);
                    new_delay{i,1} = [cell_delay, new_delay{i,1}, cell_delay_e2e];
                    
                    cell_relia_e2e = prod([cell_relia, new_relia{i,1}]);
                    new_relia{i,1} = [cell_relia, new_relia{i,1}, cell_relia_e2e];
                    
                    cell_throu_e2e = min([cell_throu, new_throu{i,1}]);
                    new_throu{i,1} = [cell_throu, new_throu{i,1}, cell_throu_e2e];
                    
                    cell_cost_e2e = sum([cell_cost, new_cost{i,1}]);
                    new_cost{i,1} = [cell_cost, new_cost{i,1}, cell_cost_e2e];
                end
            end        
        end
        temp_relay{c-1,1} = new_relay;
        temp_delay{c-1,1} = new_delay;
        temp_relia{c-1,1} = new_relia;
        temp_throu{c-1,1} = new_throu;
        temp_cost{c-1,1} = new_cost;
        
    end
    result_LPCA{t+2, 1} = temp_relay;
    result_LPCA{t+2, 2} = temp_delay;
    result_LPCA{t+2, 3} = temp_relia;
    result_LPCA{t+2, 4} = temp_throu;
    result_LPCA{t+2, 5} = temp_cost;
end


%% Results visualization










