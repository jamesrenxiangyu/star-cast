%% main: Test with SUMO data
close all; clear; clc;

%% Channel Parameters Setting
% QoS settings
DEL_th = 20e-3; % E2E delay in sec
SNR_th = 20; % SNR in dB
RAT_th = 80e6; % Data rate in bps
REL_th = 1 - 1e-2; % End-to-end reliability
COV_th = 500; % Coverage requirement in m

QoS_th = [SNR_th; RAT_th; REL_th; DEL_th; COV_th];

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
Veh_max = 300; % Number of vehicles on road
Lane_num = 4; % Number of lanes

path = 'New_ver/SUMO/';
filename = 'denseTrace';
tarfile = strcat(filename, '.mat');
files = dir(path);
if ~isempty(dir(strcat(path, tarfile)))
    load(tarfile)
else
    csvfile = [path, strcat(filename, '.csv')];
    input_data = get_myData(csvfile);
    save(strcat(path, tarfile), 'input_data');
end

input_data = input_data(cell2mat(input_data(:,end)) == 1, :);


% Generate test data
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

src_id = 1;


%% Set data preferences

init_time = 80;
timeGap = 5;

% V2N2V delay metric
Cdelay = 2 * Pkt/RAT_th * 1e3; % in ms
Ccost = 2 * 5;
Vcost = 2 * 1;


%% Network table
% Output: dist_table structure
% layer_id, v_idx, v_loc, v_spd, dist
Loc = cell2mat(input_data{init_time,2}(2:end,:));

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
sampleTime = timeGap;
refDistance = dmin;
clusterMode = 1; % use revised kmeans

[nCluster, Cluster] = get_Cluster(Loc_table, sampleTime, refDistance, COV_th, clusterMode, src_id);

% % visualize clusters
% for j = 2:nCluster
%     % visualize relay locations per cluster
%     figure(1)
%     relay_loc = Cluster{j,1};
%     plot(relay_loc(:,2), relay_loc(:,3), 'o');
%     CH_loc = Cluster{j,2};
%     plot(CH_loc(2), CH_loc(3), '*', 'MarkerSize', 10, 'LineWidth', 2.0);
%     hold on;
% end

%% GPCA Routing algorithm
NUM = nCluster -1; % total number of clusters

% Output: result_con contains:
%        Relays (cell): v_id, loc_x, loc_y, spd; 
%        Delay: per-hop delay, end-to-end delay (first hop delay V2N2V); 
%        Reliability: per-hop rel, end-to-end rel (first hop reliability V2N2V); 
%        Cost: per-hop cost, end-to-end cost (first hop cost V2N2V); 

result_con_gpca = get_GPCA(Cluster, NUM, Ch_par, QoS_th, refDis, src_id);



%% LPCA Routing algorithm

% result_con_lpca = get_LPCA(input_data, result_con_gpca, Cluster, NUM, Ch_par, QoS_th, refDis, src_id, timeGap, init_time);

result_NPCA = get_NPCA(input_data, result_con_gpca, Cluster, NUM, Ch_par, QoS_th, refDis, src_id, timeGap, init_time);

%% Results visualization
for j = 2:NUM
    
    % visualize relay locations per cluster
    figure(1)
    relay_loc = Cluster{j,1};
    plot(relay_loc(:,2), relay_loc(:,3), 'o');
    CH_loc = Cluster{j,2};
    plot(CH_loc(2), CH_loc(3), '*', 'MarkerSize', 10, 'LineWidth', 2.0);
    hold on;
    
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

figure(1);
ylabel("y-axis (m)");
xlabel("x-axis (m)");
grid on;
ylim([0,20]);

figure(2);
ylabel("E2e Delay (ms)");
xlabel("Cluster number");
xticks(0:NUM)
grid on;

figure(3);
ylabel("Reliability");
xlabel("Cluster number");
ylim([0.9,1]);
xticks(0:NUM);
grid on;

figure(4);
ylabel("E2E throughput");
xlabel("Cluster number");
xticks(0:NUM);
grid on;

figure(5);
ylabel("E2E Cost");
xlabel("Cluster number");
xticks(0:NUM);
grid on;
