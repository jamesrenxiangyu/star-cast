%% LPCA: Improved DGMR based on relay selection
function [rCan, Del, Rel] = DGMR_V3(QoS_th, Locations, Speed, refRelay, CH_idx, Ch_par, refDistance)
%
% Input: QoS_th -- SNR, Data rate, per-hop and e2e reliability, delay threshold
%        Locations -- Locations of all vehicles of the cluster. It has size
%                     N x 3, first column is the index, rest are the
%                     x and y locations.  
%        Speed -- Speed of all vehicles, of size N x 1
%        refRelay -- Original relay vehicles indices
%        CH_idx -- Cluster head index
%        Ch_par -- Channel model parameters, per hop delay constant
%        refDistance -- per-hop transmission upper and lower bound
% 
% Output: rCan -- relay vehicles (path)
%         Del -- end-to-end delay
%         Rel -- end-to-end reliability

locRaw = Locations;
X = [locRaw, Speed(:,2)];
X = sortrows(X, 2); % sort locations by x-axis
loc = X(:, 1:3);
spd = X(:, [1 4]);
% Coverage = max(loc(:,2)) - min(loc(:,1));
% h_loc = loc(find(loc(:,1)==CH_idx), 2:end);

PDR_th = QoS_th(3);
REL_th = QoS_th(4);
DEL_th = QoS_th(5);

pkt = Ch_par(8); % Packet length in bit
tau = Ch_par(9); % Delay constant

dmin = refDistance(1); % distance lower bound 
dmax = refDistance(2); % distance upper bound

%% Divide the cluster depending on the location of the CH
cflag = find(loc(:,1)==CH_idx); % find the location of the CH in the cluster
l = [loc(:,1) loc(:,2:end)-loc(cflag,2:end)]; % Normalize locations
if cflag == 1 || cflag == size(loc,1)) % CH is at the back edge of the cluster
    l1 = l;
    s1 = spd;
    l2 = [];
    s2 = [];
elseif cflag == size(loc,1)) % CH is at the front edge of the cluster
    l1 = flip(l);
    s1 = flip(s);
    l2 = [];
    s2 = [];
else % CH is in the middle of the cluster
    l1 = l(cflag:end, :);
    s1 = l(clag:end, :);
    l2 = flip(l(1:cflag,:));
    s2 = flip(s(1:cflag,:));
end    
%% Execute relay selection for each hop (Whole path)

 % Update delay and reliability budget
 
%% Output calculation



