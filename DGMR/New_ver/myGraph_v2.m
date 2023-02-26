% Generate two sets of graph: Graph1 - use delay_cost as edge weight;
% Graph2 - use pdr as edge weight
function [Graph1, Graph2] = myGraph_v2(Locations, QoS_th, Ch_par, refDistance)

RAT_th = QoS_th(2);
PDR_th = QoS_th(3);
REL_th = QoS_th(4);
DEL_th = QoS_th(5);


pkt = Ch_par(8); % Packet length in bit
tau = Ch_par(9); % Delay constant


loc = Locations;

% Initial Reference parameters
d_max = refDistance(2);


% Graph initialization
gSize = length(loc);
Graph1 = zeros(gSize, gSize); % delay cost graph
Graph2 = zeros(gSize, gSize); % reliability cost graph

% Weight calculation
for i = 1:gSize
    temp = loc - loc(i);
    temp(temp <= 0 | temp>d_max) = inf;
    Graph1(i,:) = temp;
    f1 = find(temp ~= inf, 1);
    f2 = find(temp ~= inf, 1, 'last');

    for j = f1:f2
        [pdr, rate] = linkCal(temp(j), Ch_par);
        Graph1(i,j) = (pkt/rate + tau)* 1e3; % e2e delay;
        Graph2(i,j) = pdr; % e2e delay;
    end
end

    





