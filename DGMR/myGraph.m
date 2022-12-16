% Use log(delay_cost * pdr_ratio) as edge weight.
function Graph = myGraph(Locations, QoS_th, Ch_par, refDistance)

RAT_th = QoS_th(2);
PDR_th = QoS_th(3);
REL_th = QoS_th(4);
DEL_th = QoS_th(5);

pkt = Ch_par(8); % Packet length in bit
tau = Ch_par(9); % Delay constant


loc = Locations;
cov = loc(end) - loc(1);

% Initial Reference parameters
d_min = refDistance(1);
d_max = refDistance(2);
hmin = ceil(cov/d_max); % minimum hop count
pmin = exp(log(REL_th)/hmin); % minimum per-hop pdr

% Graph initialization
gSize = length(loc);
Graph = zeros(gSize, gSize); % final graph

% Weight calculation
for i = 1:gSize
    temp = loc - loc(i);
    % Consider all vehicles whoes distance smaller than the threshold
    % Alternative: use d_min as lower bound to reduce edge numbers
    temp(temp <= 0 | temp>d_max) = inf;
    Graph(i,:) = temp;
    f1 = find(temp ~= inf, 1);
    f2 = find(temp ~= inf, 1, 'last');

    for j = f1:f2
        [pdr, rate] = linkCal(temp(j), Ch_par);
        pr = pmin/pdr; % threshold/pdr ratio
        if pr > 1
            pr = pr*10;
        end
        Graph(i,j) = log(pr * (pkt/rate + tau)* 1e3);
%         Graph(i,j) = (PDR_th/pdr) * (pkt/rate + tau)* 1e3; % e2e delay;
    end
end

    





