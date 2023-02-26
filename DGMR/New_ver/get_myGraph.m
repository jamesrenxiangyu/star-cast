% Use log(delay_cost * pdr_ratio) as edge weight.
function Graph = get_myGraph(Locations, tempCov, QoS_th, Ch_par, refDistance)

RAT_th = QoS_th(2);
REL_th = QoS_th(3);



pkt = Ch_par(8); % Packet length in bit
tau = Ch_par(9); % Delay constant
loc = Locations(:,2:3);

cov = tempCov;

% Initial Reference parameters
d_max = refDistance(2);

hmin = ceil(cov/d_max); % minimum hop count
pmin = exp(log(REL_th)/hmin); % minimum per-hop pdr

% Graph initialization
gSize = size(loc,1);
Graph = inf * ones(gSize, gSize); % final graph

% Weight calculation
for i = 1:gSize-1
    % Consider all vehicles whoes distance smaller than the threshold
    % Alternative: use d_min as lower bound to reduce edge numbers

    for j = i+1:gSize
        % find inter-vehicle distance
        temp_dist = norm(loc(j,:) - loc(i,:));
        [temp_pdr, temp_rate, ~]= get_linkCal(temp_dist, Ch_par, RAT_th);
        norm_pdr = pmin/temp_pdr;
        % filter out bad paths
        if norm_pdr > 1
            norm_pdr = 1000000;
        end
        Graph(i,j) = log(norm_pdr * (pkt/temp_rate + tau)* 1e3);
    end

end

    





