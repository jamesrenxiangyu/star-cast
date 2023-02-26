%% GPCA: Compute the routing path for each cluster
function result_con = get_GPCA(Cluster, NUM, Ch_par, QoS_th, refDis, src_id)
% Input: Cluster: cluster member information
%        NUM: number of clusters
%        Ch_par: channel parameters
%        refDis: reference distance
%        src_id: v_id of source vehicle
%
% Output: result_con contains:
%        Relays (cell): v_id, loc_x, loc_y, spd; 
%        Delay: per-hop delay, end-to-end delay (first hop delay V2N2V); 
%        Reliability: per-hop rel, end-to-end rel (first hop reliability V2N2V); 
%        Cost: per-hop cost, end-to-end cost (first hop cost V2N2V); 

Pkt = Ch_par(8); % packet size
Tau = Ch_par(9); % fixed per-hop delay const

RAT_th = QoS_th(2); % data rate requirement

% initialize result table
% Relays (cell): v_id, loc_x, loc_y, spd; 
% Delay: per-hop delay, end-to-end delay (first hop delay V2N2V); 
% Reliability: per-hop rel, end-to-end rel (first hop reliability V2N2V);
% Throughput: minimum data rate achieved along the path
% Cost: per-hop cost, end-to-end cost (first hop cost V2N2V); 
Results = {'Relays', 'Delay', 'Reliability', 'Throughput', 'Cost'};
result_con = cell(5, size(Results,2));
result_con = [Results; result_con];

% V2N2V delay metric
Cdelay = (2 * Pkt/RAT_th + Tau) * 1e3; % in ms
Ccost = 2 * 5;
Vcost = 2 * 1;

% Compute routes for each cluster
for i = 2:NUM + 1
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
    
    
    if ~isempty(find(temp_Cluster(:,1) == src_id, 1))
        src_flag = 1;
    else
        src_flag = 0;
    end
    
    if fg == 1
        
        % single member cluster
        if src_flag == 1
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
        if src_flag == 1
            % src vehicle within cluster
            temp_del_f = [temp_del_f; 0];
            temp_cost_f = [temp_cost_f; 0];
            temp_rel_f = [temp_rel_f; 1.0];
            temp_del_b = [temp_del_b; 0];
            temp_cost_b = [temp_cost_b; 0];
            temp_rel_b = [temp_rel_b; 1.0];
        else
            % add first-hop delay
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
%             loc = temp_Cluster(:, 2:3) - front(1, 2:3); 
%             temp_Cluster(:,2:3) = loc;
            tempLoc = temp_Cluster;
            tempCov = norm(back(1,2:3) - front(1,2:3));
        % CH is the back vehicle of the cluster
        elseif isequal(ch(1), back(1)) 
            ctype = 2;
%             loc = abs(temp_Cluster(:, 2:3) - back(1, 2:3));
%             loc = flip(loc);
%             temp_Cluster(:,2:3) = loc;
            tempLoc = temp_Cluster;
            tempCov = norm(back(1,2:3) - front(1,2:3));
        else % CH is in the middel of the cluster
            ctype = 3;
%             loc = abs(temp_Cluster(:, 2:3) - back(1, 2:3));
%             loc = flip(loc);
%             temp_Cluster(:,2:3) = loc;
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