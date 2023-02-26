function result_LPCA = get_LPCA(input_data_all, result_con, Cluster, NUM, Ch_par, QoS_th, refDis, src_id, timeGap, init_time)
% Input: input_data - gloable network mobility information
%        result_con - GPCA routing results
%        Cluster - GPCA cluster results   
%        NUM - number of clusters
%        Ch_par, QoS_th, refDis - Network information
%        src_id - v_id of source vehicle
%        timeGap - time gap to update network topology
%        init_time - time slot to start LPCA
%
% Output: result_LPCA - LPCA re-routing decisions for each cluster at each
%         time slot.


Pkt = Ch_par(8); % packet size
Tau = Ch_par(9); % fixed per-hop delay const
RAT_th = QoS_th(2);

% V2N2V delay metric
Cdelay = (2 * Pkt/RAT_th + Tau) * 1e3; % in ms
Ccost = 2 * 5;
Vcost = 2 * 1;


% initialize result table
% Relays (cell): v_id, loc_x, loc_y, spd; 
% Delay: per-hop delay, end-to-end delay (first hop delay V2N2V); 
% Reliability: per-hop rel, end-to-end rel (first hop reliability V2N2V); 
% Cost: per-hop cost, end-to-end cost (first hop cost V2N2V); 

Cluster = Cluster(2:end, :);

title = {'Relays', 'Delay', 'Reliability', 'Throughput', 'Cost'};
result_GPCA = result_con(2:end,:);
result_LPCA = cell(timeGap, size(title,2));
for i = 1:5
    result_LPCA{1,i} = result_GPCA(:,i);
end
result_LPCA = [title; result_LPCA];

% assume cluster remain the same within timeGap time slots, 
% i.e., the same input Cluster is used for each time slot
for t = 1:timeGap
    sprintf("Iterating %d time slot", t)
    % load vehicle information at time: time+t
    input_data = input_data_all{init_time + t, 2};
%     input_data = Cluster;
    
    % for storing routing result for each time slot
    temp_relay = cell(NUM,1);
    temp_delay = cell(NUM,1);
    temp_relia = cell(NUM,1);
    temp_throu = cell(NUM,1);
    temp_cost = cell(NUM,1);
    
    % compute for each cluster
    for c = 1:NUM
        sprintf("Iterating %d clusther", c)
        % cluster_ids and cluster heads are not changed
        tempCluster_id = Cluster{c, 1}(:, 1);
        % check if src vehicle is in this cluster: 1-yes, 0-no
        flag = ~isempty(find(tempCluster_id == src_id, 1));
                
        % 'v_id'
        tempCH = Cluster{c,2}(1);
        % 'v_id, loc_x, loc_y, spd'
        tempCluster_member = [];
        ids = cell2mat(input_data(2:end,1));
        for m = 1:size(tempCluster_id,1)
            id_flag = find(ids == tempCluster_id(m)) + 1;
            tempMem = cell2mat(input_data(id_flag, :));
            tempCluster_member = [tempCluster_member; tempMem];
        end
%         tempCluster_member = cell2mat(input_data(c,1));
        
        % sort vehicle information by loc_x
        if isempty(tempCluster_member)
            warning('Error! Recluster');
            break;
        end
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
            old_relay = old_relay(:,1);
                      
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
        temp_relay{c,1} = new_relay;
        temp_delay{c,1} = new_delay;
        temp_relia{c,1} = new_relia;
        temp_throu{c,1} = new_throu;
        temp_cost{c,1} = new_cost;
        
    end
    result_LPCA{t+2, 1} = temp_relay;
    result_LPCA{t+2, 2} = temp_delay;
    result_LPCA{t+2, 3} = temp_relia;
    result_LPCA{t+2, 4} = temp_throu;
    result_LPCA{t+2, 5} = temp_cost;
end