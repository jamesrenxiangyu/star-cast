%% NPCA: Without re-routing
function [new_relay, new_delay, new_relia, new_throu, new_cost] = get_npcaResults(QoS_th, subMember, refRelay, CH_idx, Ch_par, refDistance)
%
% Input: QoS_th -- SNR, Data rate, per-hop and e2e reliability, delay threshold
%        subMember -- Locations and speed of vehicles of the same Tx direction in the cluster: v_id, loc_x, loc_y, speed
%        refRelay -- Original relay vehicles v_id
%        CH_idx -- CH of the subMember to locate first vehicle
%        Ch_par -- Channel model parameters, per hop delay constant
%        refDistance -- per-hop transmission upper and lower bound
% 
% Output: new_relay -- new relay vehicles (path)
%         new_delay -- per-hop delay
%         new_relia -- per-hop reliability
%         new_throu -- per-hop throughput
%         new_cost -- per-hop cost

%% initialize network pararmeters

new_relay = [];
new_delay = [];
new_relia = [];
new_throu = [];
new_cost = [];

% get current locations of relay vehicles
subMember = sortrows(subMember, 2);
% 'v_id, loc_x, loc_y, spd'
relays = zeros(length(refRelay), 4);
for i = 1:length(refRelay)
    relay_flag = find(subMember(:,1)==refRelay(i),1);
    if isempty(relay_flag)
        warning('Relay out of cluster');
%         return;
    else
        relays(i,:) = subMember(relay_flag, :);
    end
end
% sort by loc_x
relays = sortrows(relays,2);

% memCoverage = abs(max(subMember(:,2)) - min(subMember(:,2)));

RAT_th = QoS_th(2);

Pkt = Ch_par(8); % Packet length in bit
tau = Ch_par(9); % Delay constant

dmin = refDistance(1);
dmax = refDistance(2);


% V2N2V delay metric
Vcost = 2 * 1;

%% Compute results

% ** CH should be within relays
src_flag = find(subMember(:,1) == CH_idx, 1);
src_flag_relay = find(relays(:,1) == CH_idx, 1);

if isempty(src_flag_relay)
    % CH not in the relay
    warning('Error! Recluster');
    return;
end

if isempty(src_flag)
    % CH not in the subMember
    flag = 0;
    warning('Error! Recluster');
else
    if src_flag_relay == 1
        % CH is in the front of the relays
        flag = 1;
    elseif src_flag_relay == size(relays,1)
        % CH is in the back of the relays
        flag = 2;
        % flip for easier computation
        relays = flip(relays);
        subMember = flip(subMember);
    else
        % CH is in the middle of the subMember
        flag = 3;
    end
end

% compute result for different cases
switch flag
    case 0
        new_relay = [];
        new_delay = 100;
        new_relia = 0;
        new_cost = 0;
        new_throu = 0;
        return
    case {1,2}
        for m = 1:size(relays,1)-1
            temp_dis = norm(relays(m,2:3) - relays(m+1,2:3));
            [temp_pdr, temp_rate, ~]= get_linkCal(temp_dis, Ch_par, RAT_th);
            temp_delay = (Pkt/temp_rate + tau) * 1e3; % in ms
            new_delay = [new_delay, temp_delay];
            new_relia = [new_relia, temp_pdr];
            new_throu = [new_throu, temp_rate];
            new_cost = [new_cost, Vcost];
        end
        new_relay = relays;
        % compute pre-first hop
        if relays(1,1) == subMember(1,1)
            disp('Reach front edge');
        else
            disp('one-hop forward');
            temp_dis = norm(subMember(1,2:3) - relays(1, 2:3));
            if temp_dis > dmax + 50
                temp_pdr = 0.001;
                temp_rate = 1e6;
            else
                [temp_pdr, temp_rate, ~]= get_linkCal(temp_dis, Ch_par, RAT_th);
            end
            temp_delay = (Pkt/temp_rate + tau) * 1e3;
            new_delay = [temp_delay, new_delay];
            new_relia = [temp_pdr, new_relia];
            new_throu = [temp_rate, new_throu];
            new_cost = [Vcost, new_cost];   
        end
           
        
        % compute last hop
        if relays(end,1) == subMember(end,1)
            disp('Reach back edge');
        else
            disp('one-hop backward');
            temp_dis = norm(subMember(end,2:3) - relays(end, 2:3));
            if temp_dis > dmax + 50
                temp_pdr = 0.001;
                temp_rate = 1e6;
            else
                [temp_pdr, temp_rate, ~]= get_linkCal(temp_dis, Ch_par, RAT_th);
            end
            temp_delay = (Pkt/temp_rate + tau) * 1e3;
            new_delay = [new_delay, temp_delay];
            new_relia = [new_relia, temp_pdr];
            new_throu = [new_throu, temp_rate];
            new_cost = [new_cost, Vcost];
%             if  dmax >= temp_dis
%             % last hop within coverage
%                 [temp_pdr, temp_rate, ~]= get_linkCal(temp_dis, Ch_par, RAT_th);
%                 temp_delay = Pkt/temp_rate + tau;
%                 new_delay = [new_delay, temp_delay];
%                 new_relia = [new_relia, temp_pdr];
%                 new_throu = [new_throu, temp_rate];
%                 new_cost = [new_cost, Vcost];
%             else
%             % last hop out of coverage
%                 [temp_pdr, temp_rate, ~]= get_linkCal(temp_dis, Ch_par, RAT_th);
%                 temp_delay = Pkt/temp_rate + tau;
%                 new_delay = [new_delay, temp_delay];
%                 new_relia = [new_relia, temp_pdr];
%                 new_throu = [new_throu, temp_rate];
%                 new_cost = [new_cost, Vcost];     
        end
           
        
    case 3
        forw_mem = subMember(src_flag:end, :);
        forw_rel = relays(src_flag_relay:end, :);
        back_mem = flip(subMember(1:src_flag, :));
        back_rel = flip(relays(1:src_flag_relay, :));
        
        if isempty(forw_mem)
            warning('Error');
        end
        
        new_relay_f = forw_rel;
        new_relay_b = back_rel;
        
        % forward transmission
        for m = 1:size(forw_rel,1)-1
            temp_dis = norm(forw_rel(m,2:3) - forw_rel(m+1,2:3));
            if temp_dis > dmax + 50
                temp_pdr = 0.001;
                temp_rate = 1e6;
            else
                [temp_pdr, temp_rate, ~]= get_linkCal(temp_dis, Ch_par, RAT_th);
            end
            temp_delay = (Pkt/temp_rate + tau) * 1e3;
            new_delay_f = [new_delay_f, temp_delay];
            new_relia_f = [new_relia_f, temp_pdr];
            new_throu_f = [new_throu_f, temp_rate];
            new_cost_f = [new_cost_f, Vcost];
        end
        
        % compute last hop
        if forw_rel(end,1) == forw_mem(end,1)
            disp('Reach back edge');
        else
            disp('one-hop backward');
            temp_dis = norm(forw_mem(end,2:3) - forw_rel(end, 2:3));
            if temp_dis > dmax + 50
                temp_pdr = 0.001;
                temp_rate = 1e6;
            else
                [temp_pdr, temp_rate, ~]= get_linkCal(temp_dis, Ch_par, RAT_th);
            end
            temp_delay = (Pkt/temp_rate + tau) * 1e3;
            new_delay_f = [new_delay_f, temp_delay];
            new_relia_f = [new_relia_f, temp_pdr];
            new_throu_f = [new_throu_f, temp_rate];
            new_cost_f = [new_cost_f, Vcost];  
        end
            
        
        % backward transmission
        for m = 1:size(back_rel,1)-1
            temp_dis = norm(back_rel(m,2:3) - back_rel(m+1,2:3));
            if temp_dis > dmax + 50
                temp_pdr = 0.001;
                temp_rate = 1e6;
            else
                [temp_pdr, temp_rate, ~]= get_linkCal(temp_dis, Ch_par, RAT_th);
            end
            temp_delay = (Pkt/temp_rate + tau) * 1e3;
            new_delay_b = [new_delay_b, temp_delay];
            new_relia_b = [new_relia_b, temp_pdr];
            new_throu_b = [new_throu_b, temp_rate];
            new_cost_b = [new_cost_b, Vcost];
        end
        
        % compute last hop
        if back_rel(end,1) == back_mem(end,1)
            disp('Reach back edge');
        else
            disp('one-hop backward');
            temp_dis = norm(back_mem(end,2:3) - back_rel(end, 2:3));
            if temp_dis > dmax + 50
                temp_pdr = 0.001;
                temp_rate = 1e6;
            else
                [temp_pdr, temp_rate, ~]= get_linkCal(temp_dis, Ch_par, RAT_th);
            end
            temp_delay = (Pkt/temp_rate + tau) * 1e3;
            new_delay_b = [new_delay_b, temp_delay];
            new_relia_b = [new_relia_b, temp_pdr];
            new_throu_b = [new_throu_b, temp_rate];
            new_cost_b = [new_cost_b, Vcost];  
        end
        
        new_relay = {forw_rel; back_rel};
        new_delay = {new_delay_f; new_delay_b};
        new_relia = {new_relay_f; new_relia_b};
        new_throu = {new_throu_f; new_throu_b};
        new_cost = {new_cost_f; new_cost_b};
        
end





